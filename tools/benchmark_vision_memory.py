#!/usr/bin/env python3
"""Linux traditional-vision memory benchmark and process-tree profiler; standard library only."""
import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import signal
import statistics
import subprocess
import sys
import time


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            value.update(chunk)
    return value.hexdigest()


def meminfo():
    values = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key, value = line.split(":", 1)
        values[key] = int(value.split()[0])
    return {key: values[key] for key in
            ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree")}


def tree_pids(root):
    pending, found = [root], set()
    while pending:
        pid = pending.pop()
        if pid in found:
            continue
        found.add(pid)
        # Children can have been spawned by any thread, not only the main one.
        for children in Path(f"/proc/{pid}/task").glob("*/children"):
            try:
                pending.extend(map(int, children.read_text().split()))
            except (OSError, ValueError):
                pass  # Processes/threads may exit between enumeration and read.
    return found


def memory_sample(pid, elapsed, measuring):
    total = {key: 0 for key in ("Rss", "Pss", "SwapPss")}
    pids = tree_pids(pid)
    readable, unreadable = 0, 0
    for child in pids:
        try:
            data = Path(f"/proc/{child}/smaps_rollup").read_text()
        except FileNotFoundError:
            continue
        except (PermissionError, ProcessLookupError):
            unreadable += 1
            continue
        values = dict((key, int(value.split()[0]))
                      for key, value in (line.split(":", 1)
                                         for line in data.splitlines() if ":" in line))
        if "Pss" not in values:
            unreadable += 1
            continue
        readable += 1
        for key in total:
            total[key] += values.get(key, 0)
    return {"elapsed_s": elapsed, "measurement_phase": measuring,
            "tree_rss_kib": total["Rss"] if readable else None,
            "tree_pss_kib": total["Pss"] if readable else None,
            "tree_swap_pss_kib": total["SwapPss"] if readable else None,
            "processes_read": readable, "processes_unreadable": unreadable,
            "host": meminfo()}


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    return values[max(0, math.ceil(len(values) * fraction) - 1)]


def send_group(process, sig):
    try:
        os.killpg(process.pid, sig)
        return True
    except ProcessLookupError:
        return False


def live_group_pids(group):
    """Ignore zombies: only their parent can reap them, and they own no memory."""
    found = []
    for stat in Path("/proc").glob("[0-9]*/stat"):
        try:
            # comm can contain spaces and parentheses, so split at its last ')'.
            fields = stat.read_text().rsplit(")", 1)[1].split()
            if int(fields[2]) == group and fields[0] not in ("Z", "X"):
                found.append(int(stat.parent.name))
        except (OSError, ValueError, IndexError):
            pass  # A process may disappear while its stat file is read.
    return sorted(found)


def cleanup_process_group(process, terminate_grace_s=.5, kill_grace_s=.5):
    """Bound cleanup even if a task is stuck in an uninterruptible kernel wait."""
    initial = live_group_pids(process.pid)
    signals = []

    def wait_until(deadline):
        remaining = live_group_pids(process.pid)
        while remaining and time.monotonic() < deadline:
            process.poll()  # Reap our direct child as soon as possible.
            time.sleep(min(.02, max(0, deadline - time.monotonic())))
            remaining = live_group_pids(process.pid)
        return remaining

    remaining = initial
    if remaining:
        if send_group(process, signal.SIGTERM):
            signals.append("SIGTERM")
        remaining = wait_until(time.monotonic() + terminate_grace_s)
    if remaining:
        if send_group(process, signal.SIGKILL):
            signals.append("SIGKILL")
        remaining = wait_until(time.monotonic() + kill_grace_s)
    process.poll()
    return {"initial_live_pids": initial, "signals_sent": signals,
            "remaining_live_pids": remaining,
            "leader_reaped": process.returncode is not None,
            "complete": not remaining and process.returncode is not None}


def run_case(command, directory, interval, duration=None):
    directory.mkdir()
    stdout_path, stderr_path = directory / "stdout.log", directory / "stderr.log"
    samples, interrupted_at, measuring = [], None, False
    begin = time.monotonic()
    with stdout_path.open("w") as stdout, stderr_path.open("w") as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr,
                                   start_new_session=True)
        try:
            while process.poll() is None:
                elapsed = time.monotonic() - begin
                if not measuring:
                    measuring = "VISION_MEMORY_MEASURE_BEGIN" in stderr_path.read_text()
                sample = memory_sample(process.pid, elapsed, measuring)
                samples.append(sample)
                if duration is not None and elapsed >= duration:
                    if interrupted_at is None:
                        send_group(process, signal.SIGINT)
                        interrupted_at = elapsed
                    elif elapsed - interrupted_at > 4:
                        # Hand off to bounded TERM/KILL cleanup. SIGKILL does
                        # not make an uninterruptible kernel wait return.
                        break
                    elif elapsed - interrupted_at > 2:
                        send_group(process, signal.SIGTERM)
                time.sleep(interval)
        except BaseException:
            cleanup_process_group(process, terminate_grace_s=0)
            raise
        # A wrapper may exit before its capture child: do not leave our process
        # group running silently, and report any tasks cleanup cannot stop.
        cleanup = cleanup_process_group(process)
        returncode = process.poll()
    usable = [sample for sample in samples if sample["tree_pss_kib"] is not None]
    steady = [sample["tree_pss_kib"] for sample in usable
              if sample["measurement_phase"]]
    probe = None
    lines = stdout_path.read_text().splitlines()
    if returncode == 0 and lines:
        try:
            probe = json.loads(lines[-1])
        except json.JSONDecodeError:
            pass  # Generic profile commands need not emit JSON.
    result = {
        "command": command, "returncode": returncode,
        "cleanup": cleanup,
        "duration_limit_reached": interrupted_at is not None,
        "application_success": returncode == 0 and interrupted_at is None,
        "collection_complete": bool(usable) and cleanup["complete"] and
                               not any(s["processes_unreadable"] for s in samples),
        "elapsed_s": time.monotonic() - begin,
        "sample_interval_s": interval, "probe": probe,
        "sampled_tree_pss_peak_kib": max((s["tree_pss_kib"] for s in usable), default=None),
        "sampled_tree_rss_peak_kib": max((s["tree_rss_kib"] for s in usable), default=None),
        "sampled_tree_swap_pss_peak_kib": max((s["tree_swap_pss_kib"] for s in usable), default=None),
        "measurement_pss_p95_kib": percentile(steady, .95),
        "measurement_samples": len(steady),
        "max_processes_read": max((s["processes_read"] for s in samples), default=0),
        "unreadable_process_samples": sum(s["processes_unreadable"] for s in samples),
        "host_mem_available_min_kib": min((s["host"]["MemAvailable"] for s in samples), default=None),
        "samples": samples,
        "stdout": str(stdout_path), "stderr": str(stderr_path),
    }
    print(f"{directory.name}: exit={returncode}, PSS峰值={result['sampled_tree_pss_peak_kib']} KiB",
          file=sys.stderr, flush=True)
    return result


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    compare = sub.add_parser("compare", help="Repeat the traditional vision memory workload")
    compare.add_argument("--probe", default="build/vision_memory_probe")
    compare.add_argument("--image", required=True)
    compare.add_argument("--threads", type=int, default=2)
    compare.add_argument("--frames", type=int, default=300)
    compare.add_argument("--warmup", type=int, default=30)
    compare.add_argument("--repeats", type=int, default=3)
    compare.add_argument("--light-config", help="Defaults to armor config detector, or config/light_bars.yaml")
    compare.add_argument("--armor-config")
    profile = sub.add_parser("profile", help="Sample a command and all its child processes")
    profile.add_argument("--duration", type=positive, default=60)
    profile.add_argument("command", nargs=argparse.REMAINDER,
                         help="-- COMMAND [ARG ...]; no shell expansion")
    for target in (compare, profile):
        target.add_argument("--output", required=True, help="New JSON file; will not overwrite")
        target.add_argument("--interval", type=positive, default=.05)
    args = parser.parse_args()
    if sys.platform != "linux":
        parser.error("This profiler requires Linux /proc")
    if args.interval < .01 or args.interval > 5:
        parser.error("--interval must be in .01..5 seconds")
    if args.mode == "compare" and not (1 <= args.repeats <= 30):
        parser.error("--repeats must be in 1..30")
    output = Path(args.output).resolve()
    if output.exists():
        parser.error("--output must be new")
    cases = output.with_suffix(".cases")
    cases.mkdir(parents=True, exist_ok=False)
    result = {"schema_version": 1,
              "started_at": dt.datetime.now().astimezone().isoformat(),
              "mode": args.mode, "architecture": platform.machine(),
              "platform": platform.platform(), "python": platform.python_version(),
              "host_before": meminfo(), "cwd": str(Path.cwd()),
              "memory_units": "KiB (1024 bytes)",
              "notes": ["PSS/RSS values are sampled simultaneous process-tree totals, not kernel high-water marks.",
                        "Summed RSS double-counts shared pages. PSS apportions them among sharers.",
                        "Short peaks or short-lived children can be missed; inspect measurement_samples and unreadable counts.",
                        "probe.peak_rss_kib is the kernel lifetime high-water mark of the algorithm process only.",
                        "PC CPU results do not measure ARM64, RKNN/NPU device buffers or the complete board workload."],
              "cases": []}
    failed = False
    if args.mode == "compare":
        probe, image = map(lambda p: str(Path(p).resolve()),
                           (args.probe, args.image))
        result["image_sha256"] = digest(image)
        result["probe_sha256"] = digest(probe)
        result["light_config_sha256"] = digest(args.light_config or "config/light_bars.yaml") if args.light_config or not args.armor_config else None
        result["armor_config_sha256"] = digest(args.armor_config) if args.armor_config else None
        result["configuration"] = vars(args)
        for repeat in range(args.repeats):
            command = [probe, "--mode", "traditional", "--image", image,
                       "--frames", str(args.frames), "--warmup", str(args.warmup),
                       "--threads", str(args.threads)]
            if args.light_config or not args.armor_config:
                command += ["--light-config", str(Path(args.light_config or "config/light_bars.yaml").resolve())]
            if args.armor_config:
                command += ["--armor-config", str(Path(args.armor_config).resolve())]
            case = run_case(command, cases / f"{repeat + 1}-traditional", args.interval)
            case.update({"algorithm": "traditional", "repeat": repeat + 1})
            result["cases"].append(case)
            if case["returncode"] or not case["probe"] or not case["collection_complete"]:
                failed = True
                break
        result["summary"] = {}
        reports = [c["probe"] for c in result["cases"]
                   if c["returncode"] == 0 and c["probe"]]
        if reports:
            result["summary"]["traditional"] = {
                "successful_runs": len(reports),
                "peak_rss_kib_max": max(r["peak_rss_kib"] for r in reports),
                "peak_rss_kib_median": statistics.median(r["peak_rss_kib"] for r in reports),
                "algorithm_mean_ms_median": statistics.median(r["algorithm_mean_ms"] for r in reports),
                "fps_median": statistics.median(r["fps"] for r in reports)}
    else:
        command = args.command[1:] if args.command[:1] == ["--"] else args.command
        if not command:
            parser.error("profile requires -- COMMAND [ARG ...]")
        case = run_case(command, cases / "profile", args.interval, args.duration)
        result["cases"].append(case)
        failed = ((case["returncode"] != 0 and not case["duration_limit_reached"])
                  or not case["collection_complete"])
        result["success_means"] = "memory collection completed; application_success is separate"
    result["success"] = not failed
    with output.open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(output), "success": not failed,
                      "summary": result.get("summary")}, ensure_ascii=False))
    return int(failed)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"benchmark_vision_memory: {error}", file=sys.stderr)
        raise SystemExit(1)
