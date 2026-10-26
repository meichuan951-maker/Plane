#!/usr/bin/env python3
"""Offline video + gogogo-derived control workload benchmark; never opens a flight link."""
import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import platform
import selectors
import statistics
import subprocess
import sys
import time

from benchmark_vision_memory import cleanup_process_group, digest, meminfo
from resource_sampling import snapshot, summarize

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ("control_only", "traditional_only", "traditional_joint")
REPORT_TIMEZONE = dt.timezone(dt.timedelta(hours=8), "Asia/Shanghai")


def positive(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def strict_json(text):
    def reject(value):
        raise ValueError("non-finite JSON constant: " + value)
    return json.loads(text, parse_constant=reject)


class Worker:
    """Own only this invocation's process group and a bounded JSON protocol."""
    def __init__(self, role, command, directory):
        self.role, self.command = role, command
        self.buffer = b""
        self.events = {}
        self.stderr = (directory / f"{role}.stderr.log").open("wb")
        self.stdout = (directory / f"{role}.stdout.jsonl").open("wb")
        try:
            self.process = subprocess.Popen(
                command, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=self.stderr, start_new_session=True, bufsize=0)
        except BaseException:
            self.stderr.close()
            self.stdout.close()
            raise
        os.set_blocking(self.process.stdout.fileno(), False)

    def send(self, data):
        self.process.stdin.write((json.dumps(data, allow_nan=False) + "\n").encode())
        self.process.stdin.flush()

    def drain(self):
        while True:
            try:
                chunk = os.read(self.process.stdout.fileno(), 65536)
            except BlockingIOError:
                break
            if not chunk:
                break
            self.stdout.write(chunk)
            self.stdout.flush()
            self.buffer += chunk
            if len(self.buffer) > 1024 * 1024:
                raise RuntimeError(f"{self.role}: oversized protocol message")
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                event = strict_json(line)
                kind = event.get("event")
                if event.get("role") != self.role or kind not in ("ready", "result"):
                    raise RuntimeError(f"{self.role}: unexpected worker protocol")
                if kind in self.events:
                    raise RuntimeError(f"{self.role}: duplicate {kind} event")
                self.events[kind] = event

    def close(self):
        error = None
        try:
            if self.process.poll() is None:
                try:
                    self.send({"event": "release"})
                    self.process.stdin.close()
                    self.process.wait(timeout=3)
                except (OSError, subprocess.TimeoutExpired) as exc:
                    error = str(exc)
            self.drain()
        finally:
            cleanup = cleanup_process_group(self.process)
            self.process.stdin.close()
            self.process.stdout.close()
            self.stderr.close()
            self.stdout.close()
        return {"returncode": self.process.returncode, "cleanup": cleanup,
                "release_error": error}


def aggregate(values):
    readable = all(v["readable"] for v in values)
    return {"identity": [v["identity"] for v in values],
            "cpu_seconds": sum(v["cpu_seconds"] for v in values)
            if all(v["cpu_seconds"] is not None for v in values) else None,
            **{key: sum(v[key] for v in values) if readable else None
               for key in ("rss_kib", "pss_kib", "swap_pss_kib")},
            "readable": readable, "complete": all(v["complete"] for v in values),
            "process_count": sum(v["process_count"] for v in values),
            "issues": [f"worker {i}: {issue}" for i, v in enumerate(values)
                       for issue in v["issues"]]}


def commands_for(args, scenario):
    commands = {}
    if scenario != "control_only":
        command = [str(args.worker), "--benchmark-worker", "--mode", "traditional",
                   "--video", str(args.video), "--width", str(args.width),
                   "--height", str(args.height), "--fps", str(args.fps),
                   "--threads", str(args.threads)]
        command += ["--armor-config", str(args.armor_config)]
        commands["vision"] = command
    if scenario == "control_only" or scenario.endswith("_joint"):
        commands["control"] = [sys.executable, "-m", "px4_go.offline_control",
                               "--benchmark-worker"]
    return commands


def validate_workloads(workers, duration):
    issues = []
    for role, worker in workers.items():
        report = worker.events.get("result", {})
        if report.get("success") is not True:
            issues.append(f"{role}: missing or unsuccessful result")
        count_key = "measured_frames" if role == "vision" else "measured_setpoints"
        if report.get(count_key, 0) < 2:
            issues.append(f"{role}: no sustained measured work")
        if role == "control" and report.get("measured_setpoints", 0) < duration * 10:
            issues.append("control: sustained setpoint rate below 10 Hz")
    return issues


def run_case(args, scenario, repeat, directory):
    directory.mkdir()
    workers, records, errors, shutdown = {}, [], [], {}
    measurement_start = measurement_end = None
    interrupted = False
    with selectors.DefaultSelector() as selector:
        def pump(timeout):
            for key, _ in selector.select(timeout):
                key.data.drain()
            for role, worker in workers.items():
                if worker.process.poll() is not None:
                    worker.drain()
                    raise RuntimeError(f"{role} exited before release: {worker.process.returncode}")

        def sample(phase):
            begin = time.monotonic()
            values = {role: snapshot(worker.process.pid) for role, worker in workers.items()}
            stamp = (begin + time.monotonic()) / 2
            records.append({"monotonic_s": stamp, "phase": phase, "workers": values,
                            "total": aggregate(list(values.values())), "host": meminfo()})

        try:
            for role, command in commands_for(args, scenario).items():
                workers[role] = Worker(role, command, directory)
                selector.register(workers[role].process.stdout, selectors.EVENT_READ, workers[role])
            deadline = time.monotonic() + args.startup_timeout
            while not all("ready" in worker.events for worker in workers.values()):
                if time.monotonic() > deadline:
                    raise RuntimeError("worker readiness timeout")
                sample("startup")
                pump(args.interval)
            start = time.monotonic() + .25
            measurement_start, measurement_end = start + args.warmup, start + args.warmup + args.duration
            for worker in workers.values():
                worker.send({"start_monotonic_s": start, "warmup_s": args.warmup,
                             "measure_s": args.duration})
            while time.monotonic() < measurement_start:
                sample("warmup")
                pump(min(args.interval, max(0, measurement_start - time.monotonic())))
            sample("measurement")
            next_sample = time.monotonic() + args.interval
            while time.monotonic() < measurement_end:
                pump(max(0, min(next_sample, measurement_end) - time.monotonic()))
                if time.monotonic() >= min(next_sample, measurement_end):
                    sample("measurement")
                    next_sample = time.monotonic() + args.interval
            # Workers hold their processes after the result so the final CPU
            # counter and resident pages remain available at this boundary.
            deadline = time.monotonic() + args.finish_timeout
            while not all("result" in worker.events for worker in workers.values()):
                if time.monotonic() > deadline:
                    raise RuntimeError("worker completion timeout")
                pump(min(args.interval, max(0, deadline - time.monotonic())))
            errors.extend(validate_workloads(workers, args.duration))
        except KeyboardInterrupt:
            interrupted = True
            errors.append("benchmark interrupted")
        except (OSError, ValueError, RuntimeError) as exc:
            errors.append(str(exc))
        finally:
            for role, worker in workers.items():
                try:
                    shutdown[role] = worker.close()
                except (OSError, ValueError, RuntimeError) as exc:
                    errors.append(f"{role} cleanup: {exc}")
            for role, state in shutdown.items():
                if state["returncode"] != 0 or not state["cleanup"]["complete"] or state["release_error"]:
                    errors.append(f"{role}: unsuccessful shutdown")
    measured = [s for s in records if s["phase"] == "measurement"]
    metrics = {}
    for role in [*workers, "total"]:
        samples = [{"monotonic_s": s["monotonic_s"],
                    **(s["total"] if role == "total" else s["workers"][role])}
                   for s in measured]
        metrics[role] = summarize(samples, args.cpu_count)
        if not metrics[role]["complete"]:
            errors.append(f"{role}: incomplete measurement")
    if not measured or measured[-1]["monotonic_s"] - measured[0]["monotonic_s"] < args.duration * .95:
        errors.append("measurement window incomplete")
    all_peaks = {}
    for role in [*workers, "total"]:
        values = [s["total"] if role == "total" else s["workers"][role] for s in records]
        all_peaks[role] = {"sampled_pss_peak_kib": max((v["pss_kib"] for v in values if v["pss_kib"] is not None), default=None),
                           "sampled_rss_peak_kib": max((v["rss_kib"] for v in values if v["rss_kib"] is not None), default=None)}
    raw = directory / "samples.json"
    raw.write_text(json.dumps(records, allow_nan=False, separators=(",", ":")) + "\n")
    result = {"scenario": scenario, "repeat": repeat, "workload_valid": not errors,
              "errors": errors, "interrupted": interrupted,
              "measurement_start_monotonic_s": measurement_start,
              "measurement_end_monotonic_s": measurement_end,
              "metrics": metrics, "whole_run_sampled_peaks": all_peaks,
              "workers": {role: {"command": worker.command, **worker.events}
                          for role, worker in workers.items()},
              "shutdown": shutdown, "samples": str(raw)}
    (directory / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"{directory.name}: {'PASS' if not errors else 'FAIL'}", flush=True)
    return result


def summarize_cases(cases):
    result = {}
    for scenario in SCENARIOS:
        good = [c for c in cases if c["scenario"] == scenario and c["workload_valid"]]
        if not good:
            continue
        roles = list(good[0]["metrics"])
        row = {"valid_repeats": len(good), "metrics": {}, "workload": {}}
        for role in roles:
            row["metrics"][role] = {}
            for key in ("cpu_percent_one_core_mean", "cpu_percent_host_mean",
                        "sampled_pss_peak_kib", "sampled_rss_peak_kib", "pss_mean_kib"):
                values = [c["metrics"][role][key] for c in good]
                row["metrics"][role][key + "_median"] = statistics.median(values)
                row["metrics"][role][key + "_max"] = max(values)
        for role in good[0]["workers"]:
            keys = ("actual_fps", "algorithm_mean_ms", "measured_frames") if role == "vision" else ("actual_hz", "period_p95_ms", "period_max_ms", "measured_setpoints")
            row["workload"][role] = {}
            for key in keys:
                values = [c["workers"][role]["result"].get(key) for c in good]
                if all(isinstance(v, (int, float)) for v in values):
                    row["workload"][role][key + "_median"] = statistics.median(values)
        result[scenario] = row
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--worker", type=Path, default=ROOT / "build/vision_video_worker")
    parser.add_argument("--armor-config", type=Path, default=ROOT / "config/armor_auto.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=positive, default=30)
    parser.add_argument("--warmup", type=positive, default=6)
    parser.add_argument("--interval", type=positive, default=.1)
    parser.add_argument("--startup-timeout", type=positive, default=30)
    parser.add_argument("--finish-timeout", type=positive, default=15)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=positive, default=30)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    args = parser.parse_args()
    if sys.platform != "linux":
        parser.error("Linux /proc is required")
    if not (1 <= args.repeats <= 30 and .01 <= args.interval <= 1 and
            1 <= args.threads <= 128 and 16 <= args.width <= 4096 and 16 <= args.height <= 4096
            and 1 <= args.fps <= 240 and .1 <= args.duration <= 3600 and args.warmup <= 600):
        parser.error("invalid repeats, interval, durations, frame size or threads")
    if len(set(args.scenarios)) != len(args.scenarios):
        parser.error("duplicate scenarios")
    input_keys = ["video", "worker", "armor_config"]
    for key in input_keys:
        path = getattr(args, key).resolve()
        if not path.is_file():
            parser.error(f"missing {key}: {path}")
        setattr(args, key, path)
    input_files = [getattr(args, key) for key in input_keys]
    args.output = args.output.resolve()
    if args.output.exists() or args.output.with_suffix(".csv").exists():
        parser.error("--output JSON and its CSV companion must both be new files")
    cases_dir = args.output.with_suffix(".cases")
    cases_dir.mkdir(parents=True, exist_ok=False)
    args.cpu_count = len(os.sched_getaffinity(0))
    cpu_model = next((line.split(":", 1)[1].strip() for line in
                      Path("/proc/cpuinfo").read_text().splitlines()
                      if line.startswith("model name")), platform.processor())
    report = {"schema_version": 1, "scope": "offline_video_plus_in_memory_control_peer",
              "started_at": dt.datetime.now(REPORT_TIMEZONE).isoformat(),
              "platform": platform.platform(), "architecture": platform.machine(),
              "cpu_model": cpu_model, "logical_cpus": os.cpu_count(),
              "effective_cpus": args.cpu_count, "cpu_affinity": sorted(os.sched_getaffinity(0)),
              "host_before": meminfo(), "configuration": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              "files": {str(p): digest(p) for p in input_files + [ROOT / "px4_go/gogogo.py",
                         ROOT / "px4_go/control.py", ROOT / "px4_go/offline_control.py",
                         ROOT / "tools/resource_sampling.py", Path(__file__).resolve()]},
              "config_directory_yaml_hashes": {str(p.resolve()): digest(p)
                  for p in sorted(args.armor_config.parent.glob("*.yaml")) if p.is_file()},
              "notes": ["CPU 100% means one logical core; host percent divides by effective CPU count.",
                        "Memory is KiB. PSS apportions shared pages; summed RSS counts them multiple times.",
                        "total is simultaneous vision+control only; profiler and other host programs are excluded.",
                        "Measurement CPU excludes initialization and warmup; startup peaks are separate.",
                        "Control includes Python, pymavlink and a protocol simulator, not PX4 firmware or EKF.",
                        "Video decode and resize are included; camera/GUI/ROS and GPU/NPU buffers are excluded.",
                        "PC CPU/memory results are not ARM64 or RKNN measurements."],
              "cases": [], "success": False}
    try:
        for repeat in range(1, args.repeats + 1):
            order = args.scenarios if repeat % 2 else list(reversed(args.scenarios))
            for scenario in order:
                case = run_case(args, scenario, repeat, cases_dir / f"{repeat:02d}-{scenario}")
                report["cases"].append(case)
                if not case["workload_valid"]:
                    return finish(report, args.output, False)
        return finish(report, args.output, True)
    except KeyboardInterrupt:
        report["error"] = "interrupted between cases"
        return finish(report, args.output, False)


def finish(report, output, success):
    report["success"] = success
    report["summary"] = summarize_cases(report["cases"])
    report["finished_at"] = dt.datetime.now(REPORT_TIMEZONE).isoformat()
    with output.open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    with output.with_suffix(".csv").open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scenario", "repeat", "role", "valid", "cpu_percent_one_core_mean",
                         "cpu_percent_host_mean", "pss_peak_mib", "rss_peak_mib"])
        for case in report["cases"]:
            for role, metric in case["metrics"].items():
                writer.writerow([case["scenario"], case["repeat"], role, case["workload_valid"],
                                 metric["cpu_percent_one_core_mean"], metric["cpu_percent_host_mean"],
                                 metric["sampled_pss_peak_kib"] / 1024 if metric["sampled_pss_peak_kib"] is not None else None,
                                 metric["sampled_rss_peak_kib"] / 1024 if metric["sampled_rss_peak_kib"] is not None else None])
    print(json.dumps({"success": success, "output": str(output),
                      "valid_cases": sum(c["workload_valid"] for c in report["cases"])}))
    return 0 if success else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"benchmark_vision_control: {exc}", file=sys.stderr)
        raise SystemExit(1)
