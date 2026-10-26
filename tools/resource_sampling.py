"""Linux process-tree resource samples, without third-party dependencies.

The root must stay alive and reap its children during a measurement. CPU time is
the sum of each current member's own and *already waited-for* child CPU time.
This transfers a reaped child's time to its parent without retaining and adding
the child's old counter a second time. Escaped/daemonized children and children
reparented outside this tree are outside this accounting contract. Membership is
checked twice; a lifecycle race makes the sample incomplete, never silently valid.
Kernels without /proc/PID/task/TID/children use one PPID scan per tree read.

PSS/RSS are sampled totals, not kernel high-water marks. Shared pages are counted
more than once in summed RSS, whereas PSS apportions them among their sharers.
"""

import math
import os
from pathlib import Path
import statistics
import time


_PROC = Path("/proc")
_CLOCK_TICKS = os.sysconf("SC_CLK_TCK")


def _parse_stat(text):
    """Parse /proc/PID/stat even when comm includes spaces or parentheses."""
    head, tail = text.rsplit(")", 1)
    pid_text, _comm = head.split(" (", 1)
    fields = tail.split()
    return {
        "pid": int(pid_text), "state": fields[0], "ppid": int(fields[1]),
        "pgrp": int(fields[2]), "starttime": int(fields[19]),
        "cpu_ticks": sum(int(fields[index]) for index in (11, 12, 13, 14)),
    }


def _read_tree(root):
    # Check our own interface, not the sampled PID: a target exiting must not
    # masquerade as a kernel without the children feature.
    if not (_PROC / "self" / "task" / str(os.getpid()) / "children").exists():
        return _read_tree_from_ppids(root)
    pending, found, issues = [root], {}, []
    while pending:
        pid = pending.pop()
        if pid in found:
            continue
        try:
            stat = _parse_stat((_PROC / str(pid) / "stat").read_text())
            if stat["pid"] != pid or stat["cpu_ticks"] < 0:
                raise ValueError("invalid process counter")
            found[pid] = stat
            task = _PROC / str(pid) / "task"
            # A child may belong to any thread, not only the group leader.
            threads = list(task.iterdir())
            if not threads:
                raise ValueError("missing process threads")
            for thread in threads:
                pending.extend(map(int, (thread / "children").read_text().split()))
        except (OSError, ValueError, IndexError) as error:
            issues.append(f"process_unreadable:{pid}:{type(error).__name__}")
    return found, issues


def _read_tree_from_ppids(root):
    """Scan stat once, then traverse PPIDs without rescanning /proc per node."""
    processes, children, issues = {}, {}, []
    try:
        entries = list(_PROC.iterdir())
    except OSError as error:
        return {}, [f"proc_scan_unreadable:{type(error).__name__}"]
    for entry in entries:
        if not entry.name.isdecimal():
            continue
        pid = int(entry.name)
        try:
            stat = _parse_stat((entry / "stat").read_text())
            if stat["pid"] != pid or stat["cpu_ticks"] < 0:
                raise ValueError("invalid process counter")
            processes[pid] = stat
            children.setdefault(stat["ppid"], []).append(pid)
        except FileNotFoundError as error:
            # An unclassified PID may exit after listing /proc. Do not let an
            # unrelated short-lived process invalidate every sampled tree.
            # A missing root is still an error; a previously observed member
            # vanishing is detected by snapshot's two membership reads.
            if pid == root:
                issues.append(f"process_unreadable:{pid}:{type(error).__name__}")
        except (OSError, ValueError, IndexError) as error:
            issues.append(f"process_unreadable:{pid}:{type(error).__name__}")
    pending, found = [root], {}
    while pending:
        pid = pending.pop()
        if pid in found or pid not in processes:
            continue
        found[pid] = processes[pid]
        pending.extend(children.get(pid, ()))
    return found, issues


def _read_memory(pid):
    values = {}
    for line in (_PROC / str(pid) / "smaps_rollup").read_text().splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key in ("Rss", "Pss", "SwapPss"):
            values[key] = int(value.split()[0])
    if any(key not in values for key in ("Rss", "Pss", "SwapPss")):
        raise ValueError("missing smaps_rollup counters")
    if any(value < 0 for value in values.values()):
        raise ValueError("negative smaps_rollup counter")
    return values


def snapshot(pid):
    """Read a tree rooted at PID; inspect ``complete`` before using its totals.

    ``cpu_seconds`` includes descendants that have exited and were reaped by a
    still-living ancestor. It is a cumulative counter, not a CPU percentage.
    Failed reads retain any partial totals for diagnosis but mark them invalid.
    """
    started = time.monotonic()
    before, issues = _read_tree(pid)
    total = {key: 0 for key in ("Rss", "Pss", "SwapPss")}
    read = 0
    for member, stat in before.items():
        # Zombies own no address space, but retain CPU time until waited for.
        if stat["state"] in ("Z", "X"):
            read += 1
            continue
        try:
            memory = _read_memory(member)
            for key in total:
                total[key] += memory[key]
            read += 1
        except (OSError, ValueError, IndexError) as error:
            issues.append(f"memory_unreadable:{member}:{type(error).__name__}")
    after, later_issues = _read_tree(pid)
    issues.extend(later_issues)
    identities_before = {(member, stat["starttime"])
                         for member, stat in before.items()}
    identities_after = {(member, stat["starttime"])
                        for member, stat in after.items()}
    if identities_before != identities_after:
        issues.append("tree_membership_changed_during_sample")
    root = after.get(pid)
    root_present = root is not None and pid in before
    if not root_present:
        issues.append("root_missing")
    complete = root_present and read == len(before) and not issues
    return {
        "identity": [pid, root["starttime"]] if root is not None else None,
        "cpu_seconds": (sum(stat["cpu_ticks"] for stat in after.values()) /
                        _CLOCK_TICKS) if after else None,
        "rss_kib": total["Rss"] if read else None,
        "pss_kib": total["Pss"] if read else None,
        "swap_pss_kib": total["SwapPss"] if read else None,
        "readable": root_present and read == len(before),
        "complete": bool(complete), "process_count": len(after),
        "processes_read": read,
        "issues": sorted(set(issues)),
        "sample_read_duration_s": time.monotonic() - started,
        "cpu_scope": "current_tree_self_and_reaped_children",
    }


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * fraction) - 1)]


def summarize(samples, cpu_count):
    """Summarize flat samples with monotonic_s, or nested ``snapshot`` samples.

    CPU 100% means one occupied logical CPU. The host-normalized figure divides
    by the caller's effective logical CPU count. P95 is over interval samples;
    memory means are arithmetic sample means. Any invalid CPU interval makes
    the CPU aggregate null rather than hiding a reset with ``max(delta, 0)``.
    """
    if not isinstance(cpu_count, int) or isinstance(cpu_count, bool) or cpu_count < 1:
        raise ValueError("cpu_count must be a positive integer")
    rows = []
    for sample in samples:
        if isinstance(sample.get("snapshot"), dict):
            rows.append({**sample["snapshot"], "monotonic_s": sample.get("monotonic_s")})
        else:
            rows.append(sample)
    issues = set()
    cpu_valid = len(rows) >= 2
    if not cpu_valid:
        issues.add("insufficient_samples")
    identity = rows[0].get("identity") if rows else None
    for row in rows:
        if not row.get("readable", False) or not row.get("complete", False):
            issues.add("incomplete_sample")
            cpu_valid = False
        if identity is None or row.get("identity") != identity:
            issues.add("identity_missing_or_changed")
            cpu_valid = False
        if not _finite(row.get("cpu_seconds")) or row.get("cpu_seconds", -1) < 0:
            issues.add("invalid_cpu_counter")
            cpu_valid = False
        if not _finite(row.get("monotonic_s")):
            issues.add("invalid_sample_time")
            cpu_valid = False
        issues.update(str(issue) for issue in row.get("issues", []))
    interval_percent = []
    for previous, current in zip(rows, rows[1:]):
        if not all(_finite(row.get(key)) for row in (previous, current)
                   for key in ("monotonic_s", "cpu_seconds")):
            continue
        wall = current["monotonic_s"] - previous["monotonic_s"]
        cpu = current["cpu_seconds"] - previous["cpu_seconds"]
        if wall <= 0:
            issues.add("non_increasing_sample_time")
            cpu_valid = False
        elif cpu < 0:
            issues.add("cpu_counter_decreased")
            cpu_valid = False
        else:
            interval_percent.append(100 * cpu / wall)
    duration = None
    if len(rows) >= 2 and all(_finite(row.get("monotonic_s"))
                              for row in (rows[0], rows[-1])):
        candidate = rows[-1]["monotonic_s"] - rows[0]["monotonic_s"]
        if candidate > 0:
            duration = candidate
    cpu_delta = ((rows[-1]["cpu_seconds"] - rows[0]["cpu_seconds"])
                 if cpu_valid and duration is not None else None)
    cpu_mean = 100 * cpu_delta / duration if cpu_delta is not None else None
    result = {
        "sample_count": len(rows), "sample_duration_s": duration,
        "cpu_count": cpu_count, "cpu_valid": bool(cpu_valid),
        "cpu_seconds_delta": cpu_delta,
        "cpu_percent_one_core_mean": cpu_mean,
        "cpu_percent_host_mean": cpu_mean / cpu_count if cpu_mean is not None else None,
        "cpu_percent_one_core_p95": _percentile(interval_percent, .95) if cpu_valid else None,
        "cpu_percent_host_p95": (_percentile(interval_percent, .95) / cpu_count
                                 if cpu_valid else None),
        "memory_complete": bool(rows),
    }
    for key in ("rss", "pss", "swap_pss"):
        values = [row[f"{key}_kib"] for row in rows
                  if row.get("readable") and row.get("complete")
                  and _finite(row.get(f"{key}_kib")) and row[f"{key}_kib"] >= 0]
        result[f"sampled_{key}_peak_kib"] = max(values, default=None)
        result[f"{key}_mean_kib"] = statistics.fmean(values) if values else None
        if len(values) != len(rows):
            result["memory_complete"] = False
            issues.add("incomplete_memory_samples")
    result["complete"] = bool(cpu_valid and result["memory_complete"])
    result["issues"] = sorted(issues)
    return result
