#!/usr/bin/env python3
"""No-hardware regressions for Linux CPU/PSS process-tree accounting."""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("resource_sampling", ROOT / "tools/resource_sampling.py")
sampling = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sampling)


def stat_text(pid, *, ticks=0, child_ticks=0, start=123, state="S", ppid=1):
    fields = ["0"] * 50
    fields[0], fields[1], fields[2] = state, str(ppid), str(pid)
    fields[11], fields[13], fields[19] = str(ticks), str(child_ticks), str(start)
    return f"{pid} (worker ) with ( spaces)) {' '.join(fields)}\n"


def fake_process(root, pid, *, children="", children_interface=True, **stat):
    path = root / str(pid)
    (path / "task" / str(pid)).mkdir(parents=True)
    (path / "stat").write_text(stat_text(pid, **stat))
    if children_interface:
        (path / "task" / str(pid) / "children").write_text(children)
        own_task = root / "self" / "task" / str(os.getpid())
        own_task.mkdir(parents=True, exist_ok=True)
        (own_task / "children").write_text("")
    (path / "smaps_rollup").write_text("Rss: 40 kB\nPss: 20 kB\nSwapPss: 0 kB\n")


def sample(t, cpu, *, identity=None, **updates):
    value = {"monotonic_s": t, "cpu_seconds": cpu, "rss_kib": 40,
             "pss_kib": 20, "swap_pss_kib": 0, "readable": True,
             "complete": True, "identity": identity or [123, 456], "issues": []}
    value.update(updates)
    return value


@unittest.skipUnless(sys.platform == "linux", "requires Linux /proc")
class ResourceSamplingTest(unittest.TestCase):
    def test_comm_with_spaces_and_parentheses(self):
        stat = sampling._parse_stat(stat_text(17, ticks=3, child_ticks=9, start=456))
        self.assertEqual((stat["pid"], stat["starttime"], stat["cpu_ticks"]), (17, 456, 12))

    def test_reaping_transfers_cpu_without_double_counting(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_process(root, 10, children="11", ticks=5)
            fake_process(root, 11, ticks=25)
            with mock.patch.object(sampling, "_PROC", root):
                running = sampling.snapshot(10)
                self.assertTrue(running["complete"])
                (root / "10/task/10/children").write_text("")
                (root / "10/stat").write_text(stat_text(10, ticks=5, child_ticks=25))
                reaped = sampling.snapshot(10)
            self.assertTrue(reaped["complete"])
            self.assertEqual(running["cpu_seconds"], reaped["cpu_seconds"])
            self.assertEqual(reaped["process_count"], 1)

    def test_ppid_fallback_finds_only_descendants_and_reaping_preserves_cpu(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_process(root, 10, ticks=5, children_interface=False)
            fake_process(root, 11, ppid=10, ticks=15, children_interface=False)
            fake_process(root, 12, ppid=11, ticks=10, children_interface=False)
            fake_process(root, 20, ticks=1000, children_interface=False)
            with mock.patch.object(sampling, "_PROC", root):
                running = sampling.snapshot(10)
                self.assertTrue(running["complete"], running)
                self.assertEqual(running["process_count"], 3)
                self.assertEqual(running["pss_kib"], 60)
                self.assertEqual(running["cpu_seconds"], 30 / sampling._CLOCK_TICKS)
                # Child 11 waited for 12, then root 10 waited for 11. The
                # descendants' total CPU now belongs to root's cutime/cstime.
                shutil.rmtree(root / "11")
                shutil.rmtree(root / "12")
                (root / "10/stat").write_text(stat_text(10, ticks=5, child_ticks=25))
                reaped = sampling.snapshot(10)
                self.assertTrue(reaped["complete"], reaped)
                self.assertEqual(reaped["process_count"], 1)
                self.assertEqual(running["cpu_seconds"], reaped["cpu_seconds"])
                shutil.rmtree(root / "10")
                missing = sampling.snapshot(10)
            self.assertFalse(missing["complete"])
            self.assertFalse(missing["readable"])
            self.assertIsNone(missing["cpu_seconds"])
            self.assertIn("root_missing", missing["issues"])

    def test_ppid_fallback_ignores_exited_unclassified_pid_but_rejects_lost_member(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_process(root, 10, children_interface=False)
            fake_process(root, 11, ppid=10, children_interface=False)
            fake_process(root, 20, children_interface=False)
            (root / "20/stat").unlink()
            with mock.patch.object(sampling, "_PROC", root):
                stable = sampling.snapshot(10)
                self.assertTrue(stable["complete"], stable)
                self.assertEqual(stable["process_count"], 2)
                read_memory = sampling._read_memory

                def read_then_exit(pid):
                    memory = read_memory(pid)
                    if pid == 11:
                        (root / "11/stat").unlink()
                    return memory

                with mock.patch.object(sampling, "_read_memory", side_effect=read_then_exit):
                    value = sampling.snapshot(10)
                (root / "10/stat").unlink()
                missing_root = sampling.snapshot(10)
            self.assertFalse(value["complete"])
            self.assertIn("tree_membership_changed_during_sample", value["issues"])
            self.assertFalse(missing_root["complete"])
            self.assertIn("root_missing", missing_root["issues"])
            self.assertIn("process_unreadable:10:FileNotFoundError", missing_root["issues"])

    def test_lifecycle_race_and_missing_root_are_incomplete(self):
        original = {10: sampling._parse_stat(stat_text(10))}
        replacement = {10: sampling._parse_stat(stat_text(10, start=999))}
        with mock.patch.object(sampling, "_read_tree", side_effect=[(original, []), (replacement, [])]), \
                mock.patch.object(sampling, "_read_memory", return_value={"Rss": 1, "Pss": 1, "SwapPss": 0}):
            raced = sampling.snapshot(10)
        self.assertFalse(raced["complete"])
        self.assertIn("tree_membership_changed_during_sample", raced["issues"])
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(sampling, "_PROC", Path(directory)):
            missing = sampling.snapshot(10)
        self.assertFalse(missing["readable"])
        self.assertFalse(missing["complete"])
        self.assertIsNone(missing["cpu_seconds"])

    def test_child_of_nonleader_thread_and_partial_memory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_process(root, 10)
            fake_process(root, 11)
            thread = root / "10/task/12"
            thread.mkdir()
            (thread / "children").write_text("11")
            with mock.patch.object(sampling, "_PROC", root):
                self.assertEqual(sampling.snapshot(10)["process_count"], 2)
                (root / "11/smaps_rollup").unlink()
                broken = sampling.snapshot(10)
            self.assertFalse(broken["readable"])
            self.assertFalse(broken["complete"])
            self.assertEqual(broken["pss_kib"], 20)

    def test_zombie_retains_cpu_but_has_no_memory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_process(root, 10, children="11", ticks=5)
            fake_process(root, 11, ticks=25, state="Z")
            (root / "11/smaps_rollup").unlink()
            with mock.patch.object(sampling, "_PROC", root):
                value = sampling.snapshot(10)
            self.assertTrue(value["complete"])
            self.assertEqual(value["cpu_seconds"], 30 / sampling._CLOCK_TICKS)
            self.assertEqual(value["pss_kib"], 20)

    def test_summary_cpu_uses_full_window_and_host_normalization(self):
        result = sampling.summarize([sample(0, 0), sample(1, 1), sample(4, 2)], 4)
        self.assertTrue(result["complete"])
        self.assertEqual(result["cpu_percent_one_core_mean"], 50)
        self.assertEqual(result["cpu_percent_host_mean"], 12.5)
        self.assertEqual(result["sample_duration_s"], 4)
        self.assertEqual(result["sampled_pss_peak_kib"], 20)

    def test_counter_reset_identity_change_and_missing_data_invalidate_cpu(self):
        for rows, reason in (
            ([sample(0, 2), sample(1, 1)], "cpu_counter_decreased"),
            ([sample(0, 1), sample(1, 2, identity=[123, 457])], "identity_missing_or_changed"),
            ([sample(0, 1), sample(1, None, complete=False)], "incomplete_sample"),
            ([sample(1, 1), sample(1, 2)], "non_increasing_sample_time"),
        ):
            with self.subTest(reason=reason):
                result = sampling.summarize(rows, 2)
                self.assertFalse(result["complete"])
                self.assertFalse(result["cpu_valid"])
                self.assertIsNone(result["cpu_percent_one_core_mean"])
                self.assertIn(reason, result["issues"])
        self.assertFalse(sampling.summarize([], 1)["complete"])
        with self.assertRaises(ValueError):
            sampling.summarize([], 0)

    def test_nested_samples_and_incomplete_memory(self):
        rows = [{"monotonic_s": 0, "snapshot": sample(999, 0)},
                {"monotonic_s": 1, "snapshot": sample(999, 1, pss_kib=None)}]
        result = sampling.summarize(rows, 1)
        self.assertTrue(result["cpu_valid"])
        self.assertFalse(result["memory_complete"])
        self.assertFalse(result["complete"])
        self.assertEqual(result["cpu_percent_one_core_mean"], 100)

    def test_real_busy_sleep_and_reaped_child(self):
        # The fixture performs a fixed amount of *CPU time*, not a timing-sensitive
        # FPS assertion. No serial ports, networks, camera or flight code is used.
        code = """
import subprocess, sys, time
print('ready', flush=True)
for line in sys.stdin:
    if line.strip() == 'busy':
        begin = time.process_time()
        while time.process_time() - begin < .10:
            pass
    elif line.strip() == 'child':
        subprocess.run([sys.executable, '-c',
            'import time; t=time.process_time(); exec("while time.process_time()-t<.10: pass")'], check=True)
    elif line.strip() == 'sleep':
        time.sleep(.15)
    else:
        break
    print('done', flush=True)
"""
        worker = subprocess.Popen([sys.executable, "-u", "-c", code], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(self._stop_worker, worker)
        self.assertEqual(worker.stdout.readline().strip(), "ready")
        deltas = {}
        for mode in ("sleep", "busy", "child"):
            before = sampling.snapshot(worker.pid)
            before["monotonic_s"] = time.monotonic()
            worker.stdin.write(mode + "\n")
            worker.stdin.flush()
            self.assertEqual(worker.stdout.readline().strip(), "done")
            after = sampling.snapshot(worker.pid)
            after["monotonic_s"] = time.monotonic()
            result = sampling.summarize([before, after], max(1, len(os.sched_getaffinity(0))))
            self.assertTrue(result["complete"], result)
            deltas[mode] = result["cpu_seconds_delta"]
            if mode == "child":
                stat = sampling._parse_stat(Path(f"/proc/{worker.pid}/stat").read_text())
                self.assertEqual(after["process_count"], 1)
                self.assertEqual(after["cpu_seconds"], stat["cpu_ticks"] / sampling._CLOCK_TICKS)
        self.assertGreater(deltas["busy"], .05)
        self.assertGreater(deltas["child"], .05)
        self.assertGreater(deltas["busy"], deltas["sleep"])

    @staticmethod
    def _stop_worker(worker):
        if worker.poll() is None:
            worker.stdin.close()
            try:
                worker.wait(timeout=5)
            except subprocess.TimeoutExpired:
                worker.kill()
                worker.wait(timeout=5)
        worker.stdout.close()
        worker.stderr.close()


if __name__ == "__main__":
    unittest.main()
