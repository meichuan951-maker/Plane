#!/usr/bin/env python3
"""Verify that synchronized profiling rejects empty or failed workloads."""
from pathlib import Path
from types import SimpleNamespace
import sys
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import benchmark_vision_control as benchmark


FAKE_WORKER = '''
import json,sys,time
role,mode=sys.argv[1:]
print(json.dumps({'event':'ready','role':role}),flush=True)
request=json.loads(sys.stdin.readline())
if mode=='exit': raise SystemExit(4)
end=request['start_monotonic_s']+request['warmup_s']+request['measure_s']
while time.monotonic()<end: time.sleep(.005)
print(json.dumps({'event':'result','role':role,'success':True,
                  'measured_frames':0 if mode=='empty' else 20,
                  'measured_setpoints':20,'actual_fps':30,'actual_hz':20}),flush=True)
sys.stdin.readline()
'''


class JointBenchmarkTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="joint-benchmark-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.worker = self.root / "fake worker.py"
        self.worker.write_text(FAKE_WORKER)
        self.args = SimpleNamespace(duration=.2, warmup=.05, interval=.04,
                                    startup_timeout=3, finish_timeout=3, cpu_count=1)

    def run_case(self, mode="ok"):
        commands = {role: [sys.executable, str(self.worker), role,
                           mode if role == "vision" else "ok"]
                    for role in ("vision", "control")}
        with patch.object(benchmark, "commands_for", return_value=commands):
            return benchmark.run_case(self.args, "traditional_joint", 1,
                                      self.root / "case")

    def test_same_window_totals_and_clean_release(self):
        case = self.run_case()
        self.assertTrue(case["workload_valid"], case["errors"])
        metric = case["metrics"]
        self.assertAlmostEqual(metric["total"]["cpu_seconds_delta"],
                               metric["vision"]["cpu_seconds_delta"] +
                               metric["control"]["cpu_seconds_delta"], places=7)
        self.assertGreaterEqual(metric["total"]["sample_duration_s"], .19)
        for shutdown in case["shutdown"].values():
            self.assertEqual(shutdown["returncode"], 0)
            self.assertTrue(shutdown["cleanup"]["complete"])

    def test_zero_processed_frames_is_not_a_successful_measurement(self):
        case = self.run_case("empty")
        self.assertFalse(case["workload_valid"])
        self.assertIn("vision: no sustained measured work", case["errors"])

    def test_early_exit_is_failure_and_other_worker_is_cleaned(self):
        case = self.run_case("exit")
        self.assertFalse(case["workload_valid"])
        self.assertTrue(any("exited before release" in error for error in case["errors"]))
        self.assertTrue(all(s["cleanup"]["complete"] for s in case["shutdown"].values()))

    def test_available_scenarios_generate_traditional_workloads(self):
        self.assertEqual(benchmark.SCENARIOS,
                         ("control_only", "traditional_only", "traditional_joint"))
        args = SimpleNamespace(worker=Path("worker"), video=Path("video.avi"),
                               armor_config=Path("armor.yaml"), width=640,
                               height=480, fps=30, threads=2)
        for scenario in benchmark.SCENARIOS:
            with self.subTest(scenario=scenario):
                commands = benchmark.commands_for(args, scenario)
                self.assertEqual("control" in commands,
                                 scenario in ("control_only", "traditional_joint"))
                self.assertEqual("vision" in commands, scenario != "control_only")
                if "vision" in commands:
                    command = commands["vision"]
                    self.assertEqual(command[command.index("--mode") + 1], "traditional")
                    self.assertEqual(command[command.index("--armor-config") + 1],
                                     "armor.yaml")
                    for option in ("--weights", "--num-classes", "--input-size"):
                        self.assertNotIn(option, command)


class RemovedDetectorCLITest(unittest.TestCase):
    def run_cli(self, script, *arguments):
        return subprocess.run([sys.executable, str(ROOT / "tools" / script), *arguments],
                              capture_output=True, text=True, timeout=5, cwd=ROOT)

    def test_scheduler_help_has_only_traditional_detection(self):
        for script, arguments in (
                ("benchmark_vision_control.py", ["--help"]),
                ("benchmark_vision_memory.py", ["--help"]),
                ("benchmark_vision_memory.py", ["compare", "--help"])):
            with self.subTest(script=script, arguments=arguments):
                result = self.run_cli(script, *arguments)
                self.assertEqual(result.returncode, 0, result.stderr)
                help_text = (result.stdout + result.stderr).lower()
                for removed in ("yolo", "onnx", "--weights", "--num-classes", "--input-size"):
                    self.assertNotIn(removed, help_text)

    def test_schedulers_reject_removed_model_arguments_without_outputs(self):
        with tempfile.TemporaryDirectory(prefix="removed-detector-cli-") as directory:
            output = Path(directory) / "report.json"
            for script, arguments in (
                    ("benchmark_vision_control.py", ["--video", "unused.avi"]),
                    ("benchmark_vision_memory.py", ["compare", "--image", "unused.png"])):
                for option in ("--weights", "--num-classes", "--input-size"):
                    with self.subTest(script=script, option=option):
                        result = self.run_cli(script, *arguments, "--output", str(output),
                                              option, "1")
                        self.assertEqual(result.returncode, 2, result.stderr)
                        self.assertIn("unrecognized arguments:", result.stderr)
                        self.assertIn(option, result.stderr)
                        self.assertFalse(output.exists())
                        self.assertFalse(output.with_suffix(".cases").exists())

    def test_scheduler_rejects_removed_detector_scenarios(self):
        with tempfile.TemporaryDirectory(prefix="removed-detector-scenario-") as directory:
            output = Path(directory) / "report.json"
            for scenario in ("yolo_only", "yolo_joint"):
                with self.subTest(scenario=scenario):
                    result = self.run_cli("benchmark_vision_control.py", "--video", "unused.avi",
                                          "--output", str(output), "--scenarios", scenario)
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertIn("invalid choice:", result.stderr)
                    self.assertIn(scenario, result.stderr)
                    self.assertFalse(output.exists())
                    self.assertFalse(output.with_suffix(".cases").exists())


if __name__ == "__main__":
    unittest.main()
