#!/usr/bin/env python3
"""Exercise the real video worker's synchronization, EOF, and failure contract.

Run: python3 test/vision/test_video_worker.py --worker build/vision_video_worker
The tiny MJPEG fixture requires Python OpenCV only to encode test data.
"""
import argparse
import json
import selectors
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None


def json_line(process, timeout=15):
    with selectors.DefaultSelector() as selector:
        selector.register(process.stdout, selectors.EVENT_READ)
        if not selector.select(timeout):
            raise AssertionError("Worker did not produce JSON before deadline")
        line = process.stdout.readline()
    if not line:
        raise AssertionError("Worker exited before expected JSON")
    return json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(
        AssertionError("Non-finite JSON value: " + value)))


@unittest.skipIf(cv2 is None, "Python OpenCV is needed to encode the MJPEG fixture")
class VideoWorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="vision-worker-")
        cls.video = Path(cls.temp.name) / "two frames.avi"
        writer = cv2.VideoWriter(str(cls.video), cv2.VideoWriter_fourcc(*"MJPG"),
                                 30, (128, 96))
        if not writer.isOpened():
            raise unittest.SkipTest("MJPEG encoder unavailable")
        for offset in (0, 4):
            image = np.zeros((96, 128, 3), dtype=np.uint8)
            image[25:70, 30 + offset:36 + offset, 0] = 255
            image[25:70, 90 + offset:96 + offset, 0] = 255
            writer.write(image)
        writer.release()

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def start(self):
        process = subprocess.Popen([
            str(WORKER), "--mode", "traditional", "--video", str(self.video),
            "--width", "128", "--height", "96", "--fps", "30", "--threads", "2",
            "--armor-config", str(ROOT / "config/armor_auto.yaml"),
            "--benchmark-worker"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1)
        self.addCleanup(self.stop, process)
        return process

    @staticmethod
    def stop(process):
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)

    def test_real_worker_eof_and_release_handshake(self):
        process = self.start()
        ready = json_line(process)
        self.assertEqual(ready["event"], "ready")
        self.assertEqual(ready["mode"], "traditional")
        self.assertFalse(ready["pose_enabled"])
        self.assertEqual((ready["source_width"], ready["source_height"]), (128, 96))
        start = time.monotonic() + .1
        process.stdin.write(json.dumps({"start_monotonic_s": start,
                                        "warmup_s": .1, "measure_s": .25}) + "\n")
        process.stdin.flush()
        result = json_line(process)
        self.assertEqual(result["event"], "result")
        self.assertEqual(result["mode"], "traditional")
        removed_fields = {"weights", "model_sha256", "num_classes",
                          "network_input_size", "model_scope", "benchmark_model"}
        self.assertFalse(removed_fields.intersection(ready))
        self.assertFalse(removed_fields.intersection(result))
        self.assertTrue(result["success"])
        self.assertGreater(result["measured_frames"], 0)
        self.assertGreater(result["eof_loops"], 0)
        self.assertGreater(result["peak_rss_kib"], 0)
        self.assertGreaterEqual(result["actual_start_monotonic_s"], start)
        self.assertIsNone(process.poll(), "Worker must remain resident for final sample")
        process.stdin.write('{"event":"release"}\n')
        process.stdin.flush()
        _, error = process.communicate(timeout=5)
        self.assertEqual(process.returncode, 0, error)

    def test_invalid_schedule_never_reports_success(self):
        process = self.start()
        self.assertEqual(json_line(process)["event"], "ready")
        process.stdin.write(json.dumps({"start_monotonic_s": time.monotonic(),
                                        "warmup_s": 0, "measure_s": -1}) + "\n")
        process.stdin.flush()
        output, error = process.communicate(timeout=5)
        self.assertNotEqual(process.returncode, 0)
        self.assertNotIn('"success":true', output)
        self.assertIn("schedule", error)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    args, remaining = parser.parse_known_args()
    WORKER, ROOT = args.worker.resolve(), args.root.resolve()
    unittest.main(argv=[__file__] + remaining)
