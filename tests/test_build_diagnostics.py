"""Compiler diagnostics and failed stages on real processes and log files."""

import hashlib
import importlib.util
import json
from importlib.machinery import SourceFileLoader
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import blender
import forest_process

loader = SourceFileLoader("launcher", str(ROOT / "ironlark"))  # the launcher script, as a module
launcher = importlib.util.module_from_spec(importlib.util.spec_from_loader("launcher", loader))
loader.exec_module(launcher)


class Diagnostics(unittest.TestCase):
    def test_interleaved_early_errors_and_invalid_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "build log.txt"
            for error in (b"fatal error: GL/gl.h: missing", b"undefined reference to glGetError",
                          b"Permission denied", b"No space left on device", b"Killed signal terminated program"):
                log.write_bytes(b"context\xff\n" + error + b"\n" + b"compiling other file\n" * 100)
                excerpt = forest_process.error_excerpt(log)
                self.assertIn(error.decode(), excerpt)
                self.assertLessEqual(len(excerpt.splitlines()), 18)
                self.assertIn("context", excerpt)

    @unittest.skipUnless(shutil.which("g++"), "C++ compiler required")
    def test_missing_header_printed_with_complete_log_and_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, log = root / "broken.cpp", root / "build.log"
            source.write_text('#include "ironlark-missing-test-header.h"\n')
            with self.assertRaisesRegex(RuntimeError, "ironlark-missing-test-header.h"):
                forest_process.run_stage(["g++", "-c", str(source), "-o", str(root / "broken.o")], log)
            self.assertIn("fatal error:", log.read_text())
            metrics = json.loads(log.with_suffix(".metrics.json").read_text())
            self.assertNotEqual(metrics["exit_code"], 0)
            self.assertIn("exited", metrics["error"])

    def test_forked_workers_count_their_shared_pages_once(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "stage.log"
            # 200 MiB written by the parent, then shared by two forked children: 600 MiB resident, 200 MiB of it.
            forest_process.run_stage([sys.executable, "-c", (
                "import os, time\n"
                "data = bytearray(200 << 20)\n"
                "for i in range(0, len(data), 4096): data[i] = 1\n"
                "for _ in range(2):\n"
                "    if os.fork() == 0:\n"
                "        time.sleep(1.5)\n"
                "        os._exit(0)\n"
                "time.sleep(1.5)\n")], log)
            peak = json.loads(log.with_suffix(".metrics.json").read_text())["peak_pss_mib"]
            self.assertGreater(peak, 200)
            self.assertLess(peak, 400)

    def test_spawn_failure_writes_log_and_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / "build.log"
            with self.assertRaisesRegex(RuntimeError, "No such file"):
                forest_process.run_stage([str(root / "missing-compiler")], log)
            self.assertIn("No such file", log.read_text())
            metrics = json.loads(log.with_suffix(".metrics.json").read_text())
            self.assertIsNone(metrics["exit_code"])
            self.assertIn("No such file", metrics["error"])


@unittest.skipUnless(shutil.which("cc") or shutil.which("gcc"), "C compiler required")
class Lockstep(unittest.TestCase):
    """The launcher compiles the flight bridge's C loop for each run against the pinned Webots, or stops the flight."""

    def build(self, source):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        (root / "controllers/flight_bridge").mkdir(parents=True)
        (root / "controllers/flight_bridge/lockstep.c").write_text(source)
        self.addCleanup(setattr, launcher, "ROOT", launcher.ROOT)
        launcher.ROOT = root
        return launcher.build_lockstep(ROOT / "webots")

    def test_the_library_is_built_from_the_source_at_hand(self):
        source = (ROOT / "controllers/flight_bridge/lockstep.c").read_text()
        resend = source.replace("readable(sock, 1.0)", "readable(sock, 2.0)")
        self.assertNotEqual(resend, source)
        self.assertTrue(self.build(source).startswith(b"\x7fELF"))
        self.assertNotEqual(self.build(resend), self.build(source))

    def test_a_source_that_does_not_compile_stops_the_flight(self):
        source = (ROOT / "controllers/flight_bridge/lockstep.c").read_text().replace("int motor_count,", "int motor_count")
        with self.assertRaisesRegex(RuntimeError, "(?s)does not compile.*error"):
            self.build(source)


class Drone(unittest.TestCase):
    """The launcher rebuilds the drone unless its receipt matches every input and output."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        (self.root / "protos/drone").mkdir(parents=True)
        self.addCleanup(setattr, launcher, "ROOT", launcher.ROOT)
        launcher.ROOT = self.root
        files = {"config/sensors.json": "{}", "protos/IronlarkDrone.proto": "PROTO", "protos/drone/body.obj": "v 0 0 0"}
        for name, text in files.items():
            (self.root / name).parent.mkdir(parents=True, exist_ok=True)
            (self.root / name).write_text(text)
        digests = {name: hashlib.sha256(text.encode()).hexdigest() for name, text in files.items()}
        for package in ("numpy-2.2.6", "pillow-12.3.0"):
            (self.root / f".venv/lib/python3.12/site-packages/{package}.dist-info").mkdir(parents=True)
        (self.root / "protos/drone/receipt.json").write_text(json.dumps({
            "inputs": {"config/sensors.json": digests["config/sensors.json"]},
            "packages": {"numpy": "2.2.6", "pillow": "12.3.0"},
            "outputs": {name: digests[name] for name in files if name.startswith("protos/")}}))

    def test_a_complete_build_is_current(self):
        self.assertIsNone(launcher.drone_stale())

    def test_a_changed_input_or_output_or_a_missing_one_is_stale(self):
        (self.root / "config/sensors.json").write_text('{"camera": {}}')
        self.assertEqual(launcher.drone_stale(), "config/sensors.json changed")
        (self.root / "config/sensors.json").write_text("{}")
        (self.root / "protos/drone/body.obj").unlink()
        self.assertEqual(launcher.drone_stale(), "protos/drone/body.obj changed")
        (self.root / "protos/drone/body.obj").write_text("v 0 0 0")
        site = self.root / ".venv/lib/python3.12/site-packages"
        (site / "pillow-12.3.0.dist-info").rename(site / "pillow-12.4.0.dist-info")
        self.assertIn("now {'numpy': '2.2.6', 'pillow': '12.4.0'}", launcher.drone_stale())
        (self.root / "protos/drone/receipt.json").unlink()
        self.assertEqual(launcher.drone_stale(), "no completed build")

    def test_a_receipt_cut_short_is_stale(self):
        receipt = self.root / "protos/drone/receipt.json"
        receipt.write_text(receipt.read_text()[:40])
        self.assertEqual(launcher.drone_stale(), "its receipt is unreadable")


class Extraction(unittest.TestCase):
    """Blender is extracted once per archive (tools/blender.py), and again when its files change."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def test_an_extraction_belongs_to_its_archive(self):
        import tarfile

        archive = self.root / "source.tar"
        (self.root / "packed").mkdir()
        (self.root / "packed/scene.blend").write_text("first")
        with tarfile.open(archive, "w") as tar:
            tar.add(self.root / "packed", arcname="packed")
        program = self.root / "program"
        blender.unpack(archive, program, tarfile.open, "sha-1", member="packed", added_ok=True)
        (program / "__pycache__").mkdir()
        (program / "__pycache__/module.pyc").write_text("written by the program, later")
        blender.unpack(archive, program, tarfile.open, "sha-1", member="packed", added_ok=True)
        self.assertTrue((program / "__pycache__/module.pyc").exists())  # a program's own folder: added files stand
        target = self.root / "source"
        blender.unpack(archive, target, tarfile.open, "sha-1", member="packed")
        with self.assertRaises(PermissionError):  # the source is read-only, so a build reads what the archive holds
            (target / "scene.blend").write_text("saved")
        with self.assertRaises(PermissionError):
            (target / "scene.blend@").write_text("Blender's save, renamed over the scene after")
        extracted = (target / "scene.blend").stat().st_ino
        blender.unpack(archive, target, tarfile.open, "sha-1", member="packed")
        self.assertEqual((target / "scene.blend").stat().st_ino, extracted)  # read-only and unchanged: trusted
        for edit in (lambda: (target / "scene.blend.png").write_text("found before scene.blend by a texture lookup"),
                     lambda: (target / "scene.blend").write_text("saved"),  # the same size: its mtime tells
                     lambda: (target / "scene.blend").unlink(),
                     lambda: None):  # made writable, edits that keep sizes and mtimes could have gone unseen
            blender.freeze(target, False)  # whoever makes it writable again
            edit()
            blender.unpack(archive, target, tarfile.open, "sha-1", member="packed")
            self.assertEqual(sorted(p.name for p in target.iterdir()), [".archive-sha256", "scene.blend"])
            self.assertEqual((target / "scene.blend").read_text(), "first")
            self.assertNotEqual((target / "scene.blend").stat().st_ino, extracted)  # extracted again
            extracted = (target / "scene.blend").stat().st_ino
        blender.freeze(target, False)
        (target / "scene.blend").write_text("edited")
        (target / ".archive-sha256").write_text("sha-1")  # as the extraction marker was before it listed files
        blender.unpack(archive, target, tarfile.open, "sha-1", member="packed")
        self.assertEqual((target / "scene.blend").read_text(), "first")
        blender.unpack(archive, target, tarfile.open, "sha-2", member="packed")
        self.assertEqual((target / "scene.blend").read_text(), "first")  # another archive: extracted again
        blender.unpack(archive, target, tarfile.open, "sha-1", member="packed")
        self.addCleanup(blender.freeze, target, False)  # so the temporary folder can be removed
