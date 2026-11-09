"""Drive launcher refusal through its CLI, without creating run artifacts."""

import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from tests.test_renderer import ROOT, make_project, write_manifest


class Workflows(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory(prefix="ironlark workflow ")
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name)
        make_project(self.root)
        shutil.copyfile(ROOT / "ironlark", self.root / "ironlark")
        (self.root / "tools").mkdir()
        for tool in ("renderer_support.py", "receipts.py", "run_resources.py"):  # what the launcher imports
            shutil.copyfile(ROOT / "tools" / tool, self.root / "tools" / tool)

    def refused(self, expected):
        env = os.environ.copy()
        env.pop("WEBOTS_HOME", None)
        result = subprocess.run([sys.executable, str(self.root / "ironlark"), "run"],
                                capture_output=True, text=True, env=env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(expected, result.stderr)
        self.assertFalse((self.root / "runs").exists())
        self.assertFalse((self.root / ".cache/flight.lock").exists())

    def test_renderer_build_refused_while_a_flight_runs(self):
        lock = self.root / ".cache/flight.lock"
        lock.parent.mkdir(exist_ok=True)
        with lock.open("w") as held:
            fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)  # a flight, or the world `run` left open
            result = subprocess.run([sys.executable, str(self.root / "ironlark"), "build-renderer"],
                                    capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Another Ironlark flight or build is running", result.stderr)

    def test_missing_manifest_refused_before_run_artifacts(self):
        self.refused("build-renderer")

    def test_old_manifest_refused_before_run_artifacts(self):
        manifest = write_manifest(self.root)
        recorded = json.loads(manifest.read_text())
        del recorded["schema"]
        manifest.write_text(json.dumps(recorded))
        self.refused("schema")

    def test_changed_patch_refused_before_run_artifacts(self):
        write_manifest(self.root)
        (self.root / "native/patches/0001.patch").write_text("changed patch")
        self.refused("series_sha256")

    def test_working_tree_edit_refused_before_run_artifacts(self):
        write_manifest(self.root)
        (self.root / ".cache/webots-source/src/renderer.cpp").write_text("int value = 2;\n")
        self.refused("checkout_diff_sha256")


if __name__ == "__main__":
    unittest.main()
