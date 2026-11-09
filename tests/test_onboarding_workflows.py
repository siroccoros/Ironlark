"""Drive launcher refusal through its CLI, without creating run artifacts."""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Workflows(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory(prefix="ironlark workflow ")
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name)
        shutil.copyfile(ROOT / "ironlark", self.root / "ironlark")
        (self.root / "tools").mkdir()
        for tool in ("receipts.py", "run_resources.py"):  # what the launcher imports
            shutil.copyfile(ROOT / "tools" / tool, self.root / "tools" / tool)

    def refused(self, expected, *args, webots_home=None):
        env = os.environ.copy()
        env.pop("WEBOTS_HOME", None)
        if webots_home:
            env["WEBOTS_HOME"] = webots_home
        result = subprocess.run([sys.executable, str(self.root / "ironlark"), *args],
                                capture_output=True, text=True, env=env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(expected, result.stderr)
        self.assertFalse((self.root / "runs").exists())
        self.assertFalse((self.root / ".cache/flight.lock").exists())

    def test_forest_world_refused_naming_main(self):
        self.refused("the empty and maze worlds (the forest lives on the main branch)", "run", "--world", "forest")

    def test_a_seed_without_the_maze_refused(self):
        self.refused("--seed is only available with --world maze", "run", "--seed", "3")

    def test_forest_and_renderer_commands_refused_naming_main(self):
        for command in ("build-world", "build-renderer", "render-reference"):
            self.refused("live on the main branch", command)

    def test_invalid_webots_home_refused_before_run_artifacts(self):
        self.refused("WEBOTS_HOME points to an invalid installation", "check",
                     webots_home=str(self.root / "no webots here"))

    def test_missing_webots_refused_before_run_artifacts(self):
        self.refused("Complete setup first", "run")


if __name__ == "__main__":
    unittest.main()
