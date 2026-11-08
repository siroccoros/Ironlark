"""Renderer selection, live resources and prerequisites on real files and Git checkouts."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import renderer_support as renderer
import build_webots_renderer as builder


def git(source, *args):
    return subprocess.check_output(["git", "-C", str(source), *args], text=True, stderr=subprocess.PIPE).strip()


def make_project(root):
    for name in renderer.NATIVE_NAMES:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(name)
    patches = root / "native/patches"
    patches.mkdir()
    (patches / "0001.patch").write_text("patch")
    source = root / ".cache/webots-source"
    (source / "resources/wren/shaders").mkdir(parents=True)
    (source / "resources/wren/shaders/pbr.frag").write_text("stock shader")
    (source / "src").mkdir()
    (source / "src/renderer.cpp").write_text("int value = 1;\n")
    git(source, "init", "-q")
    git(source, "add", ".")
    git(source, "-c", "user.name=Test", "-c", "user.email=test@example.com", "commit", "-qm", "Initial source")
    for home in (root / "webots", root / ".cache/webots-renderer"):
        (home / "resources").mkdir(parents=True)
        (home / "resources/version.txt").write_text(renderer.RELEASE)
        for name in ("webots", "bin/webots-bin"):
            path = home / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("#!/bin/sh\nexit 0\n")
            path.chmod(0o755)
    for name in ("include/controller/c/webots/robot.h", "include/ode/ode/ode.h",
                 "include/qt/QtCore/QtCore/qglobal.h", "lib/webots/libQt6Core.so"):
        path = root / "webots" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("SDK fixture")
    return source


def compilers_on_path(directory, **scripts):
    """gcc and g++ as shell scripts ahead of the real ones on PATH, as the environment of a subprocess."""
    directory.mkdir(exist_ok=True)
    for name, body in scripts.items():
        path = directory / name.replace("gxx", "g++")
        path.write_text("#!/bin/sh\n" + body + "\n")
        path.chmod(0o755)
    return {"PATH": f"{directory}{os.pathsep}{os.environ['PATH']}"}


def write_manifest(root):
    path = root / ".cache/webots-renderer/ironlark-renderer.json"
    path.write_text(json.dumps(renderer.renderer_identity(root)))
    return path


class Selection(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory()
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name)
        self.source = make_project(self.root)
        self.stock = self.root / "webots"
        self.home = self.root / ".cache/webots-renderer"
        self.manifest = self.home / "ironlark-renderer.json"

    def test_forest_rejects_stock_override(self):
        write_manifest(self.root)
        with self.assertRaisesRegex(RuntimeError, "WEBOTS_HOME.*stock Webots"):
            renderer.select_webots(self.root, "forest", str(self.stock))

    def test_forest_rejects_missing_manifest(self):
        with self.assertRaisesRegex(RuntimeError, "build-renderer first[.]$"):
            renderer.select_webots(self.root, "forest")

    def test_forest_rejects_malformed_manifest(self):
        for contents in ("", "{", "[]", "null", '{"build_in_progress": true}'):
            self.manifest.write_text(contents)
            with self.assertRaises(RuntimeError):
                renderer.select_webots(self.root, "forest")

    def test_old_manifest_requires_rebuild(self):
        recorded = renderer.renderer_identity(self.root)
        del recorded["schema"]
        self.manifest.write_text(json.dumps(recorded))
        with self.assertRaisesRegex(RuntimeError, "schema"):
            renderer.select_webots(self.root, "forest")

    def test_every_identity_field_is_checked(self):
        for field in renderer.renderer_identity(self.root):
            recorded = renderer.renderer_identity(self.root)
            recorded[field] = "stale"
            self.manifest.write_text(json.dumps(recorded))
            with self.assertRaisesRegex(RuntimeError, field):
                renderer.select_webots(self.root, "forest")

    def test_changed_native_source_and_patch_rejected(self):
        for name in (renderer.NATIVE_NAMES[0], "native/patches/0001.patch"):
            write_manifest(self.root)
            (self.root / name).write_text("changed")
            with self.assertRaisesRegex(RuntimeError, "stale"):
                renderer.select_webots(self.root, "forest")

    def test_staged_and_unstaged_edits_require_build_but_dev_build_is_accepted(self):
        write_manifest(self.root)
        path = self.source / "src/renderer.cpp"
        path.write_text("int value = 2;\n")
        with self.assertRaisesRegex(RuntimeError, "checkout_diff"):
            renderer.select_webots(self.root, "forest")
        git(self.source, "add", "src/renderer.cpp")
        write_manifest(self.root)
        self.assertEqual(renderer.select_webots(self.root, "forest"), self.home)
        path.write_text("int value = 3;\n")
        with self.assertRaisesRegex(RuntimeError, "checkout_diff"):
            renderer.select_webots(self.root, "forest")

    def test_checkout_commit_change_requires_build(self):
        write_manifest(self.root)
        git(self.source, "-c", "user.name=Test", "-c", "user.email=test@example.com",
            "commit", "--allow-empty", "-qm", "New development commit")
        with self.assertRaisesRegex(RuntimeError, "checkout_commit"):
            renderer.select_webots(self.root, "forest")

    def test_empty_allows_stock_even_with_stale_renderer(self):
        self.manifest.write_text("{}")
        self.assertEqual(renderer.select_webots(self.root), self.stock)
        self.assertEqual(renderer.select_webots(self.root, "empty", str(self.stock)), self.stock)

    def test_current_manifest_and_symlink_override_accepted(self):
        write_manifest(self.root)
        alias = self.root / "a path with spaces"
        alias.symlink_to(self.home, target_is_directory=True)
        self.assertEqual(renderer.select_webots(self.root, "forest", str(alias)), self.home)

    def test_incomplete_runtime_rejected(self):
        write_manifest(self.root)
        (self.home / "bin/webots-bin").chmod(0o644)
        with self.assertRaisesRegex(RuntimeError, "missing executable"):
            renderer.select_webots(self.root, "forest")

    def test_actual_release_checked(self):
        write_manifest(self.root)
        (self.home / "resources/version.txt").write_text("R2024a")
        with self.assertRaisesRegex(RuntimeError, "R2025a"):
            renderer.select_webots(self.root, "forest")

    def test_edits_to_served_resources_are_live_and_others_require_build(self):
        shaders = self.stock / "resources/wren/shaders"
        shaders.mkdir(parents=True)
        (shaders / "pbr.frag").write_text("stock shader")
        (shaders / "skybox.frag").write_text("stock sky")
        (self.source / "resources/wren/shaders/skybox.frag").write_text("stock sky")
        (self.source / "resources/Makefile.include").write_text("CC = gcc\n")
        git(self.source, "add", ".")
        git(self.source, "-c", "user.name=Test", "-c", "user.email=test@example.com", "commit", "-qm", "Resources")
        base = git(self.source, "rev-parse", "HEAD")
        shader = self.source / "resources/wren/shaders/pbr.frag"
        shader.write_text("patch in progress")
        builder.overlay_resources(self.home, self.stock, self.source, base)
        write_manifest(self.root)
        shader.write_text("edited after the build")
        self.assertEqual(renderer.select_webots(self.root, "forest"), self.home)
        (self.source / "resources/wren/shaders/skybox.frag").write_text("edited sky")
        with self.assertRaisesRegex(RuntimeError, "skybox.frag is edited but served from stock"):
            renderer.select_webots(self.root, "forest")
        (self.source / "resources/wren/shaders/skybox.frag").write_text("stock sky")
        (self.source / "resources/Makefile.include").write_text("CC = clang\n")
        with self.assertRaisesRegex(RuntimeError, "checkout_diff"):
            renderer.select_webots(self.root, "forest")

    def test_live_overlay_serves_working_tree_and_new_nested_resources(self):
        stock_shader = self.stock / "resources/wren/shaders/pbr.frag"
        stock_shader.parent.mkdir(parents=True)
        stock_shader.write_text("stock shader")
        base = git(self.source, "rev-parse", "HEAD")
        shader = self.source / "resources/wren/shaders/pbr.frag"
        shader.write_text("development shader")
        added = self.source / "resources/new/nested/node.wrl"
        added.parent.mkdir(parents=True)
        added.write_text("new node")
        git(self.source, "add", "resources/new/nested/node.wrl")
        builder.overlay_resources(self.home, self.stock, self.source, base)
        served = self.home / "resources/wren/shaders/pbr.frag"
        self.assertTrue(served.is_symlink())
        self.assertEqual(served.read_text(), "development shader")
        shader.write_text("edited again")
        self.assertEqual(served.read_text(), "edited again")
        self.assertEqual((self.home / "resources/new/nested/node.wrl").read_text(), "new node")
        self.assertEqual(stock_shader.read_text(), "stock shader")
        builder.overlay_resources(self.home, self.stock, self.source, base)
        self.assertEqual(served.read_text(), "edited again")


class Prerequisites(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory()
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name)
        make_project(self.root)

    def preflight(self, **variables):
        env = {**os.environ, **variables}
        code = "import json,sys; sys.path.insert(0,sys.argv[1]); import renderer_support as r; print(json.dumps(r.renderer_preflight(sys.argv[2])))"
        return json.loads(subprocess.check_output([sys.executable, "-c", code, str(ROOT / "tools"), str(self.root)], env=env, text=True))

    @unittest.skipUnless(shutil.which("gcc") and shutil.which("g++"), "C and C++ compilers required")
    def test_missing_headers_reported_together_using_real_compiler(self):
        missing = self.preflight(**compilers_on_path(self.root / "bin", gxx=f'exec {shutil.which("g++")} -nostdinc "$@"'),
                                 CFLAGS="ignored-by-webots", LDFLAGS="ignored-by-webots")
        for label in ("OpenGL", "GLU", "OpenAL", "FreeType", "zlib"):
            self.assertTrue(any(item.startswith(label + ":") for item in missing), missing)
        self.assertFalse(any("overrides" in item for item in missing))

    def test_failed_compilers_reported(self):
        missing = self.preflight(**compilers_on_path(self.root / "bin", gcc="exit 1", gxx="exit 1"))
        self.assertIn("c compiler compile/link: gcc", missing)
        self.assertIn("c++ compiler compile/link: g++", missing)

    def test_incomplete_sdk_reports_missing_parts(self):
        (self.root / "webots/include/ode/ode/ode.h").unlink()
        self.assertIn("installed SDK missing: include/ode/ode/ode.h", renderer.sdk_problems(self.root / "webots"))

    def test_failed_preflight_stops_builder_before_checkout_and_downloads(self):
        shutil.rmtree(self.root / ".cache/webots-source")
        (self.root / "tools").mkdir()
        for name in ("renderer_support.py", "build_webots_renderer.py", "forest_process.py"):
            shutil.copyfile(ROOT / "tools" / name, self.root / "tools" / name)
        result = subprocess.run([sys.executable, str(self.root / "tools/build_webots_renderer.py")],
                                env={**os.environ, **compilers_on_path(self.root / "bin", gcc="exit 1", gxx="exit 1")},
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("c compiler compile/link", result.stderr)
        self.assertIn("c++ compiler compile/link", result.stderr)
        self.assertFalse((self.root / ".cache/webots-source").exists())
        self.assertFalse((self.root / ".cache/Qt").exists())


class Downloads(unittest.TestCase):
    def test_corrupt_archive_replaced_and_completed_download_reused(self):
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            remote, archive = root / "remote", root / "archive"
            remote.write_bytes(b"complete archive")
            archive.write_bytes(b"incomplete")
            archive.with_name("archive.part").write_bytes(b"interrupted")
            expected = hashlib.sha256(remote.read_bytes()).hexdigest()
            builder.fetch_archive(remote.as_uri(), archive, expected)
            remote.unlink()
            builder.fetch_archive(remote.as_uri(), archive, expected)
            self.assertEqual(archive.read_bytes(), b"complete archive")
            self.assertFalse(archive.with_name("archive.part").exists())

    def test_bad_download_does_not_replace_existing_archive(self):
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            remote, archive = root / "remote", root / "archive"
            remote.write_bytes(b"wrong contents")
            archive.write_bytes(b"previous")
            with self.assertRaisesRegex(RuntimeError, "Unexpected dependency"):
                builder.fetch_archive(remote.as_uri(), archive, "wrong digest")
            self.assertEqual(archive.read_bytes(), b"previous")


class Install(unittest.TestCase):
    def test_a_running_executable_is_replaced_not_overwritten(self):
        with tempfile.TemporaryDirectory() as scratch:
            target, built = Path(scratch, "webots-bin"), Path(scratch, "built")
            shutil.copy2(shutil.which("sleep"), target)
            built.write_bytes(b"the new build")
            running = subprocess.Popen(["sleep", "30"], executable=str(target))  # a flight's Webots
            self.addCleanup(running.wait)
            self.addCleanup(running.kill)
            builder.install(built, target)  # copied in place, this fails: Text file busy
            self.assertIsNone(running.poll())
            self.assertEqual(target.read_bytes(), b"the new build")
            self.assertFalse(target.with_name("webots-bin.part").exists())


if __name__ == "__main__":
    unittest.main()
