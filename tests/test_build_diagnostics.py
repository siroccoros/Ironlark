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
import build_forest_world
import forest_pipeline
import forest_process
import receipts
import stage_cache

loader = SourceFileLoader("launcher", str(ROOT / "ironlark"))  # the launcher script, as a module
launcher = importlib.util.module_from_spec(importlib.util.spec_from_loader("launcher", loader))
loader.exec_module(launcher)


def setUpModule():
    """The digests these tests learn go to a log of their own, not the checkout's (.cache/digests.jsonl)."""
    directory = tempfile.TemporaryDirectory()
    unittest.addModuleCleanup(directory.cleanup)
    stage_cache.DIGESTS.__init__(Path(directory.name) / "digests.jsonl")


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



class Receipts(unittest.TestCase):
    """A forest world carries what it was built from; the pipeline resumes only a build of the same recipe."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        shutil.copytree(ROOT / "tools", self.root / "tools", ignore=shutil.ignore_patterns("__pycache__"))
        (self.root / "controllers/flight_observer").mkdir(parents=True)
        shutil.copyfile(ROOT / "controllers/flight_observer/minimap.py", self.root / "controllers/flight_observer/minimap.py")
        for package in ("numpy-2.2.6", "pillow-12.3.0", "etcpak-0.9.15"):
            (self.root / f".venv/lib/python3.12/site-packages/{package}.dist-info").mkdir(parents=True)
        self.world = self.root / "worlds/pine_forest"
        (self.world / "meshes").mkdir(parents=True)
        (self.world / "pine_forest.wbt").write_text("#VRML_SIM R2025a utf8\n")
        (self.world / "meshes/tree.obj").write_text("v 0 0 0\n")

    def test_the_recipe_covers_every_script_the_pipeline_runs_and_what_they_import(self):
        code = receipts.pipeline_code(ROOT)
        for name in ("forest_pipeline", "export_forest", "forest_mesh", "bake_impostors", "build_forest_world",
                     "forest_edge", "forest_process"):
            self.assertIn(f"tools/{name}.py", code)
        self.assertIn("controllers/flight_observer/minimap.py", code)  # the world's view, through look_at
        self.assertNotIn("tools/render_forest.py", code)

    def test_a_world_is_current_until_its_code_packages_or_files_change(self):
        receipts.write_world(self.world, receipts.recipe(self.root))
        self.assertEqual(receipts.world_problems(self.root, self.world), [])
        with (self.root / "tools/forest_mesh.py").open("a") as script:
            script.write("# changed\n")
        self.assertEqual(receipts.world_problems(self.root, self.world), ["tools/forest_mesh.py changed"])
        shutil.copyfile(ROOT / "tools/forest_mesh.py", self.root / "tools/forest_mesh.py")
        with (self.root / "controllers/flight_observer/minimap.py").open("a") as script:
            script.write("# changed\n")
        self.assertEqual(receipts.world_problems(self.root, self.world), ["controllers/flight_observer/minimap.py changed"])
        shutil.copyfile(ROOT / "controllers/flight_observer/minimap.py", self.root / "controllers/flight_observer/minimap.py")
        (self.root / ".venv/lib/python3.12/site-packages/numpy-2.2.6.dist-info").rename(
            self.root / ".venv/lib/python3.12/site-packages/numpy-2.3.0.dist-info")
        self.assertEqual(receipts.world_problems(self.root, self.world), ["numpy 2.2.6 is now 2.3.0"])

    def test_a_changed_or_missing_output_is_named(self):
        receipts.write_world(self.world, receipts.recipe(self.root))
        (self.world / "meshes/tree.obj").write_text("v 0 0 1\n")  # the same size: only its contents tell
        self.assertEqual(receipts.world_problems(self.root, self.world), [])
        self.assertEqual(receipts.world_problems(self.root, self.world, contents=True),
                         ["meshes/tree.obj is not the file the build wrote"])
        (self.world / "pine_forest.wbt").unlink()
        self.assertIn("pine_forest.wbt is not the file the build wrote", receipts.world_problems(self.root, self.world))
        (self.world / "receipt.json").unlink()
        self.assertEqual(receipts.world_problems(self.root, self.world),
                         ["it has no receipt: built before receipts, or not finished"])

    def test_a_build_of_the_world_it_already_has_builds_nothing(self):
        for name, value in (("SIM", self.root), ("CACHE", self.root / ".cache")):
            self.addCleanup(setattr, forest_pipeline, name, getattr(forest_pipeline, name))
            setattr(forest_pipeline, name, value)
        receipts.write_world(self.world, receipts.recipe(self.root))
        forest_pipeline.build_world()
        self.assertFalse((self.root / ".cache/forest-build").exists())  # no stage, no assembly, nothing emptied
        (self.world / "meshes/tree.obj").write_text("v 0 0 1\n")  # the same size: only its contents tell

        def fetch_source():  # the 2.7 GB download, and the hour of stages after it
            raise InterruptedError("the build began")

        self.addCleanup(setattr, forest_pipeline, "source_blend", forest_pipeline.source_blend)
        forest_pipeline.source_blend = fetch_source
        with self.assertRaisesRegex(InterruptedError, "the build began"):
            forest_pipeline.build_world()

    def test_a_world_whose_receipt_was_cut_short_is_built_again(self):
        for name, value in (("SIM", self.root), ("CACHE", self.root / ".cache")):
            self.addCleanup(setattr, forest_pipeline, name, getattr(forest_pipeline, name))
            setattr(forest_pipeline, name, value)
        receipts.write_world(self.world, receipts.recipe(self.root))
        receipt = self.world / "receipt.json"
        receipt.write_text(receipt.read_text()[:40])
        self.assertEqual(receipts.world_problems(self.root, self.world), ["its receipt is unreadable"])

        def fetch_source():
            raise InterruptedError("the build began")

        self.addCleanup(setattr, forest_pipeline, "source_blend", forest_pipeline.source_blend)
        forest_pipeline.source_blend = fetch_source
        with self.assertRaisesRegex(InterruptedError, "the build began"):
            forest_pipeline.build_world()

    def test_a_build_whose_code_changed_while_it_ran_is_not_certified(self):
        began = forest_pipeline.recipes(self.root)
        forest_pipeline.check_unchanged(self.root, began)
        with (self.root / "tools/forest_mesh.py").open("a") as script:
            script.write("# changed\n")
        with self.assertRaisesRegex(RuntimeError, r"\(tools/forest_mesh.py changed\)\. Build the world again"):
            forest_pipeline.check_unchanged(self.root, began)
        receipts.write_world(self.world, began["full"])  # the receipt names the code the build began with
        self.assertEqual(receipts.world_problems(self.root, self.world), ["tools/forest_mesh.py changed"])

    def test_the_export_recipe_leaves_out_the_stages_after_the_export(self):
        code = forest_pipeline.recipes(ROOT)["export"]["code"]
        for name in ("forest_pipeline", "export_forest", "export_forest_visuals", "bake_tree_materials",
                     "compose_forest_floor", "export_forest_sky", "stage_cache"):
            self.assertIn(f"tools/{name}.py", code)
        for name in ("build_forest_world", "bake_forest_light", "bake_impostors", "forest_valley", "dds"):
            self.assertNotIn(f"tools/{name}.py", code)

    def test_a_build_resumes_only_under_the_same_export_code(self):
        build = self.root / "forest-build"
        today = {"export": "export today", "full": "today"}
        forest_pipeline.start_build(build, self.world, today, skip_export=False)
        (build / "terrain.obj").write_text("stage output")
        assembly_edited = {"export": "export today", "full": "tomorrow"}
        forest_pipeline.start_build(build, self.world, assembly_edited, skip_export=False)
        self.assertTrue((build / "terrain.obj").exists())  # the exports stand
        export_edited = {"export": "export tomorrow", "full": "tomorrow"}
        forest_pipeline.start_build(build, self.world, export_edited, skip_export=False)
        self.assertFalse((build / "terrain.obj").exists())
        self.assertEqual(json.loads((build / "recipe.json").read_text()), "export tomorrow")

    def test_a_build_replaces_the_world_only_once_it_completes_even_through_a_link(self):
        main_world = self.root / "main checkout/worlds/pine_forest"
        (main_world / "meshes").mkdir(parents=True)
        (main_world / "meshes/material_bark_nor_gl.png").write_text("an earlier build's")
        (main_world / "receipt.json").write_text("an earlier build's")
        linked = self.root / "worktree/worlds/pine_forest"
        linked.parent.mkdir(parents=True)
        linked.symlink_to(main_world)
        build, recipe = self.root / "forest-build", {"export": "e", "full": receipts.recipe(self.root)}
        building = forest_pipeline.start_build(build, linked, recipe, skip_export=False)
        self.assertEqual(building, main_world.with_name(".pine_forest.building"))  # beside the folder, not the link
        self.assertEqual(list(building.iterdir()), [])
        (building / "pine_forest.wbt").write_text("this build's")
        building = forest_pipeline.start_build(build, linked, recipe, skip_export=False)  # the last was interrupted
        self.assertEqual(list(building.iterdir()), [])  # so nothing of it stands in for this build's files
        self.assertEqual(sorted(p.name for p in main_world.iterdir()), ["meshes", "receipt.json"])  # the last world
        (building / "pine_forest.wbt").write_text("this build's")
        forest_pipeline.finish_build(linked, building, recipe["full"])
        self.assertTrue(linked.is_symlink())
        self.assertEqual(sorted(p.name for p in main_world.iterdir()), ["pine_forest.wbt", "receipt.json"])
        self.assertEqual(receipts.world_problems(self.root, linked), [])
        self.assertEqual(sorted(p.name for p in main_world.parent.iterdir()), ["pine_forest"])
        first = self.root / "first checkout/worlds/pine_forest"  # no world yet: the folder moves into place
        building = forest_pipeline.start_build(build, first, recipe, skip_export=False)
        (building / "pine_forest.wbt").write_text("the first world")
        forest_pipeline.finish_build(first, building, recipe["full"])
        self.assertEqual((first / "pine_forest.wbt").read_text(), "the first world")

    def test_a_build_compiles_the_code_it_hashes(self):
        import receipts as imported  # compiled into tools/__pycache__ by this test run, if not already
        cache = Path(imported.__cached__).parent
        self.assertTrue(cache.is_dir())
        ran = []
        run_tool = launcher.run_tool
        self.addCleanup(setattr, launcher, "run_tool", run_tool)
        launcher.run_tool = lambda name, *args: ran.append((name, cache.exists()))  # Blender and the hour it takes
        launcher.build_world(skip_export=True)
        self.assertEqual(ran, [("forest_pipeline.py", False)])

    def test_skip_export_needs_bakes_today_s_pipeline_finished(self):
        build = self.root / "forest-build"
        today = {"export": "export today", "full": "today"}
        forest_pipeline.start_build(build, self.world, today, skip_export=False)
        with self.assertRaisesRegex(RuntimeError, "did not finish"):  # its bakes have not run
            forest_pipeline.start_build(build, self.world, today, skip_export=True)
        (build / "baked.json").write_text(json.dumps("today"))  # as build_world writes after the bakes
        forest_pipeline.start_build(build, self.world, today, skip_export=True)
        bake_edited = {"export": "export today", "full": "tomorrow"}
        with self.assertRaisesRegex(RuntimeError, "made by other code"):
            forest_pipeline.start_build(build, self.world, bake_edited, skip_export=True)
        forest_pipeline.start_build(build, self.world, today, skip_export=False)  # interrupted before its bakes
        with self.assertRaisesRegex(RuntimeError, "did not finish"):
            forest_pipeline.start_build(build, self.world, today, skip_export=True)


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


class StageStore(unittest.TestCase):
    """The stage store hands back a result only for the same code, inputs and toolchain, and only intact."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.build, self.store = self.root / "build", stage_cache.Store(self.root / "store")
        self.build.mkdir()
        self.input = self.root / "tree.obj"
        self.input.write_text("v 0 0 0\n")
        self.outputs = ["impostor_pine_01.npz"]

    def recipe(self):
        return stage_cache.descriptor(ROOT, "impostor", "tools/bake_impostors.py", ["pine_01"], {"tree": self.input},
                                      {"blender": "test"})

    def bake(self, text="capture"):
        (self.build / "impostor_pine_01.npz").write_text(text)
        self.store.save(self.recipe(), self.build, self.outputs, {"radius": 1})

    def manifest(self):
        return self.store.place(self.recipe()) / "manifest.json"

    def test_no_result_is_kept_for_code_changed_since_the_build_began(self):
        started = dict(stage_cache.AT_START)
        self.addCleanup(stage_cache.AT_START.update, started)
        stage_cache.AT_START["tools/bake_impostors.py"] = "the code the build began with"  # whatever its mtime
        with self.assertRaisesRegex(RuntimeError, "tools/bake_impostors.py changed while the world was building"):
            self.bake()
        self.assertFalse(self.store.place(self.recipe()).exists())
        stage_cache.AT_START.update(started)
        self.bake()
        self.assertTrue(self.manifest().is_file())

    def test_a_result_comes_back_after_the_build_folder_is_emptied(self):
        self.bake()
        shutil.rmtree(self.build)
        self.build.mkdir()
        kept = self.store.restore(self.recipe(), self.build, self.outputs)
        self.assertEqual(kept["value"], {"radius": 1})
        self.assertEqual(kept["outputs"]["impostor_pine_01.npz"][1], hashlib.sha256(b"capture").hexdigest())
        restored = self.build / "impostor_pine_01.npz"
        self.assertEqual(restored.read_text(), "capture")
        restored.write_text("rewritten")  # a file of its own: an older checkout's stage rewrites it in place
        self.assertEqual(self.store.restore(self.recipe(), self.build, self.outputs)["value"], {"radius": 1})
        self.assertEqual(restored.read_text(), "capture")

    def test_another_input_byte_is_another_result(self):
        self.bake()
        self.input.write_text("v 0 0 1\n")
        self.assertIsNone(self.store.restore(self.recipe(), self.build, self.outputs))

    def test_a_damaged_result_is_discarded(self):
        self.bake()
        stored = self.store.place(self.recipe()) / "outputs/impostor_pine_01.npz"
        stored.chmod(0o644)
        stored.write_text("captura")  # the same size
        self.assertIsNone(self.store.restore(self.recipe(), self.build, self.outputs))
        self.assertFalse(self.store.place(self.recipe()).exists())

    def test_a_result_without_its_manifest_is_never_found(self):
        self.bake()
        self.manifest().unlink()
        self.assertIsNone(self.store.restore(self.recipe(), self.build, self.outputs))

    def test_a_result_missing_an_output_the_stage_names_is_discarded(self):
        self.bake()  # stored before the stage named its second output
        self.assertIsNone(self.store.restore(self.recipe(), self.build, [*self.outputs, "impostor_pine_01.json"]))
        self.assertFalse(self.store.place(self.recipe()).exists())

    def test_after_a_build_the_store_keeps_what_each_checkout_s_latest_build_used(self):
        import time

        recipes = [stage_cache.descriptor(ROOT, "impostor", "tools/bake_impostors.py", [name], {"tree": self.input},
                                          {"blender": "test"}) for name in ("old", "main's", "branch's")]
        (self.build / "impostor_pine_01.npz").write_text("capture")
        for recipe in recipes:
            self.store.save(recipe, self.build, self.outputs)
        (self.store.place(recipes[0]).with_name("abandoned.part")).mkdir()
        main, branch = self.root / "main checkout", self.root / "worktree"
        main.mkdir()
        branch.mkdir()
        started = time.time()
        self.assertTrue(self.store.restore(recipes[2], self.build, self.outputs))
        self.store.collect(branch, started)  # the worktree's build used one; no build of main's has said what it uses
        started = time.time()
        self.assertIsNone(self.store.restore(recipes[1], self.build, self.outputs))
        self.store.save(recipes[1], self.build, self.outputs)  # main's build misses, and stores it again
        self.store.collect(main, started)  # from now on the worktree's stays as well
        self.assertEqual([self.store.place(recipe).exists() for recipe in recipes], [False, True, True])
        self.assertEqual(list(self.store.folder.glob("*/*.part")), [])
        shutil.rmtree(branch)  # a removed worktree keeps nothing
        self.store.collect(main, time.time())  # main's next build used none: nothing is left
        self.assertEqual([self.store.place(recipe).exists() for recipe in recipes], [False, False, False])

    def test_a_manifest_of_another_recipe_or_cut_short_is_discarded(self):
        for damage in (lambda text: text.replace('"pine_01"', '"pine_02"', 1), lambda text: text[: len(text) // 2]):
            self.bake()
            self.manifest().write_text(damage(self.manifest().read_text()))
            self.assertIsNone(self.store.restore(self.recipe(), self.build, self.outputs))
            self.assertFalse(self.store.place(self.recipe()).exists())


class Digests(unittest.TestCase):
    """A file is hashed once: its sha256 is known, to every build process and later builds, while its path holds the
    same file, and hashed again after any change, even one that keeps its size and mtime."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.digests = stage_cache.Digests(self.root / "digests.jsonl")
        self.file = self.root / "tree.obj"
        self.file.write_text("v 0 0 0\n")
        self.hashed = []
        digest = stage_cache.digest
        self.addCleanup(setattr, stage_cache, "digest", digest)
        stage_cache.digest = lambda path: self.hashed.append(Path(path).name) or digest(path)  # reads the file

    def test_a_file_is_hashed_once_and_again_after_any_change(self):
        self.assertEqual(self.digests(self.file), hashlib.sha256(b"v 0 0 0\n").hexdigest())
        self.assertEqual(self.digests(self.file), hashlib.sha256(b"v 0 0 0\n").hexdigest())
        self.assertEqual(self.hashed, ["tree.obj"])
        before = self.file.stat()
        self.file.write_text("v 0 0 1\n")  # the same size, and the mtime put back
        os.utime(self.file, ns=(before.st_atime_ns, before.st_mtime_ns))
        self.assertEqual(self.digests(self.file), hashlib.sha256(b"v 0 0 1\n").hexdigest())
        other = self.root / "other.obj"
        other.write_text("v 0 0 2\n")
        os.utime(other, ns=(before.st_atime_ns, before.st_mtime_ns))
        other.rename(self.file)  # another file in its place
        self.assertEqual(self.digests(self.file), hashlib.sha256(b"v 0 0 2\n").hexdigest())
        self.assertEqual(self.hashed, ["tree.obj"] * 3)

    def test_another_process_knows_what_one_hashed_and_a_cut_line_costs_only_itself(self):
        script = (f"import sys; sys.path.insert(0, {str(ROOT / 'tools')!r}); import stage_cache; "
                  f"print(stage_cache.Digests({str(self.digests.log)!r})({str(self.file)!r}))")
        child = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, check=True)
        self.assertEqual(self.digests(self.file), child.stdout.strip())
        self.assertEqual(self.hashed, [])  # this process read it from the log
        with self.digests.log.open("a") as log:
            log.write('["/a line a crash cut sh')
        later = stage_cache.Digests(self.digests.log)
        self.assertEqual(later(self.file), child.stdout.strip())
        self.assertEqual(self.hashed, [])

    def test_a_clone_or_a_moved_folder_is_known_and_forget_keeps_what_holds(self):
        digests = stage_cache.DIGESTS
        self.addCleanup(digests.__init__, digests.log)
        digests.__init__(self.root / "build.jsonl")
        (self.root / "partial").mkdir()
        stage_cache.clone(self.file, self.root / "partial/tree.obj", read_only=True)
        (self.root / "partial").rename(self.root / "kept")
        digests.moved(self.root / "partial", self.root / "kept")
        self.assertEqual(digests(self.root / "kept/tree.obj"), digests(self.file))
        self.assertEqual(self.hashed, ["tree.obj"])  # the source, once
        (self.root / "gone.obj").write_text("v 1 1 1\n")
        digests(self.root / "gone.obj")
        (self.root / "gone.obj").unlink()
        reader = stage_cache.Digests(digests.log)
        reader.catch_up()
        digests.forget()
        kept = [json.loads(line)[0] for line in digests.log.read_text().splitlines()]
        self.assertEqual(sorted(Path(path).name for path in kept), ["tree.obj", "tree.obj"])
        (self.root / "new.obj").write_text("v 2 2 2\n")
        digests(self.root / "new.obj")
        self.assertEqual(reader(self.root / "new.obj"), digests(self.root / "new.obj"))  # read again from the start
        self.assertEqual(self.hashed, ["tree.obj", "gone.obj", "new.obj"])


class Bakes(unittest.TestCase):
    """forest_pipeline.bakes reruns a bake exactly when a byte it reads changed (the stage itself, Blender, stands in
    as a function writing its outputs: these tests are about which bakes run)."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.build, self.world, self.source, self.stages = (self.root / name for name in ("build", "world", "source",
                                                                                         "stages"))
        (self.world / "meshes").mkdir(parents=True)
        (self.source / "textures").mkdir(parents=True)
        self.build.mkdir()
        for name in ("pine.obj", "fir.obj", "bark.png", "twigs.png", "bark_normal.png", "twigs_normal.png",
                     *(f"terrain_{x}_{y}.obj" for x in range(2) for y in range(2))):
            (self.world / "meshes" / name).write_text(name)
        (self.source / "polyhaven_pine_fir_forest.blend").write_text("scene")
        (self.source / "textures/evening_road_01_puresky_8k.exr").write_text("sky")
        self.place([0.0, 0.0, 0.0])
        self.declare("meshes/bark.png", "meshes/twigs.png")

    def place(self, translation):
        (self.world / "visual_instances.json").write_text(json.dumps(
            [{"mesh": "meshes/pine.obj", "texture": "meshes/twigs.png", "translation": translation}]))

    def declare(self, first, second):
        """Two variants; the second's two meshes wear first and second."""
        (self.build / "impostor_variants.json").write_text(json.dumps({
            "fir_01": [{"mesh": "meshes/fir.obj", "tex": "meshes/twigs.png", "normal": ""}],
            "pine_01": [{"mesh": "meshes/pine.obj", "tex": first, "normal": "meshes/bark_normal.png"},
                        {"mesh": "meshes/terrain_0_0.obj", "tex": second, "normal": "meshes/twigs_normal.png"}]}))

    def bake(self, world=None):
        ran = []

        def blender_stage(name, script, *args):
            run = len(ran) + 1
            if name == "canopy-light":
                ran.append(name)
                for output in [f"terrain_{x}_{y}_layers.npy" for x in range(2) for y in range(2)] + ["light_layers.json"]:
                    (self.build / output).write_text(f"light {run}")
            else:
                variants = args[args.index("--variants") + 1:]
                ran.append((name, *variants))
                for variant in variants:
                    (self.build / f"impostor_{variant}.npz").write_text(f"{variant} {run}")
                    (self.build / f"impostor_{variant}.json").write_text(json.dumps({"run": run}))

        forest_pipeline.bakes(self.build, world or self.world, self.source, self.stages, blender_stage)
        return ran

    def test_unchanged_bakes_come_back_without_running(self):
        self.assertEqual(self.bake(), ["canopy-light", ("impostors", "fir_01", "pine_01")])
        for path in self.build.glob("*_layers.npy"):
            path.unlink()
        self.assertEqual(self.bake(), [])
        self.assertEqual((self.build / "terrain_0_0_layers.npy").read_text(), "light 1")
        self.assertEqual(json.loads((self.build / "impostors.json").read_text()), {"fir_01": {"run": 2},
                                                                                   "pine_01": {"run": 2}})

    def test_another_checkout_with_the_same_world_bakes_nothing(self):
        self.bake()
        shutil.copytree(self.world, self.root / "other checkout/world")
        self.assertEqual(self.bake(self.root / "other checkout/world"), [])

    def test_a_moved_tree_bakes_the_light_again(self):
        self.bake()
        self.place([0.0, 1.0, 0.0])
        self.assertEqual(self.bake(), ["canopy-light"])

    def test_a_texture_on_another_mesh_captures_only_that_impostor_again(self):
        self.bake()
        self.declare("meshes/twigs.png", "meshes/bark.png")  # the same files, worn by the other mesh
        self.assertEqual(self.bake(), [("impostors", "pine_01")])
        self.assertEqual((self.build / "impostor_fir_01.npz").read_text(), "fir_01 2")  # the first run's, linked back
        self.assertEqual((self.build / "impostor_pine_01.npz").read_text(), "pine_01 1")
        self.assertEqual(json.loads((self.build / "impostors.json").read_text()), {"fir_01": {"run": 2},
                                                                                   "pine_01": {"run": 1}})


class Exports(unittest.TestCase):
    """forest_pipeline.exports runs a stage only when a byte it reads changed, each in a room holding only what it
    declares (Blender stands in as a function writing what each script would)."""

    def setUp(self):
        from PIL import Image

        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.source, self.build, self.stages = self.root / "pine-forest/source", self.root / "forest-build", self.root / "forest-stages"
        (self.source / "textures").mkdir(parents=True)
        for name, colour in (("bark_diff.png", (90, 60, 40)), ("moss_diff.png", (40, 90, 40))):
            Image.new("RGB", (8, 8), colour).save(self.source / "textures" / name)
        self.build.mkdir()

    def stage(self, name, script, *args, out):
        self.ran.append(name)
        present = sorted(str(path.relative_to(out)) for path in out.rglob("*") if path.is_file())
        expected = self.declared(name)
        self.assertEqual([p for p in present if not p.startswith("texture-source-2k/")], expected, name)
        # The exporters find the source through their folder's parent.
        self.assertTrue((out.resolve().parent / "pine-forest/source/textures/bark_diff.png").is_file(), name)
        files = {"export_forest.py": {"terrain.obj": "v 0 0 0", "terrain_meta.json": "{}", "heights.json": "{}",
                                      "manifest.json": "{}"},
                 "export_tree_layout.py": {"authored_trees.json": json.dumps([{"variant": "pine_01"}, {"variant": "fir_01"}])},
                 "export_cover_layout.py": {"cover.json": getattr(self, "cover", "[]")},
                 "export_forest_water.py": {"river.obj": "v 0 0 0", "river_meta.json": "{}"},
                 "export_floor_cover.py": {"floor_cover.jsonl.gz": "x", "floor_moss.png": "moss",
                                           "floor_cover_assets.json": json.dumps({"moss": {"image": "floor_moss.png"}})},
                 "compose_forest_floor.py": {f"terrain_{x}_{y}_floor.png": "floor" for x in range(2) for y in range(2)},
                 "export_forest_sky.py": {"sky_top.hdr": "sky"}}.get(script, {})
        if script == "export_forest_visuals.py" and "--prepare-only" in args:
            files = {"twig_atlas.json": json.dumps({"pine": [{"file": "twig_pine_0_top.png", "cell": 0}]}),
                     "twig_pine_0_top.png": "card", "material_images.json": json.dumps({"twig": ["card"]})}
        elif script == "export_forest_visuals.py":
            v = args[1]
            files = {f"visual_{v}.obj": v, f"material_{v}.blend": v, "twig_atlas.json": (out / "twig_atlas.json").read_text(),
                     "visual_trees.json": json.dumps({"version": 1, "variants": {v: [{"file": f"visual_{v}.obj"}]}}),
                     "tree_properties.json": json.dumps({v: {"base_z": 0}}),
                     "material_images.json": json.dumps({f"{v}_bark": ["bark"], "twig": ["card"]})}
        elif script == "export_forest_cover.py":
            files = {"cover_assets.json": (out / "cover.json").read_text()}
        elif script == "bake_tree_materials.py":
            files = {f"baked_{args[1]}.json": "{}"}
        elif script == "export_terrain_detail.py":
            files = {f"terrain_{args[1]}_{args[2]}_{kind}.png": kind for kind in ("diff", "normal", "mask")}
            files["terrain_generated.json"] = "{}"
        for name, text in files.items():
            (out / name).write_text(text)

    def declared(self, name):
        """What the room of a stage must hold, from what the stages before it wrote."""
        if name.startswith("tree-") and name != "tree-layout":
            return ["twig_atlas.json", "twig_pine_0_top.png"]
        if name.startswith("material-"):
            return [f"material_{name[len('material-'):]}.blend"]
        return {"cover": ["cover.json"], "floor-composite": sorted(
            ["floor_cover.jsonl.gz", "floor_cover_assets.json", "floor_moss.png", "terrain_meta.json",
             *(f"terrain_{x}_{y}_diff.png" for x in range(2) for y in range(2))])}.get(
            name, ["manifest.json", "terrain_meta.json"] if name.startswith("terrain-material-") else [])

    def export(self):
        self.ran = []
        forest_pipeline.exports(self.build, self.source, self.stages, self.stage, self.stage)
        return self.ran

    def contents(self):
        return {str(p.relative_to(self.build)): p.read_bytes() for p in sorted(self.build.rglob("*")) if p.is_file()}

    def test_an_unchanged_export_runs_no_stage_and_gives_the_same_files(self):
        first = self.export()
        # The terrain, tree layout, twigs, two trees and their materials, cover layout and cover, river, four tiles,
        # floor cover, its composite and the sky.
        self.assertEqual(len(first), 17)
        built = self.contents()
        self.assertEqual(list(json.loads(built["visual_trees.json"])["variants"]), ["fir_01", "pine_01"])
        self.assertEqual(json.loads(built["material_images.json"]), {"fir_01_bark": ["bark"], "twig": ["card"],
                                                                     "pine_01_bark": ["bark"]})
        shutil.rmtree(self.build)
        self.build.mkdir()
        self.assertEqual(self.export(), [])
        self.assertEqual(self.contents(), built)

    def test_a_stage_whose_output_changed_runs_the_stages_that_read_it_and_no_other(self):
        self.export()
        for manifest in self.stages.glob("cover-layout/*/manifest.json"):
            shutil.rmtree(manifest.parent)  # its result gone (as collect would remove it); this time it differs
        self.cover = '[{"asset": "rock"}]'
        self.assertEqual(self.export(), ["cover-layout", "cover"])
        self.assertEqual((self.build / "cover.json").read_text(), self.cover)
        self.assertEqual((self.build / "cover_assets.json").read_text(), self.cover)

    def test_a_room_finds_the_source_once_the_checkout_that_linked_it_is_gone(self):
        # A worktree links its .cache to the main checkout's, so the rooms (in the shared .cache) outlive it.
        cache, gate = self.root / "main/.cache", self.root / "gate"
        cache.mkdir(parents=True)
        shutil.move(self.source.parent, cache / "pine-forest")
        gate.mkdir()
        (gate / ".cache").symlink_to(cache)
        self.source, self.stages = gate / ".cache/pine-forest/source", gate / ".cache/forest-stages"
        self.export()
        shutil.rmtree(gate)
        self.source, self.stages = cache / "pine-forest/source", cache / "forest-stages"
        shutil.rmtree(self.stages)  # every stage runs again, from the main checkout
        self.assertEqual(len(self.export()), 17)


class Valley(unittest.TestCase):
    """The assembly takes an unchanged valley from the stage store (forest_valley.shapes, 80 s, stands in as a
    function writing its files)."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        self.build, self.out, self.stages = root / "build", root / "world", root / "stages"
        self.build.mkdir()
        (self.out / "meshes").mkdir(parents=True)
        (self.build / "heights.json").write_text("[[0, 1], [1, 2]]")
        self.runs = 0
        shapes, bounds = build_forest_world.forest_valley.shapes, build_forest_world.forest_backdrop.BOUNDS
        self.addCleanup(setattr, build_forest_world.forest_valley, "shapes", shapes)
        self.addCleanup(setattr, build_forest_world.forest_backdrop, "BOUNDS", bounds)
        build_forest_world.forest_valley.shapes = self.shapes
        build_forest_world.forest_backdrop.BOUNDS = (-137.1, 62.70469, -17.65625, 177.27589)

    def shapes(self, build, out):
        self.runs += 1
        for name in build_forest_world.forest_valley.FILES:
            (out / name).write_text(f"{name} {self.runs}")
        return [f"Shape {self.runs}"]

    def test_an_unchanged_valley_is_linked_back(self):
        self.assertEqual(build_forest_world.valley_shapes(self.build, self.out, self.stages), ["Shape 1"])
        shutil.rmtree(self.out / "meshes")
        self.assertEqual(build_forest_world.valley_shapes(self.build, self.out, self.stages), ["Shape 1"])
        self.assertEqual(self.runs, 1)
        self.assertEqual((self.out / "meshes/valley_near.obj").read_text(), "meshes/valley_near.obj 1")

    def test_another_height_grid_or_mirror_computes_it_again(self):
        build_forest_world.valley_shapes(self.build, self.out, self.stages)
        (self.build / "heights.json").write_text("[[0, 1], [1, 3]]")
        self.assertEqual(build_forest_world.valley_shapes(self.build, self.out, self.stages), ["Shape 2"])
        build_forest_world.forest_backdrop.BOUNDS = (-137.1, 62.70469, -17.65625, 177.3)
        self.assertEqual(build_forest_world.valley_shapes(self.build, self.out, self.stages), ["Shape 3"])
        self.assertEqual((self.out / "meshes/valley_roofs.obj").read_text(), "meshes/valley_roofs.obj 3")


class TerrainTiles(unittest.TestCase):
    def test_the_tiles_come_back_from_the_store_as_they_were_written(self):
        from PIL import Image

        with tempfile.TemporaryDirectory() as directory:
            build, out, stages = (Path(directory) / name for name in ("build", "world", "stages"))
            build.mkdir()
            (out / "meshes").mkdir(parents=True)
            # One square of ground, 0..20 m in x and -20..0 m in legacy z, its UVs over the four tiles.
            (build / "terrain.obj").write_text("v 0 0 0\nv 20 0 0\nv 20 0 -20\nv 0 0 -20\n"
                                               "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 1 0\n"
                                               "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")
            for x in range(2):
                for y in range(2):
                    for kind in ("floor", "normal"):
                        Image.new("RGB", (8, 8), (x * 100, y * 100, 50)).save(build / f"terrain_{x}_{y}_{kind}.png")
            bounds = build_forest_world.forest_backdrop.BOUNDS
            self.addCleanup(setattr, build_forest_world.forest_backdrop, "BOUNDS", bounds)
            build_forest_world.forest_backdrop.BOUNDS = (-1.0, 21.0, -1.0, 21.0)
            computed = build_forest_world.terrain_tiles(build, out, stages)
            written = {str(path.relative_to(out)): path.read_bytes() for path in out.rglob("*") if path.is_file()}
            self.assertEqual(sorted(written), sorted(__import__("forest_terrain").TILE_FILES))
            shutil.rmtree(out / "meshes")
            self.assertEqual(build_forest_world.terrain_tiles(build, out, stages), computed)
            self.assertEqual({name: (out / name).read_bytes() for name in written}, written)


class SourcePin(unittest.TestCase):
    def test_a_source_photograph_is_named_by_its_archive_not_read_again(self):
        from PIL import Image

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, out, stages = root / "source/textures", root / "world", root / "stages"
            source.mkdir(parents=True)
            (out / "meshes").mkdir(parents=True)
            Image.new("RGB", (8, 8), (90, 60, 40)).save(source / "bark_diff.png")
            for name, value in (("SOURCE", source.resolve()), ("SOURCE_ARCHIVE", "sha-1")):
                self.addCleanup(setattr, build_forest_world, name, getattr(build_forest_world, name))
                setattr(build_forest_world, name, value)
            url = "meshes/material_bark_diff_png.png"
            build_forest_world.stored_photograph(stages, out, url, source / "bark_diff.png")
            made = (out / url).read_bytes()
            (out / url).unlink()
            (source / "bark_diff.png").unlink()  # hashing it now would fail: the archive names it
            build_forest_world.stored_photograph(stages, out, url, source / "bark_diff.png")
            self.assertEqual((out / url).read_bytes(), made)
            build_forest_world.SOURCE_ARCHIVE = "sha-2"  # another archive: made again, from the file
            with self.assertRaises(FileNotFoundError):
                build_forest_world.stored_photograph(stages, out, url, source / "bark_diff.png")


class MissingBakes(unittest.TestCase):
    """The full assembly fails, naming what to run, when a bake it draws is missing, instead of leaving it out."""

    def test_a_missing_impostor_capture_or_light_layer_fails_the_assembly(self):
        import forest_fog

        with tempfile.TemporaryDirectory() as directory:
            build, out = Path(directory) / "build", Path(directory) / "world"
            build.mkdir()
            (out / "meshes").mkdir(parents=True)
            trees = {"pine_01": [{"mesh_url": "meshes/pine.obj", "tex": "meshes/bark.png"}]}
            with self.assertRaisesRegex(RuntimeError, "No impostor capture of pine_01.*build-world"):
                build_forest_world.impostor_shapes(build, out, None, trees, [])
            self.assertEqual(build_forest_world.impostor_shapes(build, out, None, trees, [], declare_only=True), [])
            with self.assertRaisesRegex(RuntimeError, "lacks the canopy light bake's terrain_0_0_layers.npy.*build-world"):
                forest_fog.light_occlusion_fields(build, out, (0, 0, 0), (-1, 1, -1, 1))


class Textures(unittest.TestCase):
    """The assembly's BC7 textures come from the stage store while the PNG and the encoder are unchanged."""

    def test_an_unchanged_texture_is_linked_back_and_an_edited_one_encoded(self):
        from PIL import Image

        with tempfile.TemporaryDirectory() as directory:
            out, stages = Path(directory) / "world", Path(directory) / "stages"
            (out / "meshes").mkdir(parents=True)
            png = out / "meshes/bark.png"
            Image.new("RGBA", (64, 64), (90, 60, 40, 255)).save(png)
            stored = []
            for colour in ((90, 60, 40, 255), (90, 60, 40, 255), (40, 60, 90, 255)):
                Image.new("RGBA", (64, 64), colour).save(png)
                (out / "pine_forest.wbt").write_text('url "meshes/bark.png"\n')
                build_forest_world.compress_textures(out, stages)
                self.assertEqual((out / "pine_forest.wbt").read_text(), 'url "meshes/bark.dds"\n')
                stored.append(sorted(path.read_bytes() for path in stages.glob("bc7/*/outputs/meshes/bark.dds")))
                self.assertIn((out / "meshes/bark.dds").read_bytes(), stored[-1])
            self.assertEqual([len(results) for results in stored], [1, 1, 2])  # encoded, taken back, encoded


if __name__ == "__main__":
    unittest.main()
