"""Build Ironlark's rendering-only Webots patch against the installed R2025a SDK."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
from urllib.request import urlretrieve

from forest_process import run_stage
from renderer_support import RELEASE, COMMIT, NATIVE_NAMES, renderer_identity, require_renderer_prerequisites

SIM = Path(__file__).resolve().parents[1]
CACHE = SIM / ".cache"
PATCHES = SIM / "native/patches"
BRANCH = "ironlark"
# Compiled into WREN beside the patched sources: Ironlark's vertex indexer, and meshoptimizer's vertex cache
# optimizer (MIT, zeux/meshoptimizer 9e1f07b).
NATIVE_SOURCES = [SIM / name for name in NATIVE_NAMES]
PACKAGES = {
    "libOIS.1.4.tar.bz2": "ec13db6efd6901e80e9c4b437319c7949253a47fee768515a3aa1e6ca122225d",
    "libassimp-5.2.3.tar.bz2": "31c12e4e9f6bf52259dc599c8fcdb77c511c170e18cdc70543b10903aa28c697",
    "libpico.tar.bz2": "56870e42ce411f4e596b5859d373ae07e86dcca1c308c1226ec5ecd694d6ab56",
}


def call(*args):
    subprocess.run(list(map(str, args)), check=True)


def git(source, *args, env=None, input=None):
    return subprocess.run(["git", "-C", str(source), *map(str, args)], check=True, capture_output=True, text=True,
                          env=env, input=input).stdout.strip()


def series_commit(source, patches):
    """The series as commits on COMMIT, one per patch, made without touching the working tree. Author, date
    and message come from each patch, so the same series always gives the same commits."""
    parent = COMMIT
    with tempfile.TemporaryDirectory() as scratch:
        message, diff = Path(scratch) / "message", Path(scratch) / "diff"
        env = {**os.environ, "GIT_INDEX_FILE": str(Path(scratch) / "index")}
        git(source, "read-tree", COMMIT, env=env)
        for patch in patches:
            header = git(source, "mailinfo", message, diff, input=patch.read_text())
            header = dict(line.split(": ", 1) for line in header.splitlines())
            if not {"Author", "Email", "Date", "Subject"} <= header.keys():
                raise RuntimeError(f"{patch.name} is not a commit from git format-patch")
            for role in ("AUTHOR", "COMMITTER"):
                env |= {f"GIT_{role}_NAME": header["Author"], f"GIT_{role}_EMAIL": header["Email"],
                        f"GIT_{role}_DATE": header["Date"]}
            git(source, "apply", "--cached", diff, env=env)
            tree = git(source, "write-tree", env=env)
            text = header["Subject"] + "\n\n" + message.read_text()
            parent = git(source, "commit-tree", tree, "-p", parent, env=env, input=text)
    return parent


def link(path, target):
    if not path.exists() and not path.is_symlink():
        path.symlink_to(target, target_is_directory=target.is_dir())


def install(source, target):
    """target replaced by a copy of source in one step. ./ironlark build-renderer refuses while a flight runs, but a
    Webots started by hand, or left running by a killed launcher, holds no lock: it keeps the file it started, where
    copying over a running executable in place fails (Text file busy) after the whole build."""
    partial = target.with_name(target.name + ".part")
    shutil.copy2(source, partial)
    partial.replace(target)


def overlay_resources(runtime, installed, source, base=COMMIT):
    """Mirror stock resources as symlinks, serving files the series changed
    (shaders, node definitions) from the patched source."""
    changed = subprocess.check_output(
        # Working tree, not just the committed series: a patch in progress is live too.
        ["git", "-C", str(source), "diff", "--name-only", base, "--", "resources"],
        text=True,
    ).split()
    changed = {Path(name).relative_to("resources") for name in changed}
    parents = {parent for name in changed for parent in name.parents if parent != Path(".")}
    root = runtime / "resources"
    if root.is_symlink():
        root.unlink()
    if root.exists():
        shutil.rmtree(root)
    root.mkdir()

    def mirror(folder):
        for child in (installed / "resources" / folder).iterdir():
            name = folder / child.name
            if name in parents:
                (root / name).mkdir()
                mirror(name)
            elif name in changed:
                link(root / name, source / "resources" / name)
            else:
                link(root / name, child)

    mirror(Path("."))
    for name in changed:  # files the series adds
        if not (root / name).exists():
            (root / name).parent.mkdir(parents=True, exist_ok=True)
            link(root / name, source / "resources" / name)


def fetch_archive(url, archive, expected):
    """Never reuse interrupted or corrupt downloads; preserve reusable completed inputs."""
    if archive.is_file() and hashlib.sha256(archive.read_bytes()).hexdigest() == expected:
        return
    partial = archive.with_name(archive.name + ".part")
    urlretrieve(url, partial)
    if hashlib.sha256(partial.read_bytes()).hexdigest() != expected:
        raise RuntimeError(f"Unexpected dependency archive contents: {archive.name}")
    partial.replace(archive)


def build_renderer():
    source = CACHE / "webots-source"
    installed = SIM / "webots"
    if not source.exists():
        call(
            "git",
            "clone",
            "--depth",
            "1",
            "--branch",
            RELEASE,
            "--filter=blob:none",
            "--sparse",
            "https://github.com/cyberbotics/webots.git",
            source,
        )
        call(
            "git",
            "-C",
            source,
            "sparse-checkout",
            "set",
            "src",
            "include",
            "resources",
            "scripts",
            "dependencies",
        )
    if subprocess.run(["git", "-C", str(source), "cat-file", "-e", COMMIT + "^{commit}"]).returncode:
        raise RuntimeError(f"Unexpected Webots checkout: no {COMMIT}")
    call(
        "git",
        "-C",
        source,
        "submodule",
        "update",
        "--init",
        "--depth",
        "1",
        "src/glm",
        "src/stb",
    )
    # The series is branch BRANCH of the checkout: upstream COMMIT plus one commit per patch, so git's own
    # tools (log, blame, rebase) work on it and tools/save_webots_patch.py writes it back as patches.
    # Commits there carry Ironlark's identity, and `git status` shows only edits: the builder's files and
    # links are ignored, and the tracked files under the `lib` link (outside the sparse checkout) stay
    # skipped rather than showing as deleted, which would also stop a rebase.
    for key in ("user.name", "user.email"):
        if value := subprocess.run(["git", "-C", str(SIM), "config", key], capture_output=True, text=True).stdout.strip():
            git(source, "config", key, value)
    exclude = source / ".git/info/exclude"
    names = "".join(f"/{name}\n" for name in (".ironlark-patches", "bin/qt", "lib",
                                                *(f"src/wren/{path.name}" for path in NATIVE_SOURCES)))
    if names not in (excluded := exclude.read_text() if exclude.exists() else ""):
        exclude.write_text(excluded + names)
    git(source, "config", "sparse.expectFilesOutsideOfPatterns", "true")
    git(source, "update-index", "-z", "--skip-worktree", "--stdin", input=git(source, "ls-files", "-z", "--", "lib"))
    patches = sorted(PATCHES.glob("*.patch"))
    tip = series_commit(source, patches)
    head = git(source, "rev-parse", "HEAD")
    stamp = source / ".ironlark-patches"  # the series commit last checked out
    if head != tip:
        # Moving the branch must not drop commits not yet saved as patches.
        saved = head in (COMMIT, stamp.read_text().strip() if stamp.exists() else "")
        if not saved and git(source, "rev-parse", head + "^{tree}") != git(source, "rev-parse", tip + "^{tree}"):
            raise RuntimeError(
                "The Webots branch has commits not saved to native/patches; run tools/save_webots_patch.py first."
            )
        # Rewrites only the files that differ, so only they recompile. Uncommitted edits carry over: git
        # refuses to overwrite any.
        call("git", "-C", source, "checkout", "-q", "-B", BRANCH, tip)
    stamp.write_text(tip + "\n")
    for path in NATIVE_SOURCES:  # copied only when changed, so make leaves the rest alone
        if not (copy := source / "src/wren" / path.name).exists() or copy.read_bytes() != path.read_bytes():
            shutil.copyfile(path, copy)
    for name, expected in PACKAGES.items():
        archive = source / "dependencies" / name
        fetch_archive(
                "https://cyberbotics.com/files/repository/dependencies/linux64/release/"
                + name,
                archive, expected)
        with tarfile.open(archive) as tar:
            tar.extractall(
                source,
                members=[m for m in tar if m.name.startswith("include/")],
                filter="data",
            )
    # The runtime package omits three header modules. Obtain the matching Qt
    # headers without replacing its working Qt libraries or the system Qt.
    qt = CACHE / "Qt/6.5.3/gcc_64/include"
    if not all((qt / name).exists() for name in ("QtOpenGLWidgets/QOpenGLWidget", "QtQml/QtQml", "QtXml/QtXml")):
        call(
            "uvx",
            "--from",
            "aqtinstall==3.3.0",
            "aqt",
            "install-qt",
            "linux",
            "desktop",
            "6.5.3",
            "gcc_64",
            "--outputdir",
            CACHE / "Qt",
            "--archives",
            "qtbase",
            "qtdeclarative",
        )
    include = source / "include/qt"
    include.mkdir(exist_ok=True)
    for module in (installed / "include/qt").iterdir():
        link(include / module.name, module)
    for name in ("QtOpenGLWidgets", "QtQml", "QtXml"):
        (include / name).mkdir(exist_ok=True)
        link(include / name / name, qt / name)
    (source / "bin").mkdir(exist_ok=True)
    link(source / "bin/qt", installed / "bin/qt")
    link(source / "lib", installed / "lib")
    link(source / "dependencies/libOIS.so", installed / "lib/webots/libOIS-1.4.0.so")
    logs = CACHE / "build-logs"
    # Rebasing the branch rewrites the files of every later patch, and make goes by modification time.
    # ccache, when installed, recompiles only what actually changed.
    compilers = []
    if shutil.which("ccache"):
        os.environ.setdefault("CCACHE_DIR", str(CACHE / "ccache"))
        os.environ.setdefault("CCACHE_MAXSIZE", "1G")
        compilers = ["CC=ccache gcc", "CXX=ccache g++"]
    identity = renderer_identity(SIM)
    for name in ("glad", "wren", "webots"):
        print(f"Compiling {name} (two jobs)", flush=True)
        run_stage(
            [
                "make",
                "-C",
                str(source / "src" / name),
                "-j2",
                "release",
                "WEBOTS_HOME=" + str(source),
                "LD_FLAGS=-rdynamic -L" + str(source / "dependencies"),
                *compilers,
            ],
            logs / f"{name}-build.log",
        )
    # A separate installation keeps the stock binary available for comparison.
    runtime = CACHE / "webots-renderer"
    runtime.mkdir(exist_ok=True)
    for path in installed.iterdir():
        if path.name not in ("bin", "webots", "resources"):
            link(runtime / path.name, path)
    overlay_resources(runtime, installed, source)
    (runtime / "bin").mkdir(exist_ok=True)
    for path in (installed / "bin").iterdir():
        if path.name != "webots-bin":
            link(runtime / "bin" / path.name, path)
    install(source / "bin/webots-bin", runtime / "bin/webots-bin")
    install(installed / "webots", runtime / "webots")
    manifest = {**identity, "series_commit": tip}
    (runtime / "ironlark-renderer.json").write_text(json.dumps(manifest, indent=2))
    print(f"Patched Webots ready: {CACHE / 'webots-renderer'}", flush=True)


def main():
    require_renderer_prerequisites()
    build_renderer()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
    except KeyboardInterrupt:
        raise SystemExit("Renderer build interrupted; see .cache/build-logs/ and rerun ./ironlark build-renderer.")
