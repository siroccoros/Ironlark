"""Blender, pinned by checksum: downloaded once into .cache/blender/ and extracted beside it. The drone is modelled in
it (tools/build_drone.py, tools/drone_meshes.py)."""
import json
import os
from pathlib import Path
import shutil
import stat
import tarfile
from urllib.request import urlretrieve

from receipts import digest

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / ".cache"
INPUTS = {
    "blender/blender.tar.xz": (
        "https://download.blender.org/release/Blender4.2/blender-4.2.9-linux-x64.tar.xz",
        "dfbc127a7d28f9c2175b23bf9d6701b2855f31eedfb391f9a6e60adb24572846",
    ),
}


def fetch(name):
    """Download a pinned build input once and verify it before use."""
    url, expected = INPUTS[name]
    path = CACHE / name
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f"Downloading {url}", flush=True)
        partial = path.with_suffix(path.suffix + ".part")
        urlretrieve(url, partial)
        partial.rename(path)
    if digest(path) != expected:
        raise RuntimeError(f"Checksum mismatch for {path}; delete it and retry.")
    return path


def freeze(folder, frozen=True):
    """Every file and folder in folder read-only (frozen) or writable again. Nothing can edit a frozen folder, nor save
    a file into it: Blender writes a save beside the file and renames it over."""
    for path in [folder, *folder.rglob("*")]:
        if not path.is_symlink():
            mode = path.stat().st_mode
            path.chmod(mode & ~0o222 if frozen else mode | stat.S_IWUSR)


def unpack(archive, target, opener, identity, member=None, added_ok=False):
    """Extract once per archive (identity: its sha256), and again when a file it extracted was changed or removed, or a
    file was added. The marker records each file's size and mtime. added_ok lets files added since stand (Blender writes
    __pycache__ into its own folder); a partial extraction never looks complete. Without added_ok the extraction is left
    read-only, and a writable one is extracted again: only a read-only one is trusted by its marker."""
    marker = target / ".archive-sha256"

    def state(path):
        try:
            found = path.stat()
        except FileNotFoundError:
            return None
        return [found.st_size, found.st_mtime_ns]

    if marker.is_file() and marker.read_text().startswith("{"):  # an older marker, the sha256 alone: extract again
        recorded = json.loads(marker.read_text())
        added = not added_ok and any(str(path.relative_to(target)) not in recorded["files"]
                                     for path in target.rglob("*") if path.is_file() and path != marker)
        if recorded["archive"] == identity and not added and (added_ok or not os.access(target, os.W_OK)) and all(
                state(target / name) == kept for name, kept in recorded["files"].items()):
            if not added_ok:
                freeze(target)
            return
    if target.is_dir():
        freeze(target, False)
    shutil.rmtree(target, ignore_errors=True)
    partial = target.with_name(target.name + ".part")
    shutil.rmtree(partial, ignore_errors=True)
    with opener(archive) as source:
        source.extractall(partial)
    (partial / member if member else partial).rename(target)
    shutil.rmtree(partial, ignore_errors=True)
    files = {str(path.relative_to(target)): state(path) for path in sorted(target.rglob("*")) if path.is_file()}
    marker.write_text(json.dumps({"archive": identity, "files": files}) + "\n")
    if not added_ok:
        freeze(target)


BLENDER = CACHE / "blender/blender-4.2.9-linux-x64/blender"


def blender_executable():
    name = "blender/blender.tar.xz"
    unpack(fetch(name), BLENDER.parent, tarfile.open, INPUTS[name][1], member=BLENDER.parent.name, added_ok=True)
    return BLENDER
