"""What a built artifact was built from, so a check never runs on one built from other code.

The drone's receipt (protos/drone/receipt.json, written last by tools/build_drone.py) records the sha256 of every input
and output and the versions of the Python packages that compute it. Standard library only: the launcher runs on the
system Python.
"""
import hashlib
from pathlib import Path

PACKAGES = ("numpy", "pillow")


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def packages(root, names=PACKAGES):
    """{name: version} as installed in the project's environment (.venv), None where missing."""
    installed = {}
    for info in Path(root, ".venv/lib").glob("python*/site-packages/*.dist-info"):
        name, _, version = info.name[: -len(".dist-info")].partition("-")
        installed[name.lower()] = version
    return {name: installed.get(name) for name in names}
