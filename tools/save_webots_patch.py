#!/usr/bin/env python3
"""Save the Webots branch to native/patches: one patch per commit.

    .venv/bin/python tools/save_webots_patch.py        # write the branch as the series
    .venv/bin/python tools/save_webots_patch.py fog    # fold uncommitted edits into the patch named *fog*, then write

The branch is `ironlark` in .cache/webots-source, which ./ironlark build-renderer makes from the series:
upstream plus one commit per patch. A new patch is a new commit there (message: what it changes, why,
what it measured); an edit to an existing patch is folded into its commit. The series is then the
branch's `git format-patch`, and the branch is reset to the commits the builder makes from it.
"""
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from build_webots_renderer import BRANCH, COMMIT, PATCHES, series_commit

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / ".cache/webots-source"
TREES = ("src", "include", "resources")


def git(*args):
    return subprocess.run(["git", "-C", str(SOURCE), *args], check=True, capture_output=True, text=True).stdout.strip()


def main():
    if git("rev-parse", "--abbrev-ref", "HEAD") != BRANCH or git("rev-parse", "HEAD") == COMMIT:
        raise SystemExit(f"No {BRANCH} branch with patches in {SOURCE}: run ./ironlark build-renderer first.")
    if len(sys.argv) > 1:
        commits = git("rev-list", "--reverse", f"{COMMIT}..HEAD").split()
        names = [f"{n:04d}-{git('log', '-1', '--format=%f', c)}" for n, c in enumerate(commits, 1)]
        matches = [c for c, name in zip(commits, names) if sys.argv[1].lower() in name.lower()]
        if len(matches) != 1:
            raise SystemExit(f"{len(matches)} patches match {sys.argv[1]!r}: " + ", ".join(names))
        git("add", "-A", "--", *TREES)
        git("commit", "-q", "--fixup", matches[0])
        rebase = subprocess.run(["git", "-C", str(SOURCE), "-c", "sequence.editor=true", "rebase", "-q", "-i",
                                 "--autosquash", COMMIT])
        if rebase.returncode:
            raise SystemExit(f"A later patch conflicts with the edit: resolve it in {SOURCE}, `git rebase "
                             "--continue`, then run this again without a name.")
    if git("status", "--porcelain", "--", *TREES):
        raise SystemExit("Uncommitted Webots edits: commit them as a new patch, or name the patch to fold them into.")

    with tempfile.TemporaryDirectory() as scratch:
        git("format-patch", "-q", "--zero-commit", "--no-signature", "--no-numbered", "--binary", "--full-index", "--no-renames",
            "--default-prefix", "--diff-algorithm=myers", "-o", scratch, f"{COMMIT}..HEAD")
        for old in PATCHES.glob("*.patch"):
            old.unlink()
        for new in sorted(Path(scratch).glob("*.patch")):
            shutil.copyfile(new, PATCHES / new.name)
    patches = sorted(PATCHES.glob("*.patch"))
    tip = series_commit(SOURCE, patches)
    if git("rev-parse", tip + "^{tree}") != git("rev-parse", "HEAD^{tree}"):
        raise SystemExit("The written series does not rebuild the branch; native/patches is left as written.")
    git("reset", "-q", "--soft", tip)
    (SOURCE / ".ironlark-patches").write_text(tip + "\n")
    print(f"Saved {len(patches)} patches to {PATCHES.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
