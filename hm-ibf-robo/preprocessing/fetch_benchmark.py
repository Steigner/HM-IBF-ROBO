"""Fetch the upstream Robotics-Benchmarking repository holding `Pos_pnts.mat`.

`preprocessing.prepare_instances` needs the source point cloud published in
https://github.com/JakubKudela89/Robotics-Benchmarking. That repository is not part of
this codebase, so this module clones it on demand instead of requiring a manual checkout.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

DEFAULT_REPO_URL = "https://github.com/JakubKudela89/Robotics-Benchmarking.git"
"""Upstream repository holding the MATLAB point-selection reference and `Pos_pnts.mat`."""

REPO_SUBDIR = "EvoApps2023"
"""Subdirectory of the upstream repository holding the benchmark source files."""

POS_PNTS_FILENAME = "Pos_pnts.mat"
"""Name of the source point-cloud file inside `REPO_SUBDIR`."""


class RepositoryFetchError(RuntimeError):
    """Raised when the upstream repository cannot be cloned or has an unexpected layout."""


def ensure_benchmark_repo(dest: Path, repo_url: str = DEFAULT_REPO_URL) -> Path:
    """Ensures a local clone of the upstream Robotics-Benchmarking repository.

    If `dest / REPO_SUBDIR / POS_PNTS_FILENAME` already exists, the checkout is reused
    as-is and never updated; remove `dest` manually to pick up upstream changes. Otherwise
    the repository is shallow-cloned into `dest`, which must not already exist.

    Args:
        dest: Directory that should hold the checkout.
        repo_url: URL of the upstream repository.

    Returns:
        Path of the source point-cloud file, `dest / REPO_SUBDIR / POS_PNTS_FILENAME`.

    Raises:
        RepositoryFetchError: If `dest` exists without a usable checkout, the `git`
            executable is unavailable, `git clone` fails, or the expected file is missing
            after a successful clone.
    """
    source = dest / REPO_SUBDIR / POS_PNTS_FILENAME
    if source.exists():
        return source

    if dest.exists() and any(dest.iterdir()):
        raise RepositoryFetchError(
            f"{dest} exists but has no {REPO_SUBDIR}/{POS_PNTS_FILENAME}; "
            "remove it and retry to re-clone"
        )

    dest.parent.mkdir(parents=True, exist_ok=True)
    try:
        subprocess.run(
            ["git", "clone", "--depth", "1", repo_url, str(dest)],
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as error:
        raise RepositoryFetchError(
            "git executable not found; install git or pass --source explicitly"
        ) from error
    except subprocess.CalledProcessError as error:
        raise RepositoryFetchError(
            f"failed to clone {repo_url} into {dest}: {error.stderr.strip()}"
        ) from error

    if not source.exists():
        raise RepositoryFetchError(
            f"cloned {repo_url} but {source} is missing; the upstream layout may have changed"
        )
    return source
