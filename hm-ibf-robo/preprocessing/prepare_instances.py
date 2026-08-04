"""Generate deterministic robotics benchmark instances from `Pos_pnts.mat`.

This mirrors the point selection of `EvoApps2023/main.m` in the upstream
Robotics-Benchmarking repository: for each `nr_points in {3, 4, 5, 6}` and each instance
seed `i in {1..10}`, `nr_points` reachable Cartesian points are sampled with a Mersenne
Twister seeded by `i`.

The generated JSON instances are consumed by the `hm-ibf-robo` binary. They are checked
into the repository, so this script is only needed to regenerate them. The source point
cloud is not part of this repository either; unless `--source` is given, it is fetched
automatically (see `preprocessing.fetch_benchmark`) from
https://github.com/JakubKudela89/Robotics-Benchmarking into `--repo-dir`.

Run from `hm-ibf-robo/`:

    python3 -m preprocessing.prepare_instances
    python3 -m preprocessing.prepare_instances --source ../robo-evo-apps/Pos_pnts.mat
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

from .fetch_benchmark import ensure_benchmark_repo

POINT_COUNTS = (3, 4, 5, 6)
"""Target-point counts generated for the benchmark."""

INSTANCE_SEEDS = tuple(range(1, 11))
"""Instance seeds generated per target-point count."""

SUMMARY_FILE = "summary.json"
"""Name of the aggregated summary written next to the instances."""


def select_indices(pool_size: int, nr_points: int, instance_seed: int) -> list[int]:
    """Selects the point indices of one instance.

    Reproduces `floor(rand(1, nr_points) * pool_size)` under a Twister seeded by
    `instance_seed`, matching `main.m`.

    Args:
        pool_size: Number of rows in the source point cloud.
        nr_points: Number of points to select.
        instance_seed: Seed of the Mersenne Twister.

    Returns:
        The selected row indices.

    Raises:
        ValueError: If `pool_size` or `nr_points` is not positive.
    """
    if pool_size <= 0:
        raise ValueError(f"the point pool must not be empty, got {pool_size}")
    if nr_points <= 0:
        raise ValueError(f"nr_points must be positive, got {nr_points}")

    rng = np.random.RandomState(instance_seed)
    return np.floor(rng.rand(nr_points) * pool_size).astype(int).tolist()


def build_instance(
    pos_points: np.ndarray,
    nr_points: int,
    instance_seed: int,
) -> dict[str, Any]:
    """Builds one instance descriptor.

    Args:
        pos_points: The source point cloud, shaped `(n, 3)`.
        nr_points: Number of points to select.
        instance_seed: Seed of the Mersenne Twister.

    Returns:
        The instance descriptor, matching the Rust `RoboInstance` schema.

    Raises:
        ValueError: If the point cloud or the requested point count is unusable.
    """
    indices = select_indices(pos_points.shape[0], nr_points, instance_seed)
    return {
        "name": f"{nr_points}_pnts_inst{instance_seed:02d}",
        "nr_points": nr_points,
        "source_seed": instance_seed,
        "source_indices": indices,
        "points": pos_points[indices].tolist(),
    }


def build_all_instances(pos_points: np.ndarray) -> list[dict[str, Any]]:
    """Builds every benchmark instance.

    Args:
        pos_points: The source point cloud, shaped `(n, 3)`.

    Returns:
        One descriptor per `(nr_points, instance_seed)` pair, in generation order.

    Raises:
        ValueError: If the point cloud is unusable.
    """
    return [
        build_instance(pos_points, nr_points, instance_seed)
        for nr_points in POINT_COUNTS
        for instance_seed in INSTANCE_SEEDS
    ]


def load_point_cloud(source: Path) -> np.ndarray:
    """Loads the source point cloud from a MATLAB `.mat` file.

    Args:
        source: Path of the `.mat` file holding a `Pos_pnts` variable.

    Returns:
        The point cloud as a float array shaped `(n, 3)`.

    Raises:
        FileNotFoundError: If `source` does not exist.
        KeyError: If the file has no `Pos_pnts` variable.
        ValueError: If `Pos_pnts` is not an `(n, 3)` array.
    """
    from scipy.io import loadmat

    if not source.exists():
        raise FileNotFoundError(f"source point cloud not found: {source}")

    pos_points = np.asarray(loadmat(source)["Pos_pnts"], dtype=float)
    if pos_points.ndim != 2 or pos_points.shape[1] != 3:
        raise ValueError(f"expected Pos_pnts to be (n, 3), got {pos_points.shape}")
    return pos_points


def write_instances(instances: list[dict[str, Any]], output_dir: Path) -> list[Path]:
    """Writes the instance descriptors and their aggregated summary.

    Args:
        instances: The descriptors to write.
        output_dir: Destination directory, created if missing.

    Returns:
        The paths of the written instance files, excluding the summary.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    written = []
    for instance in instances:
        path = output_dir / f"{instance['name']}.json"
        path.write_text(json.dumps(instance, indent=2), encoding="utf-8")
        written.append(path)

    (output_dir / SUMMARY_FILE).write_text(json.dumps(instances, indent=2), encoding="utf-8")
    return written


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parses the command line.

    Args:
        argv: Arguments to parse; defaults to `sys.argv[1:]`.

    Returns:
        The parsed namespace.
    """
    crate_dir = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser(
        description="Regenerate the HM-IBF-ROBO benchmark instances from Pos_pnts.mat."
    )
    parser.add_argument(
        "--source",
        type=Path,
        default=None,
        help=(
            "MATLAB file holding the Pos_pnts point cloud. If omitted, it is cloned "
            "automatically into --repo-dir."
        ),
    )
    parser.add_argument(
        "--repo-dir",
        type=Path,
        default=crate_dir / "external" / "robotics-benchmark",
        help="Directory receiving the auto-cloned Robotics-Benchmarking checkout.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=crate_dir / "instances",
        help="Directory receiving the generated instance JSON files.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Regenerates every benchmark instance.

    Args:
        argv: Command line arguments; defaults to `sys.argv[1:]`.

    Returns:
        `0` on success.

    Raises:
        RepositoryFetchError: If `--source` is omitted and the upstream repository cannot
            be cloned.
        FileNotFoundError: If the source point cloud is missing.
        KeyError: If the source file has no `Pos_pnts` variable.
        ValueError: If the point cloud has an unexpected shape.
    """
    args = parse_args(argv)

    source = args.source if args.source is not None else ensure_benchmark_repo(args.repo_dir)
    instances = build_all_instances(load_point_cloud(source))
    written = write_instances(instances, args.output_dir)

    print(f"Prepared {len(written)} robotics instances in {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
