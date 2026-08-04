import json

import numpy as np
import pytest
from preprocessing.prepare_instances import (
    INSTANCE_SEEDS,
    POINT_COUNTS,
    SUMMARY_FILE,
    build_all_instances,
    build_instance,
    select_indices,
    write_instances,
)


@pytest.fixture
def point_cloud():
    return np.arange(300, dtype=float).reshape(100, 3)


def test_index_selection_is_deterministic_per_seed():
    assert select_indices(100, 4, 1) == select_indices(100, 4, 1)
    assert select_indices(100, 4, 1) != select_indices(100, 4, 2)


def test_selected_indices_stay_inside_the_pool():
    for seed in INSTANCE_SEEDS:
        for nr_points in POINT_COUNTS:
            indices = select_indices(100, nr_points, seed)
            assert len(indices) == nr_points
            assert all(0 <= index < 100 for index in indices)


def test_index_selection_matches_the_matlab_reference():
    # `main.m` uses `floor(rand(1, n) * pool)` under a Twister seeded by the instance seed.
    expected = np.floor(np.random.RandomState(3).rand(4) * 100).astype(int).tolist()

    assert select_indices(100, 4, 3) == expected


@pytest.mark.parametrize(("pool", "nr_points"), [(0, 3), (-1, 3), (100, 0), (100, -2)])
def test_index_selection_rejects_degenerate_arguments(pool, nr_points):
    with pytest.raises(ValueError):
        select_indices(pool, nr_points, 1)


def test_an_instance_carries_the_selected_points(point_cloud):
    instance = build_instance(point_cloud, 3, 7)

    assert instance["name"] == "3_pnts_inst07"
    assert instance["nr_points"] == 3
    assert instance["source_seed"] == 7
    assert len(instance["source_indices"]) == 3
    assert instance["points"] == point_cloud[instance["source_indices"]].tolist()


def test_the_full_set_covers_every_point_count_and_seed(point_cloud):
    instances = build_all_instances(point_cloud)

    assert len(instances) == len(POINT_COUNTS) * len(INSTANCE_SEEDS)
    assert {instance["nr_points"] for instance in instances} == set(POINT_COUNTS)
    assert {instance["source_seed"] for instance in instances} == set(INSTANCE_SEEDS)
    assert len({instance["name"] for instance in instances}) == len(instances)


def test_generation_is_reproducible(point_cloud):
    assert build_all_instances(point_cloud) == build_all_instances(point_cloud)


def test_writing_produces_one_file_per_instance_plus_a_summary(point_cloud, tmp_path):
    instances = build_all_instances(point_cloud)

    written = write_instances(instances, tmp_path / "instances")

    assert len(written) == len(instances)
    summary = json.loads((tmp_path / "instances" / SUMMARY_FILE).read_text(encoding="utf-8"))
    assert summary == instances

    first = json.loads(written[0].read_text(encoding="utf-8"))
    assert first == instances[0]


def test_the_checked_in_instances_follow_the_same_rng_stream(instances_dir):
    # `Pos_pnts.mat` is not part of the repository, so the shipped instances cannot be
    # regenerated here. What can be checked is the property the selection rule implies:
    # all instances of one seed draw from the same stream, so the indices of a smaller
    # instance are a prefix of the indices of every larger one.
    for seed in INSTANCE_SEEDS:
        previous: list[int] = []
        for nr_points in POINT_COUNTS:
            path = instances_dir / f"{nr_points}_pnts_inst{seed:02d}.json"
            instance = json.loads(path.read_text(encoding="utf-8"))

            indices = instance["source_indices"]
            assert len(indices) == nr_points == instance["nr_points"], path.name
            assert len(instance["points"]) == nr_points, path.name
            assert indices[: len(previous)] == previous, path.name
            previous = indices


def test_the_checked_in_instances_use_the_expected_names(instances_dir):
    expected = {
        f"{nr_points}_pnts_inst{seed:02d}" for nr_points in POINT_COUNTS for seed in INSTANCE_SEEDS
    }
    found = {path.stem for path in instances_dir.glob("*_pnts_inst*.json")}

    assert found == expected
