import subprocess
from pathlib import Path

import pytest
from preprocessing.fetch_benchmark import (
    POS_PNTS_FILENAME,
    REPO_SUBDIR,
    RepositoryFetchError,
    ensure_benchmark_repo,
)


def test_an_existing_checkout_is_reused_without_cloning(tmp_path, monkeypatch):
    dest = tmp_path / "robotics-benchmark"
    source = dest / REPO_SUBDIR / POS_PNTS_FILENAME
    source.parent.mkdir(parents=True)
    source.write_bytes(b"fake-mat-contents")

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("git should not be invoked for an existing checkout")

    monkeypatch.setattr(subprocess, "run", fail_if_called)

    assert ensure_benchmark_repo(dest) == source


def test_a_missing_checkout_is_cloned(tmp_path, monkeypatch):
    dest = tmp_path / "robotics-benchmark"
    calls = []

    def fake_run(cmd, **_kwargs):
        calls.append(cmd)
        clone_dest = Path(cmd[-1])
        source = clone_dest / REPO_SUBDIR / POS_PNTS_FILENAME
        source.parent.mkdir(parents=True)
        source.write_bytes(b"fake-mat-contents")
        return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")

    monkeypatch.setattr(subprocess, "run", fake_run)

    result = ensure_benchmark_repo(dest, repo_url="https://example.invalid/repo.git")

    assert result == dest / REPO_SUBDIR / POS_PNTS_FILENAME
    assert result.exists()
    assert len(calls) == 1
    assert calls[0][:3] == ["git", "clone", "--depth"]
    assert calls[0][-2:] == ["https://example.invalid/repo.git", str(dest)]


def test_a_non_empty_destination_without_the_source_file_is_rejected(tmp_path, monkeypatch):
    dest = tmp_path / "robotics-benchmark"
    dest.mkdir()
    (dest / "stale-leftover.txt").write_text("stale", encoding="utf-8")

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("git should not be invoked for a non-empty destination")

    monkeypatch.setattr(subprocess, "run", fail_if_called)

    with pytest.raises(RepositoryFetchError, match="remove it and retry"):
        ensure_benchmark_repo(dest)


def test_a_missing_git_executable_is_reported_clearly(tmp_path, monkeypatch):
    dest = tmp_path / "robotics-benchmark"

    def missing_git(*_args, **_kwargs):
        raise FileNotFoundError("git")

    monkeypatch.setattr(subprocess, "run", missing_git)

    with pytest.raises(RepositoryFetchError, match="git executable not found"):
        ensure_benchmark_repo(dest)


def test_a_failed_clone_is_reported_clearly(tmp_path, monkeypatch):
    dest = tmp_path / "robotics-benchmark"

    def failing_clone(cmd, **_kwargs):
        raise subprocess.CalledProcessError(128, cmd, output="", stderr="fatal: not found")

    monkeypatch.setattr(subprocess, "run", failing_clone)

    with pytest.raises(RepositoryFetchError, match="fatal: not found"):
        ensure_benchmark_repo(dest)


def test_a_successful_clone_missing_the_expected_file_is_reported(tmp_path, monkeypatch):
    dest = tmp_path / "robotics-benchmark"

    def clone_without_the_expected_file(cmd, **_kwargs):
        clone_dest = Path(cmd[-1])
        clone_dest.mkdir(parents=True)
        return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")

    monkeypatch.setattr(subprocess, "run", clone_without_the_expected_file)

    with pytest.raises(RepositoryFetchError, match="upstream layout may have changed"):
        ensure_benchmark_repo(dest)
