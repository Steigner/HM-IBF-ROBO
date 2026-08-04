"""Shared fixtures for the HM-IBF-ROBO Python test suite."""

from __future__ import annotations

from pathlib import Path

import pytest

CRATE_DIR = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def instances_dir() -> Path:
    """Returns the directory holding the instance JSON files shipped with the crate."""
    return CRATE_DIR / "instances"
