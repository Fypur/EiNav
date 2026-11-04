"""Isole les tests : ils ne lisent ni n'écrivent le settings.json du robot, et rétablissent les réglages."""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Iterator

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot import tuning  # noqa: E402


@pytest.fixture(autouse=True)
def isolated_settings(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Any]:
    monkeypatch.setattr(tuning, "SETTINGS_FILE", tmp_path / "settings.json")
    yield
    for setting in tuning.SETTINGS:
        setattr(setting.module, setting.key, tuning._DEFAULTS[setting.key])
