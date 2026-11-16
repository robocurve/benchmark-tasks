"""Offline check that the printable parts' dimensions (fabrication/params.scad) match task.py.

    python -m pytest fabrication/test_fabrication.py
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import check_params  # noqa: E402


def test_params_match_task():
    T = check_params.load_task()
    entries = check_params.parse()
    env = check_params.values(entries)
    assert check_params.check_task_values(entries, env, T) >= 50
    check_params.check_fabrication(env, T)


def test_every_scad_file_uses_params():
    here = pathlib.Path(check_params.__file__).parent
    parts = ["carrier_frame.scad", "sprung_rail.scad", "guide_rod_mount.scad", "ssd_dummy.scad", "assembly.scad"]
    for name in parts:
        text = (here / name).read_text()
        assert "include <params.scad>" in text, name
        assert text.count("(") == text.count(")") and text.count("{") == text.count("}"), name
        assert text.count("[") == text.count("]"), name
