"""Offline checks of the fabrication files (no OpenSCAD, no GPU): every simulation dimension in params.scad matches
task.py, the part files take every dimension from params.scad, and the design checks in check_params.py pass.

    python -m pytest fabrication/test_fabrication.py
"""

from __future__ import annotations

import pathlib
import shutil
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import check_params as C  # noqa: E402


def test_fabrication_files_match_task():
    assert C.main() == 0


def test_every_param_is_tagged_and_parsed():
    params, errors = C.parse_params(C.HERE / "params.scad")
    assert errors == []
    tags = {tag for _, _, tag, _, _ in params}
    assert tags == {"task", "fab"}


def test_checker_catches_a_changed_dimension():
    T = C.load_task()
    params, _ = C.parse_params(C.HERE / "params.scad")
    bad = [(n, "186" if n == "PSU_L" else v, tag, note, ln) for n, v, tag, note, ln in params]
    _, errors = C.check_task_values(T, bad)
    assert len(errors) == 1 and "PSU_L" in errors[0]


def test_checker_catches_a_stray_literal_and_unbalanced_brackets(tmp_path):
    shutil.copy(C.HERE / "params.scad", tmp_path / "params.scad")
    (tmp_path / "bad.scad").write_text('include <params.scad>\ncube([PSU_L, 7.3, PLUG[3]]);\ntranslate([0, 0, 1] cube(1);\n')
    _, errors = C.check_part_files(tmp_path)
    assert any("literal 7.3" in e for e in errors)
    assert not any("literal 3" in e for e in errors)      # PLUG[3] is an index, not a dimension
    assert any("unbalanced" in e or "unclosed" in e for e in errors)
