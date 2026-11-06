# SPDX-License-Identifier: MIT
"""Independent CPU replay of observed contacts, including adversarial evidence."""
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from square_cut_line.contact import config_dict, source_manifest
from square_cut_line.contact import DEFAULT_CONFIG, lift_gate, rotation_xyzw
from square_cut_line.regrade import main, reconstruct, regrade_trial
from square_cut_line.scoring import score_strokes


def samples(*, mode="nib", correct_guide=True, quiet_steps=120):
    result = []
    board = [.29, 0, .7555, 0, 0, 0, 1]
    square = [.208, .09, .7655, 0, 0, 0, 1] if correct_guide else [.18, .27, .7655, 0, 0, 0, 1]
    for x in np.linspace(.355, .225, 27):
        z = .8155 if mode == "nib" else (.84 if mode == "hover" else .7635)
        result.append(dict(tick=len(result)+1, dt=1/240,
                           marker_pose=[float(x), -.03, z, 0, 0, 0, 1],
                           board_pose=board, square_pose=square,
                           pair_force=[0, 0, .03 if mode != "hover" else 0],
                           pair_point=[float(x), -.03, .7635], guide_pair_force=[0, 0, 0]))
    for _ in range(quiet_steps):
        result.append(dict(tick=len(result)+1, dt=1/240,
                           marker_pose=[.225, -.03, .84, 0, 0, 0, 1],
                           board_pose=board, square_pose=square, pair_force=[0, 0, 0],
                           pair_point=[None, None, None], guide_pair_force=[0, 0, 0]))
    return result


def export(folder, records):
    graded = reconstruct(records)
    folder.mkdir(parents=True, exist_ok=True)
    (folder/"contacts.jsonl").write_text("".join(json.dumps(record)+"\n" for record in records), encoding="utf-8")
    manifest = source_manifest()
    ink = {name: graded[name] for name in ("strokes", "guided_strokes", "target_start", "target_end")}
    ink.update(contact_config=config_dict(), environment_revision=manifest["revision"])
    (folder/"ink.json").write_text(json.dumps(ink), encoding="utf-8")
    (folder/"metrics.json").write_text(json.dumps(graded["metrics"]), encoding="utf-8")
    (folder/"source_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return graded


def test_real_contact_flow_full_score_after_lift():
    result = reconstruct(samples())
    assert result["metrics"]["score"] == 1
    assert result["metrics"]["success"]
    assert result["metrics"]["target_length"] == pytest.approx(.14)
    assert result["metrics"]["coverage"] >= .9
    assert result["metrics"]["nib_contact_samples"] == 27
    assert result["metrics"]["ink_quiet_s"] == pytest.approx(.5)
    assert len(result["strokes"]) == 1


@pytest.mark.parametrize("mode", ["hover", "shaft"])
def test_no_ink_from_hover_or_barrel_even_if_export_claims_contact(mode):
    records = samples(mode=mode)
    for record in records:
        record["ink"] = True
    result = reconstruct(records)
    assert result["metrics"]["score"] == 0
    assert result["strokes"] == []
    assert result["differences"]


def test_wrong_guide_keeps_observed_ink_but_awards_zero():
    result = reconstruct(samples(correct_guide=False))
    assert result["metrics"]["geometry_score"] == 1
    assert result["metrics"]["score"] == 0
    assert result["metrics"]["guide_used_fraction"] == 0
    assert result["strokes"]


@pytest.mark.parametrize("quiet", [0, 119])
def test_lift_or_quiet_period_missing_caps_candidate_full_score(quiet):
    result = reconstruct(samples(quiet_steps=quiet))
    assert result["metrics"]["geometry_score"] == 1
    assert result["metrics"]["score"] == .5
    assert not result["metrics"]["success"]


def test_missing_physics_sample_is_not_bridged():
    records = samples()
    del records[4]
    with pytest.raises(ValueError, match="consecutive"):
        reconstruct(records)


def test_contact_loss_breaks_stroke():
    records = samples()
    records[13]["pair_force"] = [0, 0, 0]
    result = reconstruct(records)
    assert len(result["strokes"]) == 2
    assert result["metrics"]["score"] == 0  # 10 mm unmarked gap


def test_file_regrade_and_cli_compare_actual_evidence(tmp_path, capsys):
    export(tmp_path, samples())
    assert regrade_trial(tmp_path)["match"]
    assert main([str(tmp_path)]) == 0
    assert json.loads(capsys.readouterr().out)["match"]
    metrics_path = tmp_path/"metrics.json"
    metrics = json.loads(metrics_path.read_text())
    metrics["score"] = 0
    metrics_path.write_text(json.dumps(metrics))
    assert not regrade_trial(tmp_path)["match"]
    assert main([str(tmp_path)]) == 1


def test_tampered_exported_ink_does_not_change_reconstructed_score(tmp_path):
    export(tmp_path, samples())
    path = tmp_path/"ink.json"
    ink = json.loads(path.read_text())
    ink["strokes"] = []
    path.write_text(json.dumps(ink))
    result = regrade_trial(tmp_path)
    assert result["metrics"]["score"] == 1
    assert not result["match"]
    assert regrade_trial(tmp_path, check_ink=False)["match"]


def test_source_revision_covers_code_and_assets_but_not_reports(tmp_path):
    (tmp_path/"assets").mkdir()
    (tmp_path/"runtime.py").write_text("first version\n")
    (tmp_path/"assets"/"marker.usda").write_text("#usda 1.0\n")
    first = source_manifest(tmp_path)
    (tmp_path/"README.md").write_text("recorded result changed\n")
    assert source_manifest(tmp_path) == first
    (tmp_path/"assets"/"marker.usda").write_text("#usda 1.0\n# changed geometry\n")
    assert source_manifest(tmp_path)["revision"] != first["revision"]


def test_regrade_cli_reports_incomplete_trial(tmp_path, capsys):
    assert main([str(tmp_path)]) == 2
    assert not json.loads(capsys.readouterr().out)["match"]


@pytest.mark.parametrize("correct_guide,quiet", [(True, 120), (False, 120), (True, 0)])
def test_final_regrade_matches_exact_runtime_metrics_without_isaac(correct_guide, quiet):
    records = samples(correct_guide=correct_guide, quiet_steps=quiet)
    rebuilt = reconstruct(records)
    source = Path(__file__).parents[1]/"square_cut_line"/"runtime.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    cls.bases = []
    cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name in {"metrics", "_target_local"}]
    namespace = dict(np=np, spec=DEFAULT_CONFIG, score_strokes=score_strokes,
                     lift_gate=lift_gate, rotation_xyzw=rotation_xyzw)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), "exec"), namespace)
    runtime = namespace[cls.name]()
    runtime._base = SimpleNamespace(physics_dt=1/240)
    runtime._root_pose = lambda name: np.asarray(records[-1][name+"_pose"])
    runtime._score_cache, runtime._score_tick, runtime._score_urgent = None, -1, False
    runtime._physics_tick = len(records)
    runtime._last_ink_tick = 27
    runtime._lift_hold_ticks = quiet
    runtime._strokes, runtime._guided_strokes = rebuilt["strokes"], rebuilt["guided_strokes"]
    runtime._samples = records
    runtime._nib_count = 27
    runtime._guide_count = 27 if correct_guide else 0
    assert runtime.metrics(force=True) == rebuilt["metrics"]


def test_low_hover_then_one_clear_frame_does_not_satisfy_lift_hold():
    records = samples(quiet_steps=240)
    # Spend 0.5 s clear of ink but only 1 mm above the board, then lift.
    for record in records[27:147]:
        record["marker_pose"][2] = .8165
    one_frame = reconstruct(records[:148])["metrics"]
    assert one_frame["ink_quiet_s"] > .5
    assert one_frame["lift_clear_s"] == pytest.approx(1/240)
    assert one_frame["score"] == .5
    sustained = reconstruct(records)["metrics"]
    assert sustained["lift_clear_s"] == pytest.approx(.5)
    assert sustained["score"] == 1


def test_clearance_hold_resets_when_nib_returns_below_five_millimetres():
    records = samples(quiet_steps=240)
    records[27+180]["marker_pose"][2] = .8165
    result = reconstruct(records)["metrics"]
    assert result["lift_clear_s"] == pytest.approx(59/240)
    assert result["score"] == .5
