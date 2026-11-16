# Seat and level a portable surface-roughness drive on a steel shaft crown, then start a hands-off cut-off traverse

Pick up the portable Mitutoyo-style surface-roughness drive unit from the rubber staging pad and seat its front 90-degree V-saddle squarely along the top crown of the turned Ø42 mm steel shaft (`lateral <= 1.5 mm`, `yaw <= 2.5 deg`, `roll <= 5.0 deg`), with its rear leveling foot resting on the stainless rear support anvil. Turn the rear Ø26 mm knurled brass thumbscrew clockwise (`3.0 turns`, `2.5 mm/rev` pitch, overcoming `+7.5 mm` / `+3.58 deg` initial nose-up pitch) until the barrel sits horizontal (`<= 1.5 deg` pitch -> LCD transitions `OUT OF RANGE -> READY`), then press `START` on the desktop console and keep both arms open and clear (`>= 3.0 cm`) while the `1.6 s`, `4.8 mm` motorized cut-off traverse completes (`Ra = 0.82 um`).

## Files

- `qc_met8.py`: scene, scoring, physics step watch (`@on_step`), and open-loop bimanual `demo()`.
- `task.json`: title, category (`Impact on jobs`), BLS May 2023 context, instruction, setup, scoring summary, `run_limit`: `1,600 steps (80 seconds at 20 Hz)`.
- `SCENE.md`: Stage 3 real-scale bench specification (`station`, `TABLE_TOP = 0.7475 m`).
- `test_qc_met8.py`: offline unit tests (scoring tiers, shortcuts scoring 0, demo IK & joint-limit checks, `1280x704` top-camera projection and occlusion checks).
- `DEVIATIONS.md`: documented construction and simulation deviations.
- `labkit.py`, `arena_assets_fix.py`: shared primitive-part and joint builders.
- `assets/visual/`: render-only PBR models of every object (`*.usda` + textures); `assets_build/make_visuals.py` rebuilds them.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1 GPU evaluation
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/qc_met8.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir logs/dream

# Scripted full-score demo (open loop from fixed layout)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/qc_met8.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir logs/demo

for log in logs/dream/*.json logs/demo/*.json; do inspect-robots video "$log"; done
```

`SCORE_OVER = "end"`, `SUCCESS_HOLD_S = 1.0`. The scripted demo finishes in ~38 s (~760 steps at `20 Hz`, well inside `--max-steps 1600` and the `1,600`-step `run_limit`).

## Offline checks (no simulator required)

```bash
python -m pytest -q test_qc_met8.py
```

## Scoring

| Score | Condition | Verification |
|---|---|---|
| `0.0` | Not seated on shaft crown (`lateral > 1.5 mm`, `yaw > 2.5 deg`, `roll > 5.0 deg`, `pitch > 12.0 deg`, or foot off anvil), moving (`not resting()`), or `post_disturbed` (re-touched / re-pressed after completion) | Overhead + profile inspection |
| `0.25` | Seated on crown + anvil, `pitch > 2.8 deg` (out of range), grippers not open & clear | Profile angle check |
| `0.35` | Seated on crown + anvil, partial pitch `1.5–2.8 deg`, grippers not open & clear | Profile angle check |
| `0.40` | Seated on crown + anvil, `pitch > 2.8 deg`, grippers open & `>= 3.0 cm` clear | Profile + arm clearance |
| `0.50` | Seated on crown + anvil, partial pitch `1.5–2.8 deg`, grippers open & clear | Profile + arm clearance |
| `0.55` | Seated + leveled `<= 1.5 deg` (`READY`), no clean traverse yet, grippers not clear | Profile + READY LED |
| `0.70` | Seated + leveled `<= 1.5 deg`, no clean traverse (or `vibration_fault`), grippers open & clear | READY / FAULT LED |
| `0.85` | Seated + leveled + `1.6 s` traverse completed (`vibration_fault=False`), grippers not open & clear | Done bar + gripper distance |
| `1.0` | Seated (`<= 1.5 mm`, `<= 2.5 deg`, `<= 5.0 deg`) + foot on anvil + leveled (`<= 1.5 deg`) + clean `1.6 s` hands-off traverse + both grippers open & `>= 3.0 cm` clear (`1.0 s` hold) | Level barrel + Done LED (`Ra=0.82um`) + clear arms |

Documented tiers: `0, 0.25, 0.35, 0.4, 0.5, 0.55, 0.7, 0.85, 1.0`.

## Asset Provenance

Every visible object is drawn by a render-only USD model in `assets/visual/` (built by `assets_build/make_visuals.py`,
1:1 to the dimensions below); the task's boxes and cylinders are invisible colliders only (`labkit` `hide_shapes`), so
physics and scoring are unchanged. SimReady, the other HF libraries and the asset catalog hold no surface-roughness
tester, V-block, ground test shaft or instrument mat, so the geometry is authored
to the products' datasheet dimensions with real PBR materials.

| Asset / Reference | Source URL | Revision / Date | Licence | Notes |
|---|---|---|---|---|
| Mitutoyo Surftest SJ-210 drive (`115 x 23 x 26 mm`, `0.19 kg`) & console (`160 x 65.8 x 52.1 mm`), V-adapter `12AAE644`, V-blocks `181-903-10` | `https://www.mitutoyo.com/products/form-measurement/surface-roughness/surftest-sj-210/` | Manufacturer catalog accessed 2026-11-14 | Real-product dimensional reference | Geometry authored 1:1 (`assets_build/make_visuals.py`, MIT like `labkit.py`); thread pitch and anvil U-channel per `DEVIATIONS.md` |
| PBR tiles `t_metal_chrome_a01`, `t_rubber_glossy_black_a01` (from `syringe_a01`); `t_metal_glossy_a01/b01`, `t_plastic_glossy_black_a01`, `t_plastic_blue_a01` (from `tray_sterilization_instrument_c02`) | `https://huggingface.co/datasets/nvidia/simready-assets` | `7879e0e50423290300ac20ecd6e727325f680b60` | CC-BY-4.0 (NVIDIA; `assets/visual/textures/SIMREADY-LICENSE-CC-BY-4.0.txt`) | Downscaled 4096 -> 1024 px, tinted per material |
| Bench dressing (fixed, render-only, out of both arms' paths): `instrument_caliper_a01` (vernier caliper, sha256 `e8df8a420cdcfe4e...`), `box_glove_a01` (nitrile glove box, `b5661a03be7d4b74...`), `ppe_eyeProtection_safetyGlasses_a01` (`42e4069d2c4b386e...`), `consumable_spray_solvent_a01` (solvent spray, `aeaa86081e722356...`); sha256 of each source `sm_<name>_01.usd` | `https://huggingface.co/datasets/nvidia/simready-assets` | `7879e0e50423290300ac20ecd6e727325f680b60` | CC-BY-4.0 (NVIDIA; `assets/props/<name>/LICENSE.txt`) | Flattened to visual-only USD by `assets_build/flatten_props.py` (physics and MDL dropped, UsdPreviewSurface kept, textures downscaled to 512 px), used at the models' own scale |
| LCD screens, key caps and printed labels | Drawn by `assets_build/make_visuals.py` (DejaVu fonts) | 2026-11-14 | MIT | Generic text, no brand marks |
| BLS OEWS May 2023 Workforce Context (`51-9061`, `51-4041`) | `https://www.bls.gov/oes/2023/may/oes_nat.htm` | May 2023 release | Public Domain (US BLS) | Cited verbatim in `task.json` `context` |


## Motion-guard revision
Ra is preset illustration, not a measured surface-roughness result. It is not a precision-measurement objective. Completion requires finite observed start, an intermediate sample, and actual relative travel at least4.790mm after at least1.6s. The command remains4.8mm. Engineering endpoint tolerance10um rounds up the prior CPU tracking error9.786um; it is not an official rule or hardware calibration. No extra PD speed limit is imposed. Current-source MuJoCo demo1.0/756steps, shortcuts and7of7 perturbations passed. Independent current-source CPU review passed. Current-source official Isaac and Dream outcomes are reported below.


## Current official evidence (2026-11-16 UTC; supersedes historical results)
Frozen task source SHA256:6ca90a5a19e5a2a09cd1c3ae450a6d18eb22c257ea62fa850ccc8a5df0df28ea.
Isaac scripted demo at20Hz/320x176:task_score1.0,756steps,757video frames. Evaluation SHA25668f092deec444409b32f04c6490aaefa197d016c12eebdf2509b5e4c5229bc9f.
Dream-1:one attempt,176steps,trial ERROR due to server-side inference session expiration/replacement. No aggregate task score exists; this is not0.0 and not a completed evaluation. Four HTTP requests returned200; no retry. The incomplete model-run video is retained for transparency.
Source/physics/scorer unchanged between scripted and model runs. Only task.json scores and this evidence note were updated afterward. No claim of hardware calibration or real roughness acquisition.
