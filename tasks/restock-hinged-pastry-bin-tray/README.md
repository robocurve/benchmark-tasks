# Restock a Hinged Bakery Display Bin

Bimanual tabletop task for `yam_arena`: one YAM arm lifts and holds an unpowered gravity-closing rear-hinged clear-acrylic bakery bin lid while the opposite arm removes the empty stainless inner tray onto a counter mat and places a stocked pastry tray flat inside the bin, before the holding arm lowers the lid shut without slamming.

**Layout:** the starting layout is fixed. The default is side L; set `BIN_SIDE=R` to run the mirrored layout (used by the demo checks below).

## Files

- `task.py`: Scene geometry (`HingedBin`, stainless `empty_tray`, pastry-loaded `stocked_tray`, slate `tray_mat`), `score`/`success` logic with latched slam/spill hazards, and symmetric `demo()` (`BIN_SIDE=L|R`, plus `DEMO_NO_HOLD=1` and `DEMO_DROP_DOOR=1` negative controls).
- `binkit.py`: Fixed-base articulated `HingedBin` builder with an unpowered viscous `UsdPhysics.RevoluteJoint` (`stiffness=0.0`).
- `yamkit.py`, `guardkit.py`: Local prop mesh/collider, tilted waypoint, and gripper-contact helpers.
- `test_task.py`: Offline geometry, scorer/hazard, negative-control, and 20 Hz `ArmPlan` IK tests.
- `task.json`: Task specification and scoring rubric.

## Commands

Run offline unit tests (and 20 Hz `ArmPlan` IK verification when `yam_arena` is installed):

```bash
pytest -q -p no:cacheprovider tasks/restock-hinged-pastry-bin-tray/test_task.py
```

Run the scripted demonstration in Isaac Lab-Arena (`BIN_SIDE=L` or `BIN_SIDE=R`, seed `42`):

```bash
BIN_SIDE=L inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/restock-hinged-pastry-bin-tray/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the empty inner tray inside the hinged bakery bin with the stocked pastry tray, leaving the empty tray flat on the counter mat and the bin lid closed without letting it slam." \
  --scorer task_score --max-steps 520 --no-prompt --log-dir ~/yam-eval/restock-hinged-pastry-bin-tray
```

Run negative-control hazard checks (both score `0.0` due to latched lid-slam hazard):

```bash
BIN_SIDE=L DEMO_NO_HOLD=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/restock-hinged-pastry-bin-tray/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the empty inner tray inside the hinged bakery bin with the stocked pastry tray, leaving the empty tray flat on the counter mat and the bin lid closed without letting it slam." \
  --scorer task_score --max-steps 520 --no-prompt --log-dir ~/yam-eval/restock-hinged-pastry-bin-tray-no-hold
```

Runtime per trial: ~85–95 s on a single L40S GPU (including environment startup; ~450 control steps at 20 Hz, 22.5 s simulated episode).

## Real-world build

A cut sheet, parts list, hinge checks (lid mass 0.28 kg, free swing, the 240 fps slow-motion check of the slam rule) and the washable pastry stand-in specification are in [pastry_build/BUILD.md](pastry_build/BUILD.md); `pastry_build/make_cutsheet.py` regenerates `cutsheet.svg`. The simulation, scorer and scripted demo are unchanged.
