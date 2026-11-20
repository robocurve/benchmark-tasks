# Replace a BÄSTIS lint roller refill with both hands

Hold the lint roller by its handle. Remove the used roll, fit the fresh refill, and lay the reassembled roller back in its rest.

IKEA BÄSTIS 904.256.26 reusable polypropylene handle with 301.411.26 pull-off refill. One hand supports the free handle against insertion and extraction loads. The other grips the rolled paper, withdraws it along the spindle and presses the replacement to its shoulder.

## Setup

Assembled tool length about 219 mm with a 120 mm gray handle and 99 mm sleeve. Used OD 44 mm, new OD 48 mm, 33 mm bore and tapered 27.3 mm model spindle. Handle starts horizontal in a 349 mm rail rest with an end stop. Fresh roll is parallel on a second rest and the discard tray is on the other side.

## Scoring

0.25 for removal with both arms and a released spent refill in the tray. 0.50 when the fresh roll seats with the handle held by the opposite hand, ±4 mm axial, <3.5 mm radial and <7° alignment error. 0.75 when it stays fitted during a further ≥15 mm lift by the handle alone. Full score for the reassembled roller released in its rest for 1 s within 2 mm and 3° of the initial released pose.

## Reset

Restore the handle and spent roll in the common rest. Set the replacement on the other rest, clear the discard tray and reset trial history.

## Simulation fidelity

The reference assembly is 22 cm long with a pull-off refill. Bore, roll length, handle profile and retention geometry are estimates from product views. The rests keep the parts from rolling at reset, and the handle is a free rigid body during exchange.

## Real setup and evidence

Use the linked BÄSTIS kit, two padded V-rests and a shallow discard tray. Measure the actual sleeve and actual new-roll insertion depth with calipers. Frame both arms and the sleeve end with an overhead camera. Put a reference mark at the real seated shoulder.

[IKEA BÄSTIS matched handle and refills](https://www.ikea.com/us/en/p/baestis-lint-roller-with-4-refills-s39642510/)

## v7 changes

The BÄSTIS-style parts and rail support carry over from v6. The demo now trims pickup to the measured handle or roll pose and aligns the bore to the held handle axis while the sleeve is in the right hand. The actual scorer still requires the two hands, a physically seated sleeve, a retention lift and a released hold. Watch `roll_right`, `axial_mm`, `radial_mm` and `angle_deg` in the console.

The scripted exchange lifts the assembly off the rest and turns it 32° in the plane of the table. Both hands move apart along the spindle axis during the pull. The support wrist uses the same jaw closing axis with its housing turned toward its own side. The refill wrist approaches from the right lane, withdraws axially after fitting, and parks before the handle-only lift. The script reads simulated part positions for pickup and radial insertion alignment.

The full GUI recording reached 1.00 with the fresh sleeve seated during the handle-only lift and after release. The optional `FAILSAFE_ARM_CONTACTS=1` check samples PhysX link contacts each control step. It reported a maximum 0.52 µm overlap between the two link4 colliders during final pressing.

## Run the trials in YAM Arena

From this task folder, activate the existing Arena environment and load the instruction and limit from the task specification. Use a clean reset, seed 0 and zero object jitter. Both trials run at 20 Hz with the `task_score` scorer.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
```

### Scripted demo

The recorded demo used the graphical display and 640×352 frames. Set `-E headless=true` to run without a display.
The recorded demo enabled arm contact telemetry.
```bash
FAILSAFE_ARM_CONTACTS=1 inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E object_jitter=0 -E headless=false -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/demo
```

### Dream-1

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E object_jitter=0 -E headless=true -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/model_run
```

Read the final `task_score` under `metrics` in `inspect-robots inspect <saved-eval.json>`.

The scripted demo scored **1.0** in the saved evaluation `adhoc_e69ae65a.json`.

The recorded demo confirmed the handle-only retention lift and released parking. Contact telemetry recorded about 0.52 µm of overlap between opposing link-4 colliders at the final press.

Dream-1 scored **0.0** in the saved evaluation `adhoc_7af45461.json`.
