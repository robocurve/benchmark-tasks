# Wipe the used oil off a removed piston whose connecting rod swings

Task file `piston_clean.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A car-engine piston (75 mm, 1.6 L class) lies on a bench mat on its connecting rod, coated with used oil. The robot
cleans the outside of the piston with a parts sponge on a stick and a flat sponge, rinsing the stick sponge in a
solvent tray. The rod swings freely on the wrist pin and tips the assembly over when nobody holds it.

Files:

- `piston_clean.py`: the task file (objects, wrist-pin joint, piston and rod meshes, oil patches, `score()`, `demo()`).
- `piston_scoring.py`: the scoring core (cells, sponge model, rinse, zero rules), plain numpy.
- `piston_geometry.py`: piston and rod geometry with its sources.
- `test_piston_scoring.py`: tests of the scoring rules and exploits (`python -m pytest -q`, no simulator needed).
- `check_reach.py`, `plan_offline.py`: tuning helpers (IK reach and cell coverage of the scripted demo, no simulator).

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`, Isaac Lab 3.0 / Isaac Sim 6.1), `source $ARENA_DIR/yam.env`.
- Tested on an aarch64 NVIDIA GB10 with Arena's lock re-resolved for aarch64, torch 2.11.0+cu130 and
  `LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1`; on the supported x86_64 setup none of that is needed.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/piston-clean/piston_clean.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clean the oil off the outside of the piston." \
  --scorer task_score --max-steps 4800 --store-frames --log-dir ~/yam-eval/piston-dream
```

The scripted demo (full score): the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/piston-demo`.

Videos: `for log in ~/yam-eval/piston-*/*.json; do inspect-robots video "$log"; done`.

## Score

`--scorer task_score` reports the task file's `score()` at the end of the trial (`SCORE_OVER = "end"`). The rules are
in `task.json` and in `piston_scoring.py`; in short:

- **Cells:** 145 cells of about 1 cm on the outside of the piston: 100 on the skirt below the ring belt (the ring
  grooves are a separate tool job) and 45 on the crown, its dish included. The inside of the skirt (73 cells) and the
  rod beam (36 cells) are tracked and drawn but not scored.
- **Wiping:** a cell clears only after the sponge, pressed so that its 3 cm core touches the surface, has slid 3 cm
  across it. Holding the sponge against the piston, dabbing it or hovering it close clears nothing.
- **Sponge load:** the stick sponge holds 60 cells of oil; a full sponge smears oil back onto the cells it rubs.
  It unloads 1 cell per millimetre it moves while submerged in the solvent; dipping it without moving does not
  rinse it. The flat sponge is larger than the crown and needs no rinse.
- **Ladder:** 0.25 at 25% clean, 0.5 at 50%, 0.75 at 75%, 1.0 at 95% with the piston and its rod both lying still
  on the mat (0.75 if it is still held or moving). `score()` cannot see the grippers, so "released" is checked as
  the rod's lowest point being on the mat (within 1.5 cm): a rod still in a gripper hangs above it.
- **Zero:** the piston falls faster than 0.77 m/s (a free fall from 3 cm), the piston enters the solvent, or it ends
  off the mat.

**Scoring a real trial:** UV-tracer dye in the oil; photograph the skirt from four sides and the crown through a
printed 1 cm grid under a UV torch and count glowing cells; watch the video for drops and for the piston entering
the tray.

## Exploits considered

| Attempt | Result |
| --- | --- |
| Dunk the piston in the solvent | 0 for the episode (`test_dunking_in_tray_zeroes`) |
| Press the sponge on and hold it | nothing clears (`test_pressing_without_moving_cleans_nothing`) |
| Short dabs | nothing clears (`test_short_dab_is_not_enough`) |
| Hover the sponge just off the surface | nothing clears (`test_hovering_sponge_cleans_nothing`) |
| Never rinse | the sponge saturates and smears, at most 0.25 (`test_never_rinsing_stalls_and_smears`) |
| Dip without agitation | no rinse (`test_dip_without_agitation_does_not_rinse`) |
| Wipe through the skirt wall from outside to count the inside | not possible (`test_outside_wipe_does_not_clean_inside`) |
| Rub with the grippers or the handle | only the sponges clear cells |
| Clean while holding the piston up and stop | 0.75 until it rests on the mat (`test_still_in_gripper_caps_below_one`) |
| Drop the piston onto the mat | 0 (`test_drop_zeroes_whole_episode`) |

## Demo

Open loop, no simulator state is read: fixed TCP waypoints for both arms (`demo()` in `piston_clean.py`). Positions
were tuned offline against the fixed start layout (`object_jitter=0`). The right arm picks the toppled assembly up by
the rod and holds it in three wrist orientations; the left arm strokes the skirt with the stick sponge from 14
directions and rinses it with agitation in the tray six times; the right arm then scrubs the crown on the flat
sponge by twisting the wrist and sets the assembly back on the mat.

## Runtime and seeds

- Run limit: 4,800 control steps at 20 Hz (4 minutes). The scripted demo ends after about 172 s of simulated time.
- One trial took about 12 minutes on a shared NVIDIA GB10 (aarch64), start-up included.
- Seeds: the demo assumes `object_jitter=0`.
