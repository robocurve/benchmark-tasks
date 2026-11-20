# Fit and verify a mobile robot's front bumper

Fit the replacement front bumper to the mobile-base fixture. Push the center of the bumper to verify the switch, then let it return.

A mobile robot needs a freely returning bumper to detect physical obstacles. Correct replacement aligns both keyed mounting points so that the center contact switch triggers and clears after release.

## Setup

De-energized 171 × 174 mm base plate with a spring-return X sled and two keyed sockets spaced 84 mm apart. The moving fascia has a central 88 mm-wide pad and two 14 × 15 mm mounting posts. With table height as Z zero, the base origin is (304, −70, 0) mm and the loose fascia datum is (210, −224, 5.4) mm.

## Scoring

0.30 with both posts within 5 mm in the table plane and 4 mm vertically of their respective sockets. The upright-facing checks require the local Z and X axis dot products with the fixture axes to exceed 0.986 and 0.980. 0.65 after a robot touch on the mounted fascia during 4.8–11 mm of compression. Full credit with the fascia still mounted, the sled below 1.3 mm of residual stroke, and both released for one second.

## Reset

Return the sled to zero and move the fascia to its tabletop marker. Clear the actuation test.

## Physical replication

Fabricate the two-pin carrier from PETG or aluminum using the generated dimensions. Fit a linear slide, approximately 170 N/m return spring, closed body spacer and small lever switch. Use calipers or a side ruler for compression and return. Document any fixture changes before comparing trials.

Zero a dial indicator on the unloaded sled. With the fascia released, verify that both posts are in their pockets and measure the seating errors to the fixture datum. Push the center pad once while recording switch continuity and sled displacement. Check that the switch closes during 4.8–11 mm of travel, reopens on release, and the residual travel stays under 1.3 mm for one second. Return the fascia to its start marker for the next trial.

## Fidelity

A custom fabricated rigid bench section grounded in iRobot bumper reseating and Kobuki contact detection. It does not reproduce either OEM fastening pattern. Sled motion is physical and switch logic is derived from travel.

[iRobot bumper reseating](https://homesupport.irobot.com/articles/en_US/Knowledge/20711) · [Kobuki center bumper output](https://yujinrobot.github.io/kobuki/enAppendixProtocolSpecification.html)

## Run in YAM Arena

Run from this task folder. The recorded settings use seed 0, 20 Hz, zero object jitter, and the step limit and instruction in `task.json`.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$instruction" \
  --seed 0 --scorer task_score --max-steps "$steps" --no-prompt --store-frames \
  --log-dir "logs/demo-$(date -u +%Y%m%dT%H%M%SZ)"
```

Use `inspect-robots inspect <saved-eval.json>` to read the final `task_score` and verify the released end state in the saved frames.

The Lambda scripted demo reached **1.00** in `adhoc_ae1d2656.json`. The scripted arm corrects XY and yaw using the measured fascia pose, then seats both posts. A 170 N/m return force acts on the physical sled at each physics step for every policy. The switch was active at a sampled 8.9 mm of compression, and residual travel was 0.0 mm after release.

### Dream-1

From the task folder with the same environment, instruction, and limit:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --seed 0 --scorer task_score --max-steps "$steps" --no-prompt --store-frames \
  --log-dir "logs/dream-$(date -u +%Y%m%dT%H%M%SZ)"
```

Dream-1 scored **0.0** in the saved evaluation `adhoc_35b77731.json`.
