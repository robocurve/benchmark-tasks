# Reset a peak flow meter's marker to zero before handing it to the resident

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [reset_peak_flow_meter.py](reset_peak_flow_meter.py).

A resident's peak flow meter lies on a care-home bedside table with its red marker still high up the scale from the last reading. The instruction is to put it on the tray in front of the resident for their morning reading. What goes unsaid is that the marker has to be slid back to zero first, or the reading starts from the old one and is wrong, and that the meter is handed over mouthpiece first. The robot has to turn the meter half a turn so the mouthpiece faces the resident, set it on the tray, and push the marker back along its slot to the zero stop.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=reset_peak_flow_meter.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the resident's peak flow meter on the tray in front of them for their morning reading." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/peak-flow-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`. The left arm turns the meter half a turn in two quarter turns with a fresh grip between, carries it to the tray, then lowers its closed fingers behind the marker and pushes it along the slot to the zero stop. Resetting last keeps the marker from being jolted back up the scale while the meter is carried.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the marker and the meter end.

## Checks

Scripted variants of the same scene, each replacing only `demo()`:

| Variant | Score |
|---|---|
| Demo: turn, tray, reset | 1.0 |
| Turn and tray, marker not reset | 0.25 |
| Marker reset where the meter lies, not taken to the tray | 0.5 |
| Not turned (mouthpiece away from the resident), tray, reset | 0.75 |
| Nothing done | 0.0 |

The demo was also run for two trials in one process; both scored 1.0.

## Runtime and seeds

- Demo: about 800 steps (40 s simulated), about two minutes on an L40S.
- Dream-1: 1,200 steps (1 minute simulated), about 3 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it. It builds the meter and its marker as rigid parts made of several boxes.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
