# Put the tube end in the waste bottle

A pump's orange outlet tube (6 mm silicone, 55 cm, a small stainless sinker on its free end) lies straight across
the table from a pump housing on the left. Behind it stand four identical open-top square bottles, each with a
printed label card in front of it: **FEED, WASTE, PRODUCT, RINSE** (robot's left to right). The robot must read the
labels and put the weighted tube end into the **WASTE** bottle, at least 3 cm below the rim, then let go, without
the tube going into any other bottle. The bottles look the same; only the printed word tells them apart. WASTE is
neither the bottle nearest the tube end nor one at the end of the row. The tube end lies on the right arm's side,
but the WASTE bottle is reachable only by the left arm, so the natural solution is bimanual: the right arm picks up
the sinker and sets it down on the midline, the left arm takes it, carries it over the WASTE bottle and drops it in.

Labels are built from thin black boxes on white cards (the task-file API has no textures); the 29 mm capitals are
readable in the 320x176 top camera and in the wrist cameras.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cabledemo.py` (cable simulation, scripted-demo runner) and `boxfont.py` (block-letter labels)
  are in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "This tube is the pump's outlet. Put the weighted end of the tube into the bottle labelled WASTE so it rests at least 3 cm below the rim, then let go of the tube. Do not let the tube go into any other bottle."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop (every grasp aims at the sinker's measured position and heading): the right arm picks the sinker and
sets it down on the midline; the left arm picks it up, carries it 2.5 cm above the rims to the WASTE bottle,
releases it 1 cm above the rim and moves away. task_score 1.0 (success held 1 s, episode ends at step 360).
`demo/` holds its videos, `setup.png` and `result.json`.

Shortcut checks (all score 0): `TUBE_CHEAT=none` (do nothing), `TUBE_CHEAT=nearest` (drop the sinker into the
bottle nearest to where it lay, PRODUCT), `TUBE_CHEAT=hold` (hold the sinker at the WASTE bottle's mouth without
letting go).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if either gripper's TCP is within 3 cm of the tube or sinker (not released).
- **0** if any point of the tube or the sinker lies inside a bottle other than WASTE (within its 60 x 60 mm
  opening and below its rim).
- **0** if the sinker centre is not inside the WASTE bottle (within its opening and below its rim), e.g. on the
  table, on a rim, or outside.
- Sinker inside WASTE: **1.0** if its centre is at least 3 cm below the rim, else **0.5**.
- `distance()`: horizontal distance from the sinker to the WASTE bottle's centre (m).

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz); the demo finishes in about 18 s (360 steps). One trial takes a few minutes
  wall time on one GPU including the ~1–2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the tube is never jittered (it always starts straight), so every trial
  starts identically.
