# Pull one decommissioned cable out of a clipped bundle

A white open-ended cable tray (40 cm long, left-right in front of the robot, 8 cm wide inside, 3 cm walls) holds four
6 mm cables lying side by side, held in by two black bridge clips spanning the tray. Three grey cables are "live";
the blue one, second from the robot (not the nearest one), is decommissioned. All four leave the tray's right end and
end in identical black plugs on the table, staggered so that neither the plug nearest the robot (grey A) nor the
one sticking out furthest (grey C) is the blue one. The robot must find the blue cable, draw it out lengthwise
through both clips and out of the tray's right end (its left end lies inside the tray, so it cannot simply be lifted
out: the clips hold it down), lay it entirely outside the tray and let go, while every grey cable stays under both
clips with its plug within 3 cm of where it started. The cables are 15 mm apart, so a regrasp of the blue cable
near the tray end has to avoid the neighbouring cables.

The scripted demo is bimanual (the left arm holds the live cables down while the right arm pulls), as a technician
would do; in the simulation, pulling the loose, untied blue cable barely drags its neighbours (< 5 mm), so a single
arm can also succeed. The skill tested is identification plus hand-over-hand extraction through clips without
disturbing adjacent cables.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "The blue cable in the white tray is decommissioned. Pull it out of the tray through the two black clips and lay it on the table completely outside the tray, then let go. The three grey cables are live: they must stay in the tray under both clips."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed-loop (reads the live cable states): the left arm presses the live cables onto the tray floor left of the
blue cable's end (fingers straddling the bundle). The right arm grasps the blue plug, pulls it up and to the right
(towards the robot), lets go, then re-grasps the blue cable outside the tray at the first point that is at least
3 cm from every other cable and pulls again, until no part of it is over the tray; both arms let go and lift.
Success holds for 1 s and ends the episode at step ~317 (~16 s); task_score 1.0. `demo/` holds its videos,
`setup.png` and `result.json`.

Shortcut variants (test only, `BUNDLE_CHEAT=<name> python cabledemo.py task.py out`), all score exactly 0:
`nothing` (do nothing), `nearest` (pull the cable nearest the robot, grey A, the same way the demo pulls blue),
`longest` (pull the cable whose plug sticks out furthest, grey C).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if either gripper (point between the finger pads) is within 4 cm of any cable or plug (not let go).
- **0** if any grey cable no longer passes under both clips, or any grey plug centre is more than 3 cm
  (horizontally) from its starting position.
- Otherwise **1.0** if no point of the blue cable or its plug is over the tray's footprint in top view (walls
  included, any height); **0.5** if the blue cable passes under neither clip but part of it is still over the tray;
  **0** if it still passes under a clip (e.g. untouched).
- `distance()`: blue cable length still over the tray (m), 1 m if a grey cable is disturbed.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~5-10 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; cables are never jittered (they always start straight), so every trial
  starts identically.
