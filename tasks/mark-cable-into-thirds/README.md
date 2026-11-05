# Mark a cable into equal thirds along its route

A black 6 mm cable, 75 cm long between its two plug bodies, lies on the table in an S round three fixed grey pegs.
Two orange marker pucks stand near the robot. Without moving the cable,
the robot must stand one marker beside the cable at one third of its length and the other at two thirds, measured
along the cable from plug to plug, and then let go. Because the cable winds round the pegs, the straight line
between the plugs says nothing about where the thirds are, so the robot has to measure along the cable. The
one-third point (25 cm, on the y +10 cm run) can only be reached by the left arm, and the two-thirds point (50 cm,
on the x 23 cm run) only by the right arm, so each arm places one marker. Aircraft structure, surfaces, rigging, and
systems assemblers "Verify dimensions of cable assemblies or positions of fittings, using measuring instruments"
([O*NET 51-2011.00](https://www.onetonline.org/link/details/51-2011.00)), and cable installers plan a pull by
"measuring the actual distance from one end of the cable run to the other" rather than guessing
([Cabling Installation & Maintenance](https://cablinginstall.com/design-install/cabling-installation/article/16465616/measure-your-cable-route-to-save-money)).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Without moving the black cable, stand one orange marker on the table beside it at one third and the other at two thirds of its length, measured along the cable from plug to plug. Then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live cable state: the left arm picks its marker, carries it with its wrist camera facing the
drop spot, and stands it 2.1 cm beside the cable at the point 25 cm along it, on the side away from the pegs. It
corrects from the marker's measured position just before setting it down, then lets go and lifts away. The right
arm does the same at 50 cm. Success holds for 1 s and ends the episode at step 337 (~17 s) with task_score 1.0.
`demo/` holds the videos, `setup.png` and `result.json`.

Shortcut variants (scoring checks, `MARK_CHEAT=<name>`), each scoring exactly 0:
- `none`: no motion.
- `chord`: each marker goes beside the cable at the cable point nearest the 1/3 or 2/3 point of the straight line
  between the plugs, as if the cable were straight.
- `middle`: both markers go beside the middle of the cable (37.5 cm ± 1.2 cm).
- `hold`: both markers are placed correctly, but the right arm keeps holding its marker.
- `memo` (run with `MARK_SLIDE=0.03`, so the cable lies 3 cm further along the same route): the markers go to the
  thirds of the nominal layout, i.e. memorised positions.

## Scoring (`score()`, evaluated at the end; `success()` must hold for 1 s)

A marker's along-cable position is the distance along the cable, from the start-plug face, to the cable point
nearest the marker's centre.

- **0** if a gripper is within 3 cm of the cable, a plug or a marker; or if any cable segment centre is more than
  2.5 cm (horizontally) from where it was laid. In real trials, also 0 if a peg was moved.
- A marker counts only if it stands on the table (not on the cable, a peg or the other marker) with its centre
  within 3 cm of the cable centreline.
- The two markers are paired with the 25 cm and 50 cm points in whichever way scores higher. Each earns 0.5 if
  it is within 1.5 cm of its point along the cable, 0.25 if within 2.5 cm, and 0 otherwise. Both within 1.5 cm
  gives 1.0 (success).
- `distance()`: the sum of the two along-cable errors (1 m for a marker that does not count).
- `progress()`: markers moved, markers beside the cable, credit at the thirds, cable kept in place, released.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). The demo needs 337 steps. One trial takes ~3–5 min of wall time on one
  GPU, including the ~1–2 min Isaac Sim start-up.
- The layout is fixed with `object_jitter=0`. The cable is authored in its laid S shape: its bend springs' rest
  angles are the laid angles, so every reset restores the shape exactly (and the simulated cable springs back
  towards the S if disturbed; a real cable has a weaker set and stays put mainly by friction).
- `VARIATION` in `task.py` lists ranges for a later seeded runner: sliding the cable ±3 cm along its route (which
  moves both thirds), shifting pegs and cable together, and jittering the markers.
