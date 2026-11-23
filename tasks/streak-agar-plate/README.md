# Streak an agar plate in four quadrants with an inoculating loop

An uncovered agar plate rests on the bench in front of the right arm. A marker cross on its underside divides it into
four numbered quadrants, and a 10 µL drop of the sample sits in quadrant 1. A sterile disposable loop lies on the
bench beside the plate. The robot must streak the plate for isolated colonies, the way microbiology labs do it (the
CDC's four-quadrant streak): drag the loop through the drop and streak quadrant 1 back and forth, then streak
quadrants 2, 3 and 4 in turn, each starting with a few strokes back through the quadrant before it, with the loop
gliding on the agar without digging in. Then it puts the loop in the discard tray.

Streaking for isolation is the standard way a microbiology, clinical or research lab gets a pure culture from a
sample: each quadrant thins the inoculum picked up from the one before, so single colonies grow in the last
quadrants (CDC, "Quadrant streaking for isolation" job aid; Sanders 2012, "Aseptic laboratory techniques: plating
methods", J. Vis. Exp. 63, e3064). The difficulty is physical: the slim handle has to be picked up off the bench and
held like a pen, and the 4.5 mm ring drawn over a soft gel in long strokes with almost no force: held off the
surface it draws nothing, and pressed past about a quarter of a newton it tears the agar; each quadrant's first
strokes must dip back into the one before.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the streak record, `score()`, `success()`, `self_test()` and the scripted `demo()` |
| `contactkit.py` | Reads the PhysX contact force between the loop and the agar |
| `touch.py` | Reads whether any robot link touches the agar or the loop (for "let go") |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `tiltkit.py` | Any gripper orientation for the scripted demo (the loop is held tilted, like a pen) |
| `assets/plate.usdc` | The Petri dish base (92 mm outside, 88 mm inside, 13.5 mm tall), a fixed body, 23 KB |
| `assets/agar.usdc` | The agar (88 mm across, 3.3 mm deep), with the quadrant cross, quadrant numbers (dots) and the drop drawn on it, a fixed body, 76 KB |
| `assets/loop.usdc` | The 10 µL loop (200 mm): its handle and, on a sprung hinge at the neck, the neck and ring (two links of a free articulation), 21 KB |
| `assets/height_cal.npz` | The scripted demo's height correction along its streak path (how far the arm rides above or below its planned height at each point, measured by running the plan just off the agar; `work/streak/make_cal.py`), a few KB |

No third-party assets; `work/streak/make_parts.py` builds the three USD files from the dimensions below.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top. Same layout every trial.

| Object | Where | What |
|---|---|---|
| Agar plate | centred at (34.0, -6.0), resting on the table, lid off and set aside (not in the scene) | Fisherbrand FB0875712 polystyrene Petri dish base, 100 x 15 mm style: 92 mm outside, 88 mm inside (Fisher's product note), 13.5 mm tall; holding 20 mL of 1.5 % agar medium (for example LB agar), 3.3 mm deep, poured level and set, at room temperature, with no surface water |
| Quadrant marks | on the plate's underside | A cross in black marker through the centre, one line along x and one along y, dividing the agar into quadrants 1 (far right: x > 0, y < 0 from the plate's centre), 2 (far left), 3 (near left) and 4 (near right), numbered by 1 to 4 dots near the rim |
| Sample drop | quadrant 1, centred 2.6 cm forward of and 1.1 cm right of the plate's centre | 10 µL of water tinted with blue food colouring, pipetted onto the agar just before the trial (a wet patch about 5 mm across) |
| Loop | lying flat on the table along x, ring towards the robot: ring's centre at (20.2, -20.0), butt at (40.0, -20.0) | Sterile disposable inoculating loop, 10 µL, polypropylene, 200 mm long, 3.6 mm handle, ring 4.5 mm across (MTC Bio M4025 or Heathrow Scientific 03-448-345), fresh from its pack |
| Discard tray | centred at (16.0, 5.0), between the arms, its long side along x | Autoclavable polypropylene tray, 25 x 18 cm, 4 cm deep |

## Scoring

Score from the trial video (the top camera) and a photo of the plate from above at the end. A **streak** is a line
the loop's ring draws while resting on the agar; quadrants are those marked on the underside.

- **0** if the agar is gouged (torn or cut into anywhere, seen in the photo) or a gripper touches the agar.
- **0.25** for quadrant 1: at least 40 mm of streak drawn in quadrant 1, through the drop.
- **0.5**: and quadrant 2 streaked after quadrant 1 (at least 30 mm of streak in quadrant 2), with at least 2 strokes
  that run back from quadrant 2 into quadrant 1.
- **0.75**: and quadrant 3 likewise after quadrant 2 (at least 30 mm, at least 2 strokes back into quadrant 2).
- **1.0**: and quadrant 4 likewise after quadrant 3 (at least 30 mm, at least 2 strokes back into quadrant 3,
  never touching quadrant 1), and the loop lying in the discard tray (all of it inside the tray's walls), let go, for 1 s.
- A quadrant counts only in turn: one streaked out of order (a later quadrant before an earlier one) adds nothing.
- At most **0.25** if the loop goes back to the drop after quadrant 1 is done (re-inoculating).
- At most **0.75** until the loop lies in the discard tray and the gripper has let go of it.

The gouge limit: suppliers give 550-950 g/cm² as the gel strength of 1.5 % bacteriological agar (Thermo Fisher
J10906, Avantor, Neogen); at the lowest, 550 g/cm² over the ring's contact patch (its 0.8 mm strand resting along
about half its 4.5 mm round, 0.045 cm²) is 25 g, so the simulation counts a contact force over 0.25 N as a gouge.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Streak the plate for isolated colonies: drag the loop through the drop and streak quadrant 1, then streak quadrants 2, 3 and 4 in turn, each starting with a few strokes back through the one before, without digging into the agar. Then put the loop in the discard tray." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/streak-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Seed 0; the
scene has no random placement (`-E object_jitter=0`). On an RTX 3060 the scripted demo (64 s of simulated time) takes about 4 minutes including start-up; a full 3,600-step trial takes about 10 minutes.

Offline checks (no GPU): `python task.py` runs the scoring rules on hand-made streak sequences and replays the demo's
plan through the arm's kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "..." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/streak-demo
```

The right arm picks the loop up by its handle, tilts it down to 50 degrees like a pen, drags it through the drop
and zigzags over quadrant 1, then streaks quadrants 2, 3 and 4 (each starting with three strokes back into the one
before) at 20 mm/s with the ring resting on the agar, lifting 15 mm between quadrants, and lays the loop in the
discard tray. `STREAK_LOG=1` prints the force, the streak lengths per quadrant and the strokes back every 0.5 s.

## Reset

Use a fresh agar plate (marked and with a new drop) and a fresh loop for every trial; empty the discard tray.
