# Find the ECG lead with an intermittent break by flexing each lead

Three identical ECG lead wires are connected to a training lead tester. Each lead's connector sits in a seat in front
of the tester, its cord runs across the table, and its snap clip lies on a brass shorting bar. Each lead has a green
status lamp on the tester. One lead has an intermittent break at the strain relief next to its connector. Its lamp
stays lit while the lead lies still and goes dark only while that part of the cord is bent. Nothing in the scene
shows which lead is faulty. The robot has to make the fault appear: flex each lead near its connector, watch the
lamps, take the lead whose lamp went dark out of the tester by its connector, and put it in the red reject tray. The
good leads must stay connected throughout.

The tested ability is interactive perception. The faulty lead can only be found by deforming each cable in the right
place and reading the result, and then the robot has to act on what it saw. The reject step is the add-on: no credit
is possible unless every lead was flexed.

Instruction: `Three ECG lead wires are connected to this training lead tester, and one of them has an intermittent break. Find the faulty lead and put it in the red reject tray.`

## How this differs from similar tasks

- Tasks that sort parts with a tester judge rigid parts by placing them in a fixture and reading a lamp. In this task
  the fault exists only while the cable is bent, so the robot must deform each cable near its connector to reveal it.
- Tasks that find a loose plug or match leads by label or colour use a cue that is visible from the start. Here all
  three leads look the same until each one is flexed.

## Why two arms (reach)

Offline IK, with the gripper pointing down:

- Only the left arm reaches the connector seats at y +18 cm. The right arm misses them by 23 to 68 mm (lead a 23 mm,
  b 42 mm, c 68 mm).
- Only the right arm reaches the reject tray. The left arm misses even the tray's nearest inner corner by about
  5 cm.
- So the left arm takes the faulty connector out of its seat and the right arm puts the lead in the tray.
- Pulling a lead out of the tester by its cord is a scored violation (score 0) when no gripper is within 2.5 cm of
  the connector. Real lead wires are unplugged by the connector body, because yanking the cord is what breaks them at
  the strain relief. The right arm's TCP gets no closer than about 42 mm to lead a's connector centre (IK: it can
  reach a grasp on the cord no further than y +13.8 cm, while the connector centre is at y +18 cm). So a right-arm-only
  pull always trips this rule; the `cordpull` variant scores 0. The left arm's pinch holds its TCP about 10 mm above
  the connector centre.

Flexing a lead does not need its connector held. In this rig the seat rails hold an unheld connector within 1 to
2 mm while the cord is flexed, so the right arm flexes every lead on its own. During each test the left arm hovers
over the lead's connector, and its wrist camera shows that lead's lamp, 26 mm across, filling much of the frame. The lamps are best read in that wrist view, where a dark lamp is plain at 320x176. From the top camera the tester is small and partly hidden by the left arm.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cables) is in this folder next to `task.py`, and `cabledemo.py` runs the scripted
  demo.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Three ECG lead wires are connected to this training lead tester, and one of them has an intermittent break. Find the faulty lead and put it in the red reject tray."
```

Run one trial per process, or one per embodiment reset. The scorer keeps a short history of the trial (lamps,
tests, how connectors left their seats), and `task.py` clears it on every embodiment reset. The layout is fixed
(object_jitter=0, and cables are never jittered). The faulty lead is always lead a, the one nearest the robot and
farthest from the tray.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

For each lead in turn:

1. The left arm hovers over the lead's connector, with its wrist camera on the lead's lamp.
2. The right arm takes the cord 20 cm from the connector and pushes it 5 cm towards the tester while lifting it
   5 cm. The cord humps and bends about 40 degrees where it leaves the connector.
3. The right arm lays the cord back down.

The script records which lamp went dark (only lead a's). The left arm then lifts that connector out of its seat by
the connector body and sets it down near the midline. The right arm lifts the lead by the middle of its cord, lowers
it into the tray and tucks in any part left lying on the rim. Both arms then park. The demo reaches `task_score` 1.0 in 939 steps (47 s at 20 Hz). Videos are in `demo/`.

## Scoring (`score()`; `success()` = score 1.0 with every lead at rest, held 1 s)

Score 0 if any of the following holds:

- **A good lead's lamp went dark at any moment** (latched). A lamp goes dark when its connector moves more than
  4 mm from its seat, rises more than 5 mm, or turns more than 12 degrees out of line.
- **A connector left its seat by more than 1 cm while no gripper was within 2.5 cm of it** (latched). This means it
  was pulled out by its cord.
- **Not every lead was tested.** A lead counts as tested once its cord, 5 to 7 cm from the connector, was bent more
  than 30 degrees from the connector's axis while the connector was seated. An unflexed lead never shows its fault,
  so taking a lead out without testing all three (a random or fixed pick) scores 0.
- **Not released.** At the end, a gripper TCP is within 3.5 cm of any part of any lead.

Otherwise:

- **1.0:** the faulty lead lies entirely in the tray. Every connector, cord and snap point is inside the tray's
  inner footprint (3 mm margin) and below 4 cm above the table.
- **0.5:** the faulty lead is out of the tester (its connector at least 3 cm from its seat and its snap off the
  bar) but is not entirely in the tray.
- **0:** anything else.

Removing a good lead darkens its lamp, so it scores 0.

Chance: the faulty lead is fixed (always lead a) on a fixed layout, and `score()` does not check that the robot looked
at a lamp. A blind pick scores 0 because nothing counts until all three leads are flexed. A policy that flexes all
three leads and then always takes lead a would still score 1.0; the convenient wrong choices (b, c) are the leads
nearer the tray.

## Shortcut and tier variants (`WIGGLE_DEMO=<name>`, test only)

| variant | what it does | score |
|---|---|---|
| `none` | does nothing | 0 (no lead tested) |
| `notest` | puts the faulty lead in the tray correctly, but flexes no lead first (a lucky or fixed pick) | 0 (leads not tested) |
| `near` | tests all three, then rejects the lead nearest the tray (a good lead) | 0 (good lamp went dark) |
| `pullall` | takes all three leads out of the tester by their connectors | 0 (good lamps went dark, nothing tested) |
| `flexonly` | tests all three, rejects nothing | 0 (faulty lead still in the tester) |
| `cordpull` | tests all three, then the right arm alone pulls the faulty lead out by its cord and puts it in the tray | 0 (connector pulled out by the cord) |
| `hold` | tests, then lowers the faulty lead into the tray but keeps gripping it | 0 (not released) |
| `tier05` | tests all three, takes the faulty lead out and lays it on the table beside the tray | 0.5 |
| (default) | the full demo | 1.0 |

Runtime: about 8 to 12 minutes of wall-clock time for a 2,400-step trial on an L40S (2 to 5 simulation steps per second).
