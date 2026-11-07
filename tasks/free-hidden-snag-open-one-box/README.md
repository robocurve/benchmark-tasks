# Free a cable caught inside one of three junction boxes by opening only that box, then pull it out

A building-wiring cable pull on the table. One black 6 mm cable runs through three identical closed grey junction
boxes standing in a row. Between the boxes and in front of the right box it lies in slack bows. Its tail end is
inside the left box; its plug lies in front of the right box. Inside one box, out of sight, the cable is caught: it
runs between two short pins, and a cable-tie head left on the cable cannot pass between them.

The robot is told only that the cable is caught in one of the boxes. It must open only that box, free the cable,
close the lid again, and pull the whole cable out of the boxes into the blue tray. Nothing on the boxes shows which
one it is. The cue comes from pulling: when the plug is drawn, the cable straightens from the plug up to the snag and
stops there. With the snag in the middle box, the bow between the right and middle boxes goes straight while the bow
between the middle and left boxes stays (measured in the demo: the right bow goes from 38 mm to 6 mm, the left bow
stays at 33 mm). Opening the left or right lid more than 10 degrees at any time scores 0 for the run, so opening every box
to look scores 0.

## Objects (all fixed unless stated)

| object | type, size | material (real rebuild) | position (cm from the arm-base midpoint, x forward, y left) |
|---|---|---|---|
| 3 junction boxes LEFT / MIDDLE / RIGHT | open-bottom boxes, 10 x 10 cm, 5 cm tall, 5 mm walls | grey PLA or plywood | centres x 34, y +33 / +12 / -9 |
| cable holes | 22 mm wide x 18 mm tall, at the foot of both side walls of every box | cut in the walls | centred at x 33.5 |
| 3 lids | 10 x 10 x 0.5 cm, 40 g, hinged along the far top edge, 0-100 deg stop | light grey PLA, brass butt hinge | on the boxes |
| 3 lid tabs + knobs | 3 cm x 6 mm tab at the lid's front-left corner; 16 mm knob on a loose pin | PLA tab, dark knob on M4 shoulder screw | tab at y +5 to +8 cm from each box centre |
| 2 pins (MIDDLE box only) | 8 mm diameter, 16 mm tall, 9 mm clear gap | steel dowels | x 33.5 +-0.85, y 10.5 |
| cable | black 6 mm round cable, 72 cm, free (not anchored) | PVC-jacketed 6 mm cable | tail end inside LEFT box (y about 36), through all six holes, 4 cm bows towards the robot between the boxes and from the RIGHT box to the plug |
| plug | 35 x 16 x 12 mm, 15 g, on the cable's end | black plastic | about x 34, y -29 |
| cable-tie head | rigid 12 x 13 x 12 mm block on the cable | white nylon tie head (printed block) | inside MIDDLE box, on the tail (+y) side of the pins, about 5 mm from them |
| tray | inner 16 x 18 cm, 2 cm rim, 5 mm walls | blue plastic tray | centre x 19, y -34 |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (cable simulation, the bowed start layout,
  scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "The black cable is caught inside one of the three junction boxes. Free it by opening only that box, close its lid again, then pull the whole cable out of the boxes into the blue tray and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed loop on the live cable, lids and tie head:

1. Right arm takes the plug and pulls it away from the right box in 1 cm steps until it stops following (the tie has
   jammed against the pins). The right bow is now straight, the left bow is not: the snag is in the middle box.
   It eases 5 cm of slack back.
2. Left arm swings the middle lid open by its knob, past upright onto its stop, and lets go.
3. Left arm pinches the tie head (pads clear of the pins), lifts it over the pins, sets it down on the plug side of
   the pins, then swings the lid shut and moves clear.
4. Right arm lays the plug in the tray, then draws the cable out of the right box's hole in strokes into the tray,
   tidies any stretch lying over the tray's rim, and moves clear.

`demo/` holds the videos, `setup.png` and `result.json`.

## Scoring (`score()`, at the end; `success()` must hold 1 s)

- **0** if, at any time, the LEFT or RIGHT lid opened more than 10 deg (latched for the whole run); if either
  gripper's TCP is within 3 cm of the cable, plug, tie head, any lid, tab or knob at the end; or if the tie head is not
  yet out past the RIGHT box (its centre beyond the RIGHT box's outer -y face and in no box).
- **0.3** the tie is freed and drawn out past the RIGHT box, but some cable is still above a box's footprint.
- **0.6** no part of the cable or plug is above any box's footprint.
- **1.0** in addition, every point of the cable and the plug is inside the tray's inner outline and no higher than
  1 cm above its rim, and the MIDDLE lid is closed (within 5 deg).

Each tier is a physical end state: 0.3 needs the snag freed and the cable pulled out at least past the right box;
0.6 needs the whole cable out of all three boxes. Touching, grasping or lifting anything without freeing and pulling
earns nothing, because the tie head cannot leave the middle box while it is caught.

Shortcut and partial variants (test only, `SNAG_CHEAT=<name> python cabledemo.py task.py out`):

| variant | what it does | score |
|---|---|---|
| `none` | no motion | 0 |
| `yank` | right arm alone pulls the plug hard and carries it to the tray without opening anything | 0 (the sim goes non-finite, which is latched as a violation; tie_out was never reached) |
| `onearm_left` | left arm alone opens the middle box, frees the tie, closes the lid; nobody pulls | 0 (tie never leaves the boxes) |
| `wrong_right` | opens and closes the RIGHT box first (the box next to the plug), then does the whole job right | 0 (latched) |
| `all_lids` | opens and closes the RIGHT and LEFT boxes, then does the whole job right | 0 (latched) |
| `hold` | whole job, but the left gripper keeps pinching the middle knob and the right one re-grips the cable at the end | 0 |
| `tier30` | freed, then drawn only until the tie is out past the right box | 0.3 |
| `tier60` | whole cable drawn out of the boxes, lid left open, not tidied into the tray | 0.6 |

## Why two arms (offline IK, gripper straight down, any yaw; position error shown)

| point (x, y, height above table, m) | left arm | right arm |
|---|---|---|
| middle knob, lid shut (0.293, 0.197, 0.067) | 0.0 mm | no solution |
| middle knob, lid at 60 deg (0.342, 0.197, 0.151) | 0.0 mm | no solution |
| middle knob, lid at 90 deg (0.390, 0.197, 0.164) | 0.0 mm | no solution |
| tie head when caught (0.33, 0.13, 0.016) | 0.0 mm | no solution |
| plug (0.335, -0.30, 0.02) | no solution | 0.0 mm |
| right box's -y hole, outside (0.335, -0.15, 0.015) | no solution | 0.0 mm |

Furthest reach towards the other side, 1.5 cm above the table: the right arm gets to y +0.08 at x 0.34 (+0.11 at
x 0.31, +0.13 at x 0.28); the middle box's inside starts at y +0.075 and its lid can only be opened by its knob at y
+0.197. The left arm gets to y -0.08 at x 0.34 and -0.17 at x 0.20, short of the right box's outer hole (y -0.145),
the plug (y -0.29) and the tray (y -0.25 to -0.43). So freeing the tie needs the left arm and drawing the cable out
past the right box needs the right arm; `onearm_left` and `yank` confirm it.

## How this differs

- From the trace-a-cable task (tug a cable where it leaves its box and watch which plug moves): that task identifies
  *which of several cables* belongs to an end. Here there is one cable; a hidden catch can be located from the shape
  the cable takes under pull (a cue, not enforced), then exactly one enclosure is opened and the catch cleared. The
  wrong guesses are punished by opening a box, not by picking a wrong plug.
- From board tasks that free a cable (from under stacked cases, from behind a box, pull one cable from a bundle): there
  the obstruction is visible. Here it is hidden and must be located by a pull test before any opening.
- From lid/box tasks (e.g. close a floor box with a cord in its notch): the lid is a means; the skill tested is
  clearing a hidden snag on a deformable cable and extracting the cable through three enclosures.

## Paper fields

- Skill tags: extraction, sequencing, constraint-keeping (one box only), optional cue reading (cable straightness under pull).
- Industry: building wiring / cable pulling (O*NET 47-2111.00 Electricians; 49-9052.00 Telecommunications Line
  Installers and Repairers).
- Two-arm need: reach-forced. The plug, the right box's hole and the tray are reachable only by the right arm; the
  middle lid's knob and the tie head only by the left arm (numbers above).

## Sources (quotes copied from the pages)

- O*NET 47-2111.00 Electricians (onetonline.org/link/summary/47-2111.00), task: "Place conduit, pipes, or tubing,
  inside designated partitions, walls, or other concealed areas, and pull insulated wires or cables through the
  conduit to complete circuits between boxes."
- O*NET 49-9052.00 Telecommunications Line Installers and Repairers (onetonline.org/link/summary/49-9052.00), task:
  "Pull cable through ducts by hand or with winches."
- OpenExamPrep study guide for the BICSI Installer 2 Copper exam, "7.1 Cable Pulling Setup, Pull String, and Tension
  Control" (open-exam-prep.com/study-guides/bicsi-inst2-copper/pull-copper-cable/cable-pulling-setup), a third-party
  guide that says it is not affiliated with the exam sponsors, not a BICSI document: "On long or riser pulls, station an intermediate person at each pull box."; "Stop
  immediately if the cable binds."; "Slow the pull through bends and ensure personnel on the feed end are not pushing
  cable into the conduit—pushing compounds tension at snag points."
- None of these says, in so many words, to open only the snagged box or to find the snag by pulling. Opening only one box is this task's rule (each
  opened box costs time and its cover must be refitted); it is stated in the instruction.

## Runtime and seeds

The demo reaches 1.0 at step 1,264 (63 s at 20 Hz); a 2,400-step trial takes a few minutes on one GPU. The layout is fixed
(`object_jitter=0`); the bowed start is written at every reset (`cableshape`, cable keeps the set) and is
deterministic.

## Limits

In the 320 x 176 top camera a 4 cm bow spans only a few pixels, so the cue is faint from above; the wrist cameras see
the gaps closely when an arm is near the boxes. Whether a policy can read it from the cameras is untested.

The task does not test the diagnosis. The layout is fixed, so a policy that always opens the middle box scores 1.0
without pulling first or reading any cue; the pull cue only lets a policy that does not know the layout find the box. A fixed pick of the right box (nearest the plug) or the left box scores 0, and a
random pick of one box is right 1 time in 3. The review rules forbid per-episode swaps, so this is the fixed-layout
limit, not something the scoring can block.
