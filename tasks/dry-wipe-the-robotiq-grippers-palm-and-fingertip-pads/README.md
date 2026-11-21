# Dust a UR5e's Robotiq 2F-85 gripper on the arm: wipe its deep palm pad and both fingertip pads with only the cloth

A Universal Robots UR5e stands powered off at the far end of the bench, jogged to a service pose with its wrist low
and its tool pointing back at the two arms. On its flange, on its coupling, is a Robotiq 2F-85 gripper with its
fingers parked fully open. Its three grip surfaces (the rubber palm pad between the finger bases and the rubber pad
on each fingertip's inner face) are dusty. A folded microfibre cloth lies on a service mat in front of it. The robot
must wipe all three pads clean with the cloth, touching them with nothing else, and put the cloth back on the mat.

## Why it matters

- Robotiq's 2F-85 manual (section 5.1.3): "The contact grip points for the Robotiq 2-Finger Adaptive Gripper are its
  two fingertips and palm pad." Section 6.1, Gripper cleaning: maintenance interval "Weekly or daily in dirty
  operating conditions", tools "Dry tissue or towel", and the caution "only clean the gripper with a dry towel";
  step 2 opens the fingers "to access the palm pad", step 3 says "clean all fingertips". Source: Robotiq,
  *2F-85 & 2F-140 Instruction Manual for Universal Robots* (2024-02-26),
  https://s3.amazonaws.com/com-robotiq-website-prod-assets/website-assets/support_documents/document/Robotiq_2F-85_and_2F-140_Instruction_Manual_Universal_Robots_20240226.pdf
  (pages 121 and 133).
- O*NET 17-3024.01, Robotics Technicians, core task: "Perform preventive or corrective maintenance on robotic
  systems or components."
- The 2F-85 is a standard gripper on arms that collect robot-learning data: the DROID dataset's shared platform
  (76k trajectories, Khazatsky et al., 2024, arXiv:2403.12945) is a Franka Panda "equipped with a Robotiq 2F-85
  gripper", and the 2F-85 is sold as a UR+ product for UR arms. Every such arm needs this upkeep weekly; a robot that
  can do it on another robot, with the arm left in place, takes a recurring human chore out of running robot fleets.

## What the robot has to do, and why each part is hard

1. **Take the cloth and hold it as a reaching tool.** The folded strip (120 x 40 x 28 mm) is held by one end so its
   other 100 mm reach in front of the fingers: the gripper's fingers are 85 mm apart and its palm sits 58 to 59 mm
   behind the fingertips' ends, so a hand that goes in after the cloth would touch the pads (which rules the trial
   out).
2. **Clean the palm pad, deep in the gripper.** The palm pad (31 x 22 mm) faces the arms at the bottom of the gap
   between the fingers. The cloth's end must be pushed onto it, square, and slid across it. The finger links stand
   30 mm either side of the pad's centre line right in front of it, so the 40 mm cloth can sweep only about 8 mm each
   way; a wider sweep hits the links and lifts off the pad.
3. **Clean the two fingertip pads, which face each other.** Each pad (16.2 x 38 mm of rubber) is a vertical face
   inside the gap. The cloth's side has to be pressed sideways onto it (a sideways push, not the downward press most
   wiping uses) and moved over it, with the cloth's end kept short of the fingertip holder and the hand outside the
   fingers. A cloth pressed flat on one fingertip at the wrong height wipes only part of it.
4. **Cover everything.** Each pad is split into zones (2 on the palm, 3 on each fingertip); every zone must be wiped.
5. **Put the cloth back** on the mat, let go, and move both hands away.

## Answers to questions a reviewer may ask

- **Is this the maker's whole cleaning procedure?** No. It is step 3 of it, the step that keeps the gripper
  gripping: wiping its three contact points (the two fingertips and the palm pad) with a dry towel. The maker's step 1
  removes the gripper from its coupling (which also lets step 4 clean the coupling's electrical contacts), and step 2
  turns the drive screw under the gripper to open the fingers. Here the gripper stays on the arm with its fingers
  already open, so the pads are reached through the gap between the fingers from the arms' side, with the arm and
  coupling in place. Removing and refitting the gripper (four M5 screws with tooth lock washers), turning its 2 mm
  drive screw and cleaning the coupling are left out: they are threaded-fastener and small-part steps beyond these
  grippers.
- **Why are the fingers open at the start?** The cleaning needs them open ("Open or close to access the palm pad").
  With the gripper unpowered its fingers move only through that drive screw, so the task starts after the fingers
  were opened (for example by commanding the gripper open before switching it off). They stay fixed during the task.
- **Is the dust real?** In a real trial it is white marking chalk brushed through a stencil onto the three pads
  (fixture/zones-1to1.svg). In the simulation the dust is a stated rule (see Fidelity): 60 visible cells that clear
  only where the cloth slides over them while pressing on that pad. A cloth held near a pad, pressed without sliding,
  or touching another part of the gripper clears nothing.
- **Can the robot clean with its own fingers?** No. The cloth does the cleaning: a robot finger pressed on a pad
  only pushes the dust around and leaves its own residue on the gripper's grip surfaces, which the maker's procedure
  cleans with a dry towel and nothing else. Any contact between a robot finger or gripper and a pad scores 0 for the
  trial; the simulation reads it from the contact forces between each pad and the hand links (over 0.5 N), a real
  trial from the wrist-camera video.
- **Is a folded cloth stiff enough to be pushed end-first?** A 320 x 360 mm microfibre wipe folded to a 120 x 40 mm
  strip is 24 layers thick (about 28 mm, loosely folded); held 18 mm from one end, its free 100 mm is expected to stay nearly straight
  under a few newtons pushed along its length or across its 40 mm face (an estimate, not measured). The simulation treats it as one rigid block; it does not model the
  strip bending or the outer layer creasing, and the real trial will show how far the strip gives.
- **How is "clean" read in a real trial?** Photos of each pad, square-on through the 1:1 stencil, before and after.
  A zone is clean when at most 10% of its area still shows chalk (the stencil's thin grid marks the 5 to 6 mm cells
  the simulation uses).

## Nearest tasks, and how this differs

- **Cleaning an SC fibre connector's ferrule on a cassette cleaner, then latching it (TK-6Z2YW3, accepted):** a
  one-way wipe of a part held in the hand across a fixed cleaner. Here the surfaces to clean are fixed on a mounted
  robot gripper, inside it: the cloth is the moving tool, reaching into a confined gap and pressed sideways on facing
  pads, with coverage of three separate pads.
- **Replacing a worn soft gripper finger (accepted):** robot-gripper upkeep by swapping a part. Here nothing is
  swapped: the skill is controlled wiping with a held cloth on the gripper's grip surfaces, in place on the arm.
- **Cleaning a robot's wrist camera with a brush (closed, not accepted):** the robot cleaned a flat, open window on
  its own arm, and the gripper could clean it without the brush. Here the three surfaces are inside another robot's
  gripper, the contact rule rules out any cleaning except with the cloth, and the pads are the gripper's working
  surfaces named by its maker's maintenance schedule.
- **Brushing a car battery post clean (accepted):** a round post brushed from outside. Here: flat faces in a narrow
  gap, reached from the open end, two of them facing each other.

## Run a trial

From this folder, in the Isaac Lab-Arena environment of the program's simulation walkthrough (yam-isaac-lab-arena,
the program's YAM embodiment for Isaac Lab-Arena). Trial k uses lot k mod 2, so `--epochs 2` runs both lots and
`task_score` is their mean:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Wipe the dust off the gripper's palm pad and both fingertip pads, touching them only with the cloth, then put the cloth back on the mat." \
  --scorer task_score --max-steps 2400 --epochs 2 --store-frames --log-dir ~/yam-eval/gripper-pads-dream
```

- The scripted demo: the same command with `--policy yam_scripted -E cam_width=640 -E cam_height=360`.
- `PADS_DEMO=fingers` in front of the scripted command plays a shortcut instead of the demo (a check, not the
  demo): the closed robot fingers rub both fingertip pads with no cloth. It must score 0.
- `TASK_LOT=A` or `TASK_LOT=B` in front pins every trial to one lot (debugging only).
- Runs use `object_jitter=0`. The UR5e and gripper are fixed; only the cloth's place changes between lots.
- The simulation prints one `[pads]` line a second: the force between the cloth and each pad, the cleared cells per
  zone, whether the hands are clear and the cloth's pose.

## Scoring

Every trial is scored by `score()` in `task.py`; `task.json` has the short text. Lengths in mm.

**Whole trial (checked every control step):** 0 for the trial if a robot finger or gripper link touches the palm
pad or a fingertip pad (more than 0.5 N of contact force, read from the simulator's contact report between each pad
and the six hand links), or the cloth falls off the table (any corner more than 10 mm below the table top, or off its
edge). 0 also if the pads' contacts or the hands cannot be read at any step (the scorer fails closed).

**The dust rule:** each pad's face carries cells: the palm 6 x 4 (two zones of 12: left and right halves), each
fingertip 3 x 6 (three zones of 6 along its length: palm end, middle, tip end). Each control step, for each pad
whose contact force with the cloth is at least 0.5 N on this step and the one before, every cell lying under the
cloth (inside the cloth's box grown by 1.5 mm) gains the distance the cloth slid over it along the pad's face since
the last step. Steps under 0.25 mm (slower than 5 mm/s: a press that only jitters) or over 12 mm (a jump) count
nothing. A cell is clean at 20 mm; a zone is clean when at least 90% of its cells are (a palm zone may keep 1 dusty
cell of 12, a fingertip zone none of 6), which is how a real photo through the stencil is read: a cell is dusty if any
chalk shows in it.

**Let go and still:** every score needs both hands' gripper and finger hulls (their faces sampled every 3 mm) at least
30 mm from every collider of the gripper and its coupling and from the cloth, and the cloth at rest (under 10 mm/s),
for 20 control steps (1 s) in a row, and at the end the cloth at least 2 mm from every collider of the gripper.

| Score | Simulation (`task.py`) | Real trial |
|---|---|---|
| 1.0 | all 8 zones clean; the cloth's lowest corner within 3 mm of the mat's top, flat within 10 degrees, all 8 corners inside the mat's outline | after photos: every zone at most 10% chalk; overhead video frame: cloth lying flat inside the mat's edges |
| 0.75 | all 8 zones clean, cloth elsewhere (off the mat, propped, held up) | photos as above; cloth not on the mat |
| 0.5 | 5 to 7 zones clean | photos: 5 to 7 zones |
| 0.25 | 2 to 4 zones clean | photos: 2 to 4 zones |
| 0 | otherwise, or any whole-trial rule broken, or hands within 30 mm or not still at the end | video and photos |

Trials run on lot A, then lot B; `task_score` is their mean.

### What each way of doing it scores (cases in `near_misses.py`)

| Strategy | Score |
|---|---|
| Wipe with a gripper finger instead of the cloth (any finger or gripper contact with a pad) | 0 |
| Move the cloth past the pads without touching them (hover) | 0 |
| Cloth 3 mm off a pad's face while it presses on a neighbouring part | 0 |
| Press the cloth on each pad without sliding it (jabs), or hold it pressed while it jitters 0.1 mm a step | 0 |
| Wipe with under 0.5 N | 0 |
| Wipe slower than 5 mm/s | 0 |
| Wipe the fingertips with the cloth 17 mm low (along one edge only) | 0.25 (palm only) |
| Palm pad only | 0.25 |
| Palm and one fingertip, or both fingertips only | 0.5 |
| All but one zone | 0.5 |
| All zones, cloth left lying in the gripper or hanging on a fingertip | 0 |
| All zones, cloth dropped on the table beside the mat, or propped up | 0.75 |
| All zones, cloth back flat on the mat, hands clear, still 1 s | 1.0 |
| All zones but still holding the cloth, or still for under 1 s | 0 |
| Cloth knocked off the table at any moment | 0 |

## The scripted demo

`demo()` drives one arm, open loop, from what the cameras show: the cloth's place on the mat chooses the arm (left
for lot A, right for lot B) and where to take it; the gripper and its pads are fixed and in view. It never reads the
dust cells, the contact forces or the cloth's simulated pose. The other arm stays parked.

1. Pinch the cloth across its 40 mm width, 18 mm in from its near end, and lift it level.
2. Carry it to the gripper's mouth, its far end 20 mm short of the fingertips.
3. Push its end onto the palm pad (commanded 6 mm past the pad's face: the arm gives way, so this sets a light
   press of a newton or two) and sweep it 8 mm each way, five times, at two heights 5 mm apart.
4. For each fingertip: line the cloth up beside it, push it in until its end is 16 mm short of the palm (deeper, the
   finger's inner link stands in the way), move it sideways onto the pad (commanded 9 mm past the face), then
   slide it out along the finger 20 mm and back in, ten times. Then move it off the pad and out of the gripper.
   The strokes run along the cloth: strokes across it (up and down) turn the strip in the pinch, which grips it only
   across its width.
   A light press and long strokes matter: the arm is compliant, so a hard press makes the cloth stick to the rubber
   and short strokes then only flex the arm.
5. Carry the cloth back to where it was, lower it, let go, lift and move clear.

## Fidelity

| Value | Source |
|---|---|
| 2F-85 opened: 85 mm between pads, 100.6 mm across their outsides, 152.4 mm across the fingers, 149.3 mm from base face to fingertips, base 75 mm across and 36.3 mm tall, palm pad 31 mm wide, fingertip pads 38 mm long and 22 mm wide | Robotiq manual Fig. 5-1 |
| Fingertip rubber face 16.2 mm wide | Robotiq manual Fig. 5-1 (side view) |
| Palm pad 22 mm tall, 2 mm proud; link bars 11 mm wide; chassis sections | estimates from Fig. 5-1 and 5-2 |
| Coupling GRP-ES-CPL-062: 75 mm across, 16.9 mm thick | Robotiq manual Fig. 5-5 |
| UR5e joint centres: d1 162.5, a2 425, a3 392.2, d4 133.3, d5 99.7, d6 99.6 mm; base 149 mm | Universal Robots' published DH parameters and UR5e data sheet |
| UR5e link and joint housing diameters | estimates |
| Cloth: 3M Scotch-Brite 2010 microfibre wipe, 320 x 360 mm, dry use | 3M / RS Components listing |
| Cloth mass 30 g (simulated 35 g), 24 layers about 28 mm thick loosely folded | estimates |
| Cloth friction 0.7 / 0.6, pads 1.0 / 0.9 (static / dynamic) | estimates (dry microfibre on NBR rubber) |
| Dust: a cell clears after 20 mm of cloth sliding, at 5 mm/s or faster, under at least 0.5 N | stated rule, not measured |

Simplified: everything is rigid boxes, cylinders and prisms; the fingers are fixed (the unpowered 2F-85 moves only
through its drive screw, and each finger's sprung adaptive joint is not modelled); the folded cloth is one rigid
block, so it does not drape or wrap; the UR5e is fixed (powered off, brakes on), its wrist 3 joint held by a stiff
spring as its brake; dust is a cell rule, not powder. The layout is fixed per lot: two lots stop a single fixed habit
(always taking the cloth from one side), not a memorised replay of both.

## Real trial

- Hardware: UR5e with the Robotiq 2F-85 (AGC-TIP-MLD-KIT-85 fingertips) on GRP-ES-CPL-062, jogged to the pose in
  `task.json`, fingers commanded fully open, then the robot and gripper switched off (pendant power off, e-stop in).
- Chalk: white marking chalk (a chalk-line refill), 0.05 g per pad weighed on a 0.01 g scale, brushed through the
  stencil's zone windows with a soft brush until each window is evenly white.
- The two arms' finger pads are covered with masking tape, replaced every trial, so chalk does not build up on them;
  each trial uses a fresh folded cloth and the used one is bagged, not shaken out near the rig.
- Photos: a phone square-on to each pad, 10 cm away, with the stencil laid over the pad and its 50 mm scale bar in
  view.
- Safety: the UR5e is off and braked; nothing in the task is sharp, hot or live.

## Results

All runs on one NVIDIA L40S, yam-isaac-lab-arena on Isaac Lab-Arena, `object_jitter=0`, both lots (`--epochs 2`).

| Run | What | Lot A | Lot B | Notes |
|---|---|---|---|---|
| Final demo | `yam_scripted`, 2,400-step limit | 1.0 | 1.0 | all 8 zones (60 cells) clean, cloth back flat on the mat, released and still; each trial done at about 77 s (1,550 steps) |
| Dream-1 | `dream`, 2,400-step limit | 0.0 | 0.0 | both trials ran to the step limit without refusing; the cloth stayed where it lay (moved under 3 mm) and no pad was touched |
| Shortcut check | `PADS_DEMO=fingers`: closed robot fingers rub both fingertip pads, no cloth | 0 | not run | a robot fingertip touched a pad 5 s in (trial ruled out); no dust cell cleared |

How the demo got there (traces, same scene):

- First trace: 0.5 in lot A. The palm and the right fingertip came clean; the left fingertip's palm-end zone kept one
  dusty cell because the cloth's press on that pad (pulling toward the arm's own side) was light and intermittent.
- Second trace: lot A 0.5, lot B 1.0. Up-and-down wiping strokes across a fingertip pad turned the cloth in the
  pinch (friction along the vertical at 80 mm from a grip that holds the strip only across its width), so on the
  far fingertip the cloth's end rode 10 mm low and missed the pad's upper cells.
- The demo was changed to wipe the fingertips only with strokes along the cloth (in and out along the finger), to
  fold the cloth thicker (28 mm, so it covers the 16.2 mm pad face with room for the arm's few millimetres of sag),
  and to squeeze it harder. Third trace: 1.0 in both lots, all 60 cells cleared by about 56 s.

The simulation logs one `[pads]` line a second; in the final demo the cloth-to-pad force while wiping was about 1 to
2 N (peaks near 6 N on contact), no hand ever touched a pad, the cloth ended flat on the mat where it started, and the
hold counted from the release.

## Licences

No third-party meshes or assets: every shape is a box, cylinder or prism written in `task.py` from the makers'
published dimensions. `parts.py` and `lots.py` are helper modules shipped with the task.
