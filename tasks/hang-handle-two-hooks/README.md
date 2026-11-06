# Hang a long handle across two wall hooks

A 320 mm square orange handle lies on the table pointing away from the arms. A wall rack stands at the far side
in front of the right arm: a back board with two J-hooks 200 mm apart, each a flat arm with an upturned lip. The
handle has to end up resting in **both** hooks at once, between each lip and the board, like a broom on a rack.
A tilted placement that lands one end only earns partial credit.

> Hang the orange handle on the wall rack so that it rests in both hooks at once.

## What makes this task distinctive

The handle starts pointing away from the arms, so it has to be turned a quarter turn before it can lie along the
rack. Each hook is a slot closed by a raised lip in front and the board behind, so the handle has to come down
between them rather than onto an open support: resting on the lip tops scores 0, and one seated end scores 0.3.
The two hooks are 20 cm apart next to a wall, and the scripted demo seats the handle with one arm.

What it tests: a quarter turn (the handle starts along x and must end along y, along the board), a carry above
the 15 mm lips, and seating it in two separate slots next to a wall (the demo lowers it level and slowly into both
at once).

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `score`, `distance`, scripted `demo` |
| `task.json` | the submission fields (`media` is added by the upload script) |
| `test_task.py` | offline checks (NumPy and pytest, no Isaac Sim): 52 tests |

## Run a trial

From a checkout of the arena (yam-isaac-lab-arena, tested at `bd62771`) on the GPU VM, with this folder at
`tasks/hang-handle-two-hooks/`. The scorer does not depend on the start position, so seeded jitter is allowed for
policy runs; the scripted demo is open loop and needs the fixed layout (`-E object_jitter=0`).

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/hang-handle-two-hooks/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the orange handle on the wall rack so that it rests in both hooks at once." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hang-handle-two-hooks-dream
```

Scripted demo (right arm only, 18.6 s = 372 steps of waypoints ending in a 2 s still hold; the plan plus about
100 steps):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/hang-handle-two-hooks/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the orange handle on the wall rack so that it rests in both hooks at once." \
  --scorer task_score --max-steps 480 --store-frames --log-dir ~/yam-eval/hang-handle-two-hooks-demo
```

Videos, one MP4 per camera, in each log's `frames/` folder:

```bash
for log in ~/yam-eval/hang-handle-two-hooks-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`: 1.0 when the trial ended in success, otherwise `score()` at the last step (1.0, 0.3
or 0). Put it in `task.json` `scores`, e.g. `"Dream-1 run: 0.3; demo: 1.0"`.

Runtime per trial: 2,400 steps (2 minutes simulated) for a policy and 480 steps (24 s) for the demo, plus Isaac
start-up. Wall clock on an RTX 5090, start-up included: 35 s for the demo, 203 s for the Dream-1 run. Seeds: none matter for the demo (`object_jitter=0`); for policy runs record the seed Inspect Robots
prints. Never run two Isaac instances on one GPU.

Offline tests (about 40 s):

```bash
YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest tasks/hang-handle-two-hooks/test_task.py -q
```

## Scoring in one paragraph

At each hook the handle **rests in it** when (1) its centre-line, where it crosses that hook, lies between the
lip's inner face and the board's face; (2) its underside there is within 5 mm of the hook arm's top; and (3) its
end beyond that hook reaches at least 30 mm past the hook's outer edge. It must also lie across the rack (long
axis within 60 deg of y) and be still. Score 1.0 for both hooks, 0.3 for one, 0 otherwise. Success is 1.0 held
for 1 s. Constants: `SEAT_TOL`, `OVERHANG`, `LIP_IN_X`, `WALL_X`, `ARM_TOP` in `task.py`.

- **The underside test is what separates seated from propped.** Resting on a lip top puts the underside 15 mm
  above the arm, three times the 5 mm tolerance. A handle with one end seated and the other on the other lip
  (tilted about 4 deg) earns 0.3, and on both lip tops it earns 0.
- **Either end first, any roll.** The handle looks the same end for end; a 270 deg turn hangs it as well as a 90
  deg one. The underside is computed from the full cross-section, so a bar rolled 45 deg about its own axis and
  sitting on an edge still counts.
- **Skew is allowed by the slot, not by the scorer.** A handle a few degrees off y still sits in both 87 mm slots
  (tested at 86 and 94 deg); the slot itself limits skew to about 18 deg.

## Outcomes and shortcuts (all tested)

| State | Score | Why |
|---|---|---|
| Resting in both hooks, anywhere across the slot, rolled, or 4 deg skewed | 1.0 | both hooks pass all three checks |
| One end seated, the other on top of the other hook's lip | 0.3 | underside 15 mm high at the second hook |
| One end seated, the other sloping down to the table in front of the other lip | 0.3 | centre-line in front of the lip at the second hook |
| Hung level but slid along the hooks so one end reaches only 20 mm past a hook | 0.3 | that hook fails the 30 mm overhang |
| Lying on both lip tops ("on the hook tops") | 0 | underside 15 mm high and centre-line over the lip, at both hooks |
| Leaning against the board above the hooks, along it or pointing at it | 0 | too high at one hook, under the other, or not across the rack |
| Lying on the board's top edge | 0 | centre-line behind the board face |
| On the table in front of the rack, or under the hooks inside the slots | 0 | underside 80 mm below the hook arms |
| Held 3 cm above the hooks, or still moving | 0 | not on the arms / not still |
| Pointing at the board, or more than 60 deg off y | 0 | does not lie across the rack |
| Doing nothing, or dropped off the table | 0 | — |

A random sweep of 2,000 poses near and away from the goal returns only 0, 0.3 or 1.0, and success exactly when
the score is 1.0.

## Design numbers

- **Gripper.** At `FINGER_OPEN` (-0.04695 m per finger) the pads are 94 mm apart (arena `constants.py`); the
  usable grip is taken as 84 mm (1 cm margin). The handle is gripped across its 22 mm width, 62 mm under that.
  The demo opens only to 42 mm on the way down (10 mm per side) to keep the finger travel, and with it the
  off-centre grip, small (the GPU rounds measured under 1 mm on 16-25 mm bars), and backs each pad off only 4 mm to
  let go, so the outer finger stays 11 mm off the board.
- **Finger and wrist geometry (from the URDF hulls, straight-down grip, fingers on a 22 mm bar).** Pads reach
  21 mm from the TCP along the closing direction for the lowest 43 mm, the upper fingers 35 mm, and the gripper
  body 51 mm from 70 mm above the TCP upward. This sets the slot and the board height:
  - board-side gap 35 mm (bar face to board), so the upper finger clears the board while the pads are beside it;
  - lip-side gap 30 mm, so the slot is 22 + 35 + 30 = **87 mm** (the original idea had a 40 mm hook arm, which
    leaves no room for a finger or a 12-20 mm grip offset);
  - board 145 mm tall (65 mm above the hook arms, 50 mm above the lips), so the gripper body passes over its top
    edge when the pads are down in the slot.
  The offline sweep of the arm's collision hulls over the whole plan finds the closest approach to any rack part
  is **11.1 mm** (the right finger tip to the board, at the release). The lowest link is 4.2 mm above the table
  (pad tips at the pick).
- **Hook height and joint4.** The hooks are kept at the original idea's 80 mm. The handle is carried with its
  underside 15 mm above the lip tops (95 mm), so the TCP carries at 127 mm above the table, under the ~140 mm
  where joint4 runs out. Joint4 keeps at least 0.21 rad off its limit over the whole plan.
- **Reach and the start position.** The original idea did not fix a start; the first try (centre at x 0.30, y +0.06)
  ran joint4 into its limit at the carry height (10.8 mm IK error). The handle now starts at x 0.24, y +0.04,
  where the carry height solves with 0.21 rad to spare. The rack is centred at y -0.20 in front of the right arm;
  the hung handle's middle (the grip point) is at x 0.30, y -0.20. IK over the whole plan at 20 Hz: worst 0.080 mm /
  0.015 deg, every joint inside its limits.
- **One arm, at the middle.** A 100 g bar held at its middle balances in the pads; the grip point sits between the
  hooks, where nothing is beside the fingers but the board; and the quarter turn is one wrist rotation. Two arms
  at the ends would have to swing round each other through the turn (the ends start 32 cm apart along x and finish
  32 cm apart along y), and the left arm cannot reach in front of the right one where the outer hook is. The left
  arm stays in its ready pose (28 cm from the rack).
- **The quarter turn and turn loss.** Picked with the fingers closing along y (yaw 0), turned +90 deg
  (anticlockwise from above) while lifted over the start, so the ends sweep away from the board, then carried to the
  rack. About 4 deg of turn loss was measured on the GPU; at the hooks, 100 mm from the middle, 5 deg moves the
  centre-line 8.7 mm, well inside both slot gaps.
- **Lowering.** 27 mm into the slots in 2.0 s (1.35 cm/s, under the 3.5 cm/s rule), released 3 mm above the hook
  arms, then the pads rise 3 cm before any sideways move, back away to x 0.20, and park at (0.14, -0.42, 0.14 above
  the table).
- **Carried-handle clearance.** The test sweeps the handle in the grip through the lift, turn, carry and lowering
  with the grip centred or 15 mm off along the closing direction, and turned 0 or ±5 deg in the pads. Closest to
  the rack: 3.0 mm (the end of the lowering, 3 mm above the hook arms, as planned); sideways in the slots at least
  5.3 mm to a lip and 6.1 mm to the board in the worst combination (30 and 35 mm when centred and square).
- **Final-state model.** Every combination of ±15 mm grip offset, ±5 mm wrist error in x and y at the grip and at
  the release, and ±5 deg turn loss leaves the handle resting in both hooks (score 1.0).
- **Top camera.** Projecting the arm hulls into the calibrated 640 x 360 top camera: in the first frame (both arms
  in the ready pose) and the last (right arm parked), no surface point of the handle or of either hook is behind an
  arm, and all of them are in the image. The hung handle spans about 130 x 12 px at 640 x 360 (about 65 x 6 px at
  320 x 176). The 15 mm height difference between seated and propped on a lip is hard to see from above (not
  measured); the wrist cameras show it better, and the scorer does not need the camera.

## How it differs from the nearest tasks (novelty)

- **Board B10, "Hang mugs on a mug tree".** Each mug hangs by its handle on one peg: a single-point hang, repeated.
  Here one rigid item must be supported at two separate points at once, so the handle has to end turned along the
  rack and resting in both hooks; seating one end only is the characteristic failure.
- **Board B18, "Hook a carabiner onto a rod".** Threading a gate or hook over a single rod. Here the item is plain
  and the rack does the hooking; the difficulty is two-point seating next to a wall, not a latch.
- **robosuite `tool_hang`** (not one of the six named sets). Insert a stand into a base,
  then hang a tool by a hole on a single hook. Single support point, compound objects.
- **LIBERO `put_the_wine_bottle_on_the_rack`.** Places a bottle into a rack slot; one support region, no
  two-point constraint.
- **BEHAVIOR `hang_a_bike_on_the_wall`, `hanging_pictures` and similar.** Symbolic `attached` goals; no geometric
  two-hook seating, no partial credit for one end.
- **Meta-World `pick-place-wall`.** Place a puck over a low wall: a wall as an obstacle, not as the far side of a
  slot.

What is new: a long item seated in two separate, fixed J-hooks at once, with a lip in front and a wall behind, and
scoring that separates "both hooks", "one end only" and "propped on the lips". The source idea came from robot
datasets with real hanging tasks (hang a hammer on a wall rail, hang tongs on a rack).

## Assets and licences

None. Every object is a primitive box defined in `task.py`; the room and robot come from the arena.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 35 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 203 s wall clock.
- Offline tests: 52 passed.
