# Herd three cans into a bay with a seine rope

A beach-seine mock-up. Three loose cans (5 cm across, 6 cm tall, 60 g, smooth) stand in a row across the table 30 cm
in front of the robot, 12 cm apart. A free blue 8 mm braided rope (40 cm, black end caps) lies straight across the
table behind them, one cap in front of each arm. A green landing bay, 15 cm wide and 11 cm deep, is marked at the front
centre. The robot must haul all three cans into the bay with the rope alone: never touching a can with a gripper,
never tipping one, never lifting the rope over a can, and letting go of the rope at the end.

The bay is about half as wide as the row of cans (29 cm across the outer edges), so pulling both ends straight back
sweeps the cans in a line and leaves the outer two outside it. The two wings have to be brought in and towards each
other together (pursed), so the bight wraps the cans into a cluster and herds it into the bay. This follows how a beach seine is hauled: "The drag lines are towed simultaneously from the beach. The fish is herded in
front of the bag or bunt." (FAO/FIIT Gear Type Fact-Sheet: Beach seines, 2001; see also FAO Fisheries and Aquaculture
Technical Paper 562, "Fishing with beach seines": "The long hauling ropes and the wings of the seine herd fish into the
centre part of the seine body.") The occupation link is O*NET 45-3031 (Fishing and Hunting Workers): "Put fishing
equipment into the water and anchor or tow equipment, according to the fishing method used."

Both arms are needed: at the start each arm reaches only its own side of the table (offline IK at grasp height, 40 cm
out: the left arm reaches y down to about -3 cm, the right cap lies at -21 cm, about 18-19 cm past that; the right arm is
the mirror), and the purse closes only if both wings come in at matched rates.

Instruction: `Haul all three cans into the green bay using only the blue rope, the way a beach seine is hauled. Never touch a can with a gripper, never tip one over and never lift the rope over a can. Let go of the rope when you are done.`

## How this differs from similar tasks

Other tabletop tasks gather loose objects with a rigid tool (a brush sweeping cubes into a dustpan), surround a fixed
zone with a flexible barrier, or pull a single body attached to a tether. Here the rope is attached to nothing and the
objects are not pushed by a rigid edge: the ability tested is closing a deformable rope's bight around several free
objects with two arms in step, so the rope's own shape gathers them. A rigid sweep or a straight two-handed pull
does not work, because the target is narrower than the row of objects.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated rope) is in this folder next to `task.py`; `cabledemo.py` runs the scripted demo.
  `task.py` gives the cans friction 0.5 and turns on PhysX contact reporting between the robot links and the cans.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Haul all three cans into the green bay using only the blue rope, the way a beach seine is hauled. Never touch a can with a gripper, never tip one over and never lift the rope over a can. Let go of the rope when you are done."
```

Run one trial per process (or one per embodiment reset): the scorer keeps a short history of the trial (touches,
tipping, rope over a can), which `task.py` clears on every embodiment reset.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Both arms take their rope caps (fingers across the cap, wrist cameras facing the middle of the table, where they see the rope running in from the cap). They drag the caps in
together in four 3 s strokes, forward and towards the middle: (32, ±22), (24, ±17), (16, ±12), (10, ±8.5) cm. The
wings close round the outer cans and squeeze the row into a cluster, which the bight herds towards the bay. Then both
arms draw the caps apart along the bay's near edge in 8 mm steps, which hauls the bight and the cluster further in.
This is closed loop: they stop as soon as all three cans are inside the bay. Then they open, rise clear of the rope
and park. It reaches `task_score` 1.0 in 502 steps (about 25 s) of the 1,200-step limit; videos are in `demo/`.

Shortcut variants (`SEINE_CHEAT=<name>`, test only), each scoring exactly 0:

| variant | what it does | why it scores 0 |
|---|---|---|
| `none` | does nothing | no can in the bay |
| `hold` | does the full haul, keeps holding the caps | grippers still on the rope (release rule) |
| `straight` | pulls both ends straight back without closing the wings | the cans come in as a row (outer ones at y ±7.5 cm, outside the bay); the taut rope then tips a can (38 deg) and rides over it |
| `push` | sweeps the cans into the bay with the closed grippers | robot touched a can |
| `onearm` | hauls with the left end only | the left can moves only to x 25 cm, the other two stay where they started; no can reaches the bay |

## Scoring (`score()`; `success()` = score 1.0 with all cans at rest, held 1 s)

- **Score 0 (latched for the trial):** any robot link touches a can (PhysX contact, read every physics substep); any
  can tilts more than 30 deg; any point of the rope or a cap comes over a can in top view (within 2.2 cm of its axis,
  i.e. inside its wall rather than beside it).
- **Score 0 at the end:** a gripper TCP is within 5 cm of the rope or a cap (not released).
- Otherwise **1.0** if all three cans have their whole base inside the bay, **0.5** if exactly two do, else **0**.

Real-world measurement: a top photo for the bay (each can's base fully inside the green rectangle), video for touches,
tipping and the rope passing over a can.

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz). A trial takes a few minutes wall-clock on one GPU (the first start of Isaac Sim
adds 1–2 min). The layout is fixed (`object_jitter=0`; the rope is never jittered), so every seed starts from the same
state.
