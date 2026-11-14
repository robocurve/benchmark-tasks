# Mount a boring bar in a lathe tool holder and clamp it with its two set screws

A bench lathe stand-in stands on the table, real size: a steel tool holder with a 12.4 mm through-bore and two set
screws on its top face, and, 200 mm toward the chuck, a steel workpiece with a real 22 mm blind bore 40 mm deep. A 12 mm
round boring bar (150 mm, 130 g, a flat along its shank, a cutting insert at one end) lies on a rack in front of the
holder, and a 3 mm hex T-handle key stands in a block. The instruction is one line: "Mount the boring bar in its holder,
ready to bore this hole to full depth." What the robot has to work out is how the bar goes in (plain end first, cutting
end toward the chuck, flat up so the screws bear on it, insert toward the operator's side), how far it sticks out, and
that both screws must be tightened with the key.

This is a real-size rebuild of an earlier, rejected task (a loose square bar in an open channel). The review said the bore
was a mark, there was no holder or clamp, and the instruction gave away the orientation and the target band. Here the
bore is a bore, the bar slides in it with real collision, the clamp is two set screws turned with a real key, the
instruction names no number and no orientation, and the score reads the consequence (clamped, the right way round, the
right overhang).

## Why it is hard

- **The overhang is the decision.** The bar must reach the bottom of the 40 mm hole with the holder 5 mm clear of the
  workpiece (at least 45 mm out) and, for a steel bar, stick out no more than four times its 12 mm diameter (48 mm).
  Neither number is in the instruction; the setup gives the bore depth and the bar size.
- **Orientation.** Plain end first, cutting end toward the chuck, flat up, insert toward the operator's side. The insert is
  90 degrees round from the flat, so one wrong roll fails both. A bar clamped the wrong way round scores 0 for the trial.
- **A tool, not a push.** The key's 3 mm tip must find a 5.2 mm socket, go in 7 mm, stand within 6 degrees of vertical and
  be turned clockwise. A wrist turn is under 200 degrees, a screw needs about 1.4 turns to go tight on the flat, so the
  key is turned, lifted out, turned back and dropped in again, several times per screw.
- **Two screws, a clean finish.** Both screws tight, the key out of the sockets, both grippers 3 cm clear, the bar still
  for a second.

## The parts (millimetres; the layout is in `layout.py`)

| Part | Size |
|---|---|
| Holder (fixed) | steel block 50 x 60 x 70 tall, y -20 to 40; 12.4 mm bore on the axis, chuck-side mouth chamfered in two 1.5 mm steps; two 5.2 mm square key sockets with 10 mm mouths, 15 mm deep, at y -5 and y 25 |
| Workpiece and chuck (fixed) | 50 mm round with a 22 mm bore 40 mm deep, face at y 240; chuck 80 mm across |
| Rack (fixed) | two blades, y 70 to 150, holding the bar's axis at the lathe's height, 45 mm |
| Bar (loose) | 12 mm shank, 150 mm, 130 g, a 0.8 mm deep flat along the top, a 5 x 3 x 4 mm gold insert flush with one end and 1 mm proud of the side |
| Key (loose) | 3 mm hex T-handle, 60 g: 60 mm shaft, 8 mm crossbar 90 mm long |
| Key stand (fixed) | 30 x 30 x 28 mm block, 3.8 mm hole |

Sources: a 12 mm boring bar shank and Weldon-style flat, a 12.4 mm holder bore, M6 x 1.0 set screws and a 3 mm hex key
are ordinary stock parts; the four-times-diameter limit for a steel bar is the usual machinists' rule of thumb (tool
makers' application guides give 4 x D for steel, 7 x D for carbide). Every other size is our estimate of a bench lathe
set-up, not measured on a machine.

## The set screws (the one software model)

PhysX cannot cut a thread, so `setscrew.py` is our rule, run every control step by `task.py` (a mechkit `on_step`
callback). A screw is turned while the key is **engaged**: its tip within 1.5 mm of the socket's axis (x and y), its shaft
within 6 degrees of vertical, the tip at least 7 mm under the top face. The screw turns with the key's heading about the
vertical, clockwise in, 1.0 mm a turn, never above its start. A screw meets the bar after 0.2 mm of travel for a bar on
its round and 1.0 mm for a bar with its flat up within 10 degrees (the flat is 0.8 mm lower); with no bar under it a screw
stops at 0.2 mm, so it never reaches into the bore. It is **tight** 0.4 turn past contact, and stays tight down to 0.2
turn past; it stops 0.5 mm past contact. The first tight screw fires a mechkit weld (the bar's pose relative to the
holder is captured and rewritten every step); when no screw is tight the weld is released, so a bar clamped at the wrong
stick-out can be freed and reset. The record kept when the weld fires: the flat's angle from vertical (above 10 degrees
the bar is clamped on the round, a bad clamp), the stick-out, and whether the cutting end points away from the chuck.
Real set screws take several N m; the model needs none. The bar and key are real rigid bodies in real collision with the
bore, the sockets, the rack and each other; the bore and the round shank are built from 20 and 9 flat boxes.

## Scoring

See `scoring` in `task.json`; the code is `scoring.py`. The scorer reads the bar's pose, the key's pose and the screw
model's state once per control step.

| Outcome | Score |
|---|---|
| Both screws tight, cutting end toward the chuck, flat up (10 degrees), insert toward the operator's side (15 degrees), 45 <= s <= 48 mm, key in neither socket, both grippers 3 cm clear of the bar, bar still for the last 1 s | 1.0 |
| Both screws tight and the cutting end toward the chuck, any of that missing (s outside 45 to 48 mm: too short to reach or past four diameters; flat not up; key still in a socket; a gripper near; still moving) | 0.5 |
| Bar through the bore, cutting end toward the chuck, the screws not both tight | 0.25 |
| Anything else; bar clamped cutting end away from the chuck; bar or key 5 cm or more below the table top (these two latch) | 0 |

Doing nothing scores 0. A step whose bar, key or arms cannot be read counts against every tier and restarts the 1 s window.

## Results in simulation

L40S, Isaac Lab-Arena, `-E object_jitter=0 -E control_hz=20`, one trial each, run limit 2,400 steps (2 minutes).
Offline predictions (`check_task.py`, kinematics and the rules, no physics) are in the last column; they are the
design's claims, not measurements.

| Run | What it does | Score | Offline |
|---|---|---|---|
| Nothing (`DEMO_HOLD_ONLY=1`) | both arms hold still | 0.0 | 0.0 |
| Scripted demo | both arms, the phases below | 1.0 | 1.0 |
| Right arm alone (`DEMO_IDLE_ARM=left`) | cannot reach the bar, so only the key moves | 0.0 | 0.0 |
| Left arm alone (`DEMO_IDLE_ARM=right`) | pushes the bar in and lets go; no key | 0.25 | 0.25 |
| Dream-1 | the instruction, full run limit | 0.0 | |
| `DEMO_VARIANT=too_long` | the bar pushed to a 60 mm stick-out | 0.5 | 0.5 |
| `DEMO_VARIANT=flat_down` | the bar built with its flat down (and its insert on the far side) | 0.5 | 0.5 |
| `DEMO_VARIANT=one_screw` | the demo without screw 2 | 0.25 | 0.25 |
| `DEMO_VARIANT=smoke` | bar in, screw 1 tight (the weld fires), key lifted clear and held | 0.25 | 0.25 |

The task does not claim two arms are required, and the one-arm rows above are the check. Kinematics only (not run): the
right arm cannot reach the bar where it starts (y 185); the left arm reaches the bar, both sockets and the key stand, so
a single left-arm run (bar, then key) may well work; the demo uses two arms so the right can fetch the key while the left
pushes the bar. The offline column is the design's claim, not a measurement.

## Demo

Open loop: timed waypoints from the known start layout, nothing read from the simulator. 101 s (2,021 steps).

| Time (s) | Left gripper (the bar) | Right gripper (the key) |
|---|---|---|
| 0 to 9 | over the bar's cutting end, close the two pads into one block, down behind the end face, up against it | over the key's crossbar, down, close (3 mm commanded on the 8 mm crossbar) |
| 9 to 18 | push the bar along the rack, plain end first, through the bore to a 46.5 mm stick-out (the pad block stays 46 mm clear of the face) | lift the key out of its hole and wait |
| 18 to 24 | back off (the bar rests in the bore: its middle is over the holder), up, home | wait |
| 24 to 34 | home | carry the key over screw 1, press the tip into the socket and search a 2.5 mm spiral for it |
| 34 to 57 | home | six ratchet cycles on screw 1: turn the wrist 150 degrees clockwise (1.8 s), lift out, turn back, drop in; the screw goes tight and the weld fires |
| 57 to 88 | home | the same (with its own socket search) on screw 2 |
| 88 to 101 | home | carry the key back over its stand, lower it into its hole, let go, up, home |

The arms are never close together: two straight-down grippers 70 mm apart have their wrists meet (checked: link
capsules overlap 58 mm), which is why the right waits until the left is home. Six cycles of 150 degrees turn the
screw 900 degrees; it needs about 504 on the flat (576 if the bar sits 0.2 mm low in the bore), and the model stops
turning at tight, so the margin costs time only.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
  -E control_hz=20 --instruction "Mount the boring bar in its holder, ready to bore this hole to full depth." \
  --scorer task_score --max-steps 2400 --store-frames
```

For Dream-1 use `--policy dream`. `DEMO_IDLE_ARM` (left or right) drops one arm's plan, `DEMO_VARIANT` runs a
counterfactual, `DEMO_HOLD_ONLY=1` plays no plan, `TASK_TRACE=trace.jsonl` writes one line per control step (stick-out,
flat and insert angles, screw turns, tight and engaged flags, weld, gripper gap, tier).

## Simplifications

- The screws, as above. The weld rewrites the bar's pose each step, which holds a bar that rests in the bore.
- Boxes and cylinders only: the bore and the shank are 20 and 9 flat boxes (the shank is within 0.11 mm of round; the bar
  has 0.2 mm of radial clearance in the 12.4 mm bore), the chamfer is two steps, the key's hex is a 3.2 mm cylinder.
- Fixed: holder, workpiece, chuck, rack, key stand. No spindle, slide or tailstock; the workpiece's face is 200 mm from the
  holder's so the bar can be set from the chuck side (the stick-out rule is the real one: reach the bottom of the hole with
  the holder 5 mm clear).
- The insert is a gold block on the bar's end, counted as steel in the mass.
- Friction 0.3 on the bar, 0.4 on the key; the simulator's default elsewhere.

## Real build and reset

A steel block with a 12.4 mm reamed bore and two M6 tapped holes on its top face, a 12 mm steel boring bar blank 150 mm
long with a 0.8 mm flat ground along its top and a dummy insert, a 3 mm T-handle hex key, a 50 mm steel round in a chuck,
a rack. Score with a steel rule from the holder's face to the bar's tip, a torque-free feel for the clamp (the bar does not
move under a 20 N push along the bore), and a top and a side video.

Reset: back both screws out, pull the bar and the key; lay the bar on the rack flat up, cutting end toward the chuck, plain
end at y 55, insert toward the operator's side; stand the key in its stand.

## Files

| File | What it is |
|---|---|
| `task.json`, `task.py` | The task text; the objects, the screw rule, `success()`, `score()`, `demo()` |
| `layout.py`, `setscrew.py`, `scoring.py` | Every dimension and shape; the thread model; the scorer (plain NumPy) |
| `mechkit.py`, `rigstate.py` | Build the bar and key as loose bodies and the holder as fixed colliders; read both grippers each step |
| `offline_boring.py`, `tests/` | The bar and key without a simulator; offline tests: `python3 -m unittest discover -s tests` |
