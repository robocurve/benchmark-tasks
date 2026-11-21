# Open the latched lid of a small drinks cooler and leave it open, without shoving the light cooler about

An empty personal drinks cooler stands on a rubber mat with its lid shut. The robot must open it and leave the lid
open. The lid is a swivel lid: it turns on pins through the middles of its two end plates and hangs beside the body
when open, on either long side. A catch holds it shut until the release button on one end plate is pushed in. While
the button is in, the lid can be rolled over; let go of the button with the lid still shut and it stays caught.
Pushing or lifting the lid, or pulling its handle, does nothing while it is caught. The empty cooler weighs 0.69 kg,
so a hard push on the button or the lid slides it on the mat, and a push high up tips it. One hand has to hold the
button in while the other rolls the lid over, low down on its side, without moving the cooler more than 10 mm.

## The cooler

An Igloo Retro Playmate Mini 4 qt cooler, item 00048535 (jade body, white lid). Published by the maker
(https://www.igloocoolers.com/products/retro-playmate-mini-cooler and its product data): exterior
246.2 x 174.8 x 246.2 mm, 680 g listed weight (0.69 kg used here), an "original side-push button" on the lid's end, a
"tent-top" lid, and the lid swivelling open to either side and shut again ("swivel it shut"). The maker's side and
open photos show the button in the middle of the lid's end plate, about 125 mm up, the end plates hanging down over
the body's ends to about 58 mm above the base, and the open lid hanging beside the body with its end plate turned
90 degrees about the button. A third-party summary of the product manual says to press the side button firmly, swivel
the lid to either side, and swivel it back until it clicks.

Everything else is an estimate from the maker's photos, not a measurement of a cooler: the 125 mm pivot height, the
8 mm walls, the lid's 4 mm long sides from 160 to 192 mm up, the tent top's steps, the handle, the button's 18 mm
size, 8 mm proud and 7 mm of travel, and the split of the mass (body 475 g, lid 200 g, button 15 g). No force figures
were found. Stand-ins: the button needs 2.5 N to start moving and 3.6 N fully in (a 150 N/m spring with 16.7 mm of
preload); the lid's pins and seal resist turning with up to 0.025 N m of friction. The internal catch was not
inspected; it is modelled as a rule (below), not as a drawn latch.

## Run it

The task runs in yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) on the `station` scene. It
needs the three helper files next to `task.py`: `parts.py` (multi-shape parts with joints), `lots.py` (layout lots)
and `tilt.py` (used only by the scripted demo, to lean the arms' finger tips; see below).

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Open the cooler and leave its lid open."

# Dream-1, both lots (trial k uses lot A, B for k mod 2 = 0, 1; the mean is task_score)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 2 --store-frames --log-dir ~/yam-eval/cooler-dream

# Scripted demo, both lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 2 --store-frames --log-dir ~/yam-eval/cooler-demo
```

Without `--epochs 2` only lot A runs. `TASK_LOT=A` or `B` in the environment pins every trial to one lot. Run with
`object_jitter=0` (as submitted). One trial is at most 600 control steps (30 s at 20 Hz); a trial ends early once
success has held for 1 s. The scripted demo opens the lid about 13 s into each trial and scored 1.0 in both lots; Dream-1 scored 0.0 in both (each ran the full 30 s).

## Layout

Coordinates: x forward from the line through the arm bases, y to the robot's left, from the midpoint between the
bases; z up from the table top. Part frame of the cooler: origin at the middle of the body's underside, z up, y along
its length (the button's end at +y), x across it.

- Mat: dark rubber, 380 x 340 x 3 mm, centred at x 320, y 0 mm, fixed. Friction 1.0 static, 0.9 sliding; the
  cooler's plastic 0.8 and 0.7.
- Body, jade: 175 mm across, 234 mm long, 160 mm tall, open top, 8 mm walls, a 12 mm floor with a white liner on top.
- Lid, white: two end plates 6 mm thick over the body's ends, each full width from 58 to 160 mm up, then 124 mm
  wide to 205 mm and 50 mm wide to 243 mm; the button's plate has a 24 mm square opening round the pivot. Long sides
  4 mm thick, from 160 to 192 mm up, 87 to 91 mm out from the middle (the body's side is at 87.5 mm). The tent top in
  three steps: 156 mm wide to 206 mm, 120 mm to 220 mm, 80 mm to 232 mm. Carry handle 24 mm wide: a grip 120 mm long
  from 240 to 246 mm up on two 18 mm posts, 156 mm apart overall. Pins through the end plates' middles, 125 mm up.
  Stops at 90 degrees either way, where it hangs beside the body.
- Release button, grey: a cylinder 18 mm across on the pivot, 8 mm proud of the end plate's face at rest, 7 mm of
  travel into the plate.
- Lot A (the screenshot): body centre at x 310, y -20 mm; the button end toward the robot, pointing 12 degrees to the
  robot's left of straight at it. Lot B: body centre at x 330, y 25 mm; pointing 15 degrees to the robot's right.

When a trial starts, after the lot has moved the cooler, the task stands it exactly at its lot's pose and lets it
settle 0.2 s before the first camera frame. As a guard, the first control step stands it again if it is more than
1 mm off.

## The mechanism

- Catch: with the button in less than 5 mm and the lid within 0.5 degrees of shut, the lid's limits are -0.5 to
  +0.5 degrees: it cannot be opened, whatever pushes it. With the button in 5 mm or more, the limits are -90 to +90
  degrees and the lid swings freely. Let go once the lid has left shut, the lid stays free until it comes back within
  0.5 degrees of shut, where it catches again (the maker's "swivel it back until it clicks"). So the button must be
  held in until the lid has started to turn, and no longer.
- Button: a sprung slider (150 N/m, rest point 16.7 mm out beyond its stop): 2.5 N to start it, 3.6 N at full travel.
- Lid friction: the lid's drive pulls it toward a rest point it drags along once it is more than 0.02 rad away, a
  friction of up to 0.025 N m. The lid's shell sits high above its pins, so once the lid is turned about 10 to 20
  degrees its weight beats that friction and it falls over onto its stop on its own; below that it stays where it
  is left. Damping 0.02 N m s/rad.
- The lid and button do not collide with the body they are jointed to (the simulator never collides a jointed link
  with its parent body); the joint limits are the lid's stops. They do collide with each other, the mat and the
  grippers.

## Scoring (per trial; trials are averaged)

`score()` returns 1.0 when all of these hold at the end:

1. Open: the lid turned at least 80 degrees off shut, either way. On a rig: the lid hangs beside the body.
2. Button out: within 1 mm of its rest.
3. In place: the cooler tilted 3 degrees or less, and its body's origin within 10 mm (horizontally) of where its lot
   put it.
4. Released: every point of both grippers' collision hulls (`gripper`, `tip_left`, `tip_right`; corners and face
   points about every 3 mm) more than 5 mm from every shape of the cooler, lid and button included, measured each
   control step.
5. Still: the body under 5 mm/s and 0.05 rad/s, the lid turning under 0.05 rad/s, the button under 5 mm/s.
6. Held: 1 to 5 have held at every control step for the last 2 s.

Otherwise 0. There is no partial credit: let go part way, the lid either stays near shut (under about 10 to 20
degrees) or falls open on its own, so no part-open state survives release; a part-open lid held by a finger is not
released.

Whole-trial rule (zeroes the trial once set; cleared only when the simulator resets the scene for a new trial): the
cooler tilted more than 10 degrees, or its body lifted more than 10 mm, at any control step. On a rig this is judged
from the trial video.

Failing closed: the step callback must find all six hand links (`gripper`, `tip_left` and `tip_right` on both arms);
if any is missing, that step has no release measurement and the 2 s hold restarts. A non-finite cooler state zeroes
the trial. Offline, `near_misses.py` writes into the task's per-step record what the callback would have measured
for a still end state held 2 s with both grippers 50 mm away before the cases are scored.

## Near misses (`near_misses.py`)

Cases: open on its stop (1.0), open the other way (1.0), exactly 80 degrees (1.0), 79 degrees (0), 30 degrees (0),
shut (0), shut with the button held in (0), open with the button still held in 2 mm (0), button in exactly 1 mm
(1.0), slid exactly 10 mm (1.0) and 11 mm (0), tilted 3 degrees (1.0) and 4 degrees (0), on its side (0), still
sliding (0). `_run()` checks the hold (1.95 s: 0; 2.0 s: 1.0; broken part way and restarted: 0), the release distance
(6 mm: 1.0; 4 mm and exactly 5 mm: 0), no gripper measurement (0), no start record (0), an earlier tip followed by a
good finish (0), a non-finite lid angle (0); the release measured from a 60 mm plate's faces set across the body's
corner, 4 mm (not released) and 6 mm (released) away, in both lots, where its own corners are more than 5 mm away;
every transition of the catch; the friction rest point; the whole-trial rule in both lots (tilted exactly 10 degrees:
allowed, 10.5: zero; lifted exactly 10 mm: allowed, 11 mm: zero; slid 50 mm: judged at the end, not by this rule);
and the stillness test (lid turning 0.04 rad/s: still; 0.06 rad/s: not; button moving 6 mm/s: not).

Habits a policy might fall into, and what they score: pushing, lifting or pulling the lid or handle without pressing
the button (the lid does not move; 0); pressing the button alone (the lid stays shut; 0); pressing the button, then
letting go before pushing the lid (it caught again; 0); pushing the lid high up or hard (the cooler slides past
10 mm or tips; 0); propping the lid part way with a finger (not released; 0); tipping the cooler over so the lid
falls open (0, by the whole-trial rule); replaying lot A's moves in lot B (lot B's button centre is about 26 mm from lot A's
and the cooler is turned 27 degrees the other way, so lot A's press lands beside the 18 mm button, on the end plate,
and the lid stays caught; 0).

Offline MuJoCo check of the mechanism with default contacts, force-limited soft hands (not a measurement, not
Isaac): pressing the button alone for 16 s left the lid shut and slid the cooler 0.8 mm; pushing the lid's side with
the button out left it shut (the cooler slid about 7 mm under a 40 N-capped push); holding the button and rolling the
lid 16 to 20 degrees, then letting both go, left it open on its 90-degree stop with the cooler moved 0.4 mm.

## The scripted demo

`demo()` reads only the cooler's start pose for the trial's lot (where it stands and which way its button end points,
as the cameras show), never the lid's angle, the button's travel or any simulator state during the trial.

1. The arm on the side the button end points to (the left in lot A, the right in lot B) leans its closed finger tips
   45 degrees toward the button, along the cooler's length, and presses the button, aimed 26 mm past the button's
   face (the arm gives way; on the GPU the button goes fully in, 7 mm).
2. The other arm, its closed finger tips leaned 30 degrees toward the lid, comes down beside the lid's nearer long
   side and presses it 10 mm above the side's lower edge (170 mm up, contact from about 163 to 190 mm), then rolls
   the lid 16 degrees over, away from itself, in eight steps along the lid's arc.
3. The button arm lets go once the lid is turning and backs off; the pushing arm backs off. The lid falls over to its
   stop on the far side under its own weight. Both arms park, fingers open, well clear.

`tilt.py` leans the finger tips; it patches the stock `yam_arena.demo.grip_rotation` in the task's own process only
and affects only the scripted demo, never Dream-1 or scoring. The two leaned poses were checked offline for reach
(inverse kinematics) and for clearance against the cooler (the arms' collision hulls).

## Licences

No third-party assets: the cooler is built from primitives in `task.py`.
