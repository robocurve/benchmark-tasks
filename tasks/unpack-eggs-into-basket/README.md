# Unpack a snap-shut egg carton into a basket

A closed half-dozen egg carton stands on the table with six eggs in it, and a cloth-lined basket is fixed to the
table on its right. The robot has to open the carton and move all six eggs into the basket without breaking one.

Three things make this harder than moving six objects from one place to another:

- **The lid is snapped shut and the carton is light.** Opening the snap takes a 3 N pull at the lid's tab. Lifting
  that side of the loaded carton off the table takes 1.5 N. A pull on the lid's tab alone therefore lifts the carton
  and the eggs with it, and the lid stays shut. One hand has to hold the base down while the other opens the lid.
  The same happens again at the end: the nearly empty carton weighs less than the pull of an egg that rubs its cup.
- **The eggs stand in tight cups.** Each egg has 1 mm of clearance in its cup and shows only its top 22 mm above the
  cup walls. At the height a gripper can reach, the eggs of the two rows are 18 mm apart, and a finger is 10 mm
  thick. An egg can only be taken by its cap, with one finger between the rows.
- **An egg is round and fragile.** One that is let go standing falls over and rolls. One that falls more than 3 cm
  counts as broken and earns nothing, so the eggs cannot be tipped, poured or dropped into the basket.

## Where this comes up

Emptying a carton of eggs into a bowl or basket is an everyday kitchen job, at home and in food preparation:
unpacking shopping, filling the egg basket on a counter, setting out eggs before baking or a breakfast service. The
same handling (open a light snap-shut package while holding it down, lift fragile round items out of fitted cups,
set them down so they stay put) applies to trays of fruit, ornaments and laboratory vials.

## What it measures

- Using two arms because the physics requires it: holding a light container down against the force that opens it.
- Grasping in clutter: closing on a small exposed cap with a finger in a narrow gap between neighbours.
- Gentle placement of an object that rolls, scored by what a person would check afterwards: is each egg in the
  basket, lying on the cloth, and unbroken.

It differs from egg-packing and egg-holder tasks, where loose eggs are placed into cups: here the eggs start inside
a closed, latched package and have to come out of it.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: carton, eggs, basket, the lid's snap, `success()` and `score()` |
| `eggkit.py` | Loose bodies built from boxes and capsules, with hinged flaps, for yam_arena task files |
| `demo.py` | The scripted two-handed demo and two controls that must not score |
| `test_scoring.py` | Checks of the scoring rule on hand-made states (no simulator needed) |

`eggkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from
this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the egg carton and put all six eggs in the basket, setting each one down gently on the cloth. A dropped egg is a broken egg." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/eggs-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

There are two layouts. The default is layout A. Set `EGGS_LOT=B` before the command for layout B, in which the
carton and the basket stand 1 to 1.5 cm from their places in layout A.

## Scripted demo

```bash
python demo.py task.py demo_out               # full score: writes demo_*_cam.mp4, setup.png, result.json
EGGS_LOT=B python demo.py task.py demo_b      # the same script on layout B
python demo.py task.py one_hand --one-hand    # control: the lid's tab pulled with the base not held
python demo.py task.py drop --drop            # control: each egg let go 6 cm above the cloth
```

1. The left hand, closed, presses its fingertips on the base's thumb tab. It leans 40 degrees out to its own side,
   which keeps its wrist clear of the right arm (the two tabs are 8 cm apart). The right hand pinches the lid's pull
   tab by its two side edges and carries it up and over along the lid's arc. The tab turns freely between the finger
   pads, so the hand stays pointing down. Past the top it lets go and the lid swings open toward the arms.
2. The left hand moves away. The right hand takes the eggs one at a time, starting with the column furthest from the
   basket. Its fingers come down on either side of the egg's cap, one between the two rows and one outside the
   carton, close on the cap and lift the egg clear of its cup. The arm cannot lift an egg over the eggs still
   standing in the carton, so it carries each one out over the open lid, along the front of the carton and into the
   basket.
3. In the basket it lowers the egg until its end touches the cloth, then moves sideways and down along the arc the
   cap follows as the egg rolls onto its side. The egg turns freely between the two finger pads. The hand opens
   only when the egg is lying down.
4. With one column left the carton weighs under 150 g, so the left hand presses on the thumb tab again while the
   right hand takes the last two eggs.

The demo is open loop. Every pose comes from the layout constants in the task file, which are what the first camera
frame shows with `object_jitter=0`. Object states are read only to write the result file.

## Scoring

Tools: a photo of the end state, a look at each egg before and after the trial, and the trial video.

The score is the number of eggs that count, divided by six. An egg counts when all of these hold at the end:

1. **It is unbroken.** In a real trial: no crack or leak on inspection, compared with the inspection before the
   trial. In simulation the eggs are rigid and cannot crack, so an egg is marked broken, for the rest of the trial,
   the first time it hits anything at more than 0.77 m/s, the speed a 3 cm free fall ends at. The hit speed is taken
   every control step from the egg's speed and the height it lost in that step.
2. **It rests on the basket's cloth lining.** Its centre is inside the basket's walls and its lowest point is
   within 4 mm of the lining. An egg lying on top of other eggs or on the rim does not count, and the basket has
   room for all six on the lining. An egg still in a cup of the carton does not count, wherever the carton is.
3. **It is still**: moving slower than 5 cm/s.
4. **It has been let go**: no gripper's grasp point is within 4 cm of the egg's centre.

Success is all six eggs counting. Eggs left in the carton, on the table or held score nothing; there is no credit
for opening the lid alone.

## Verification

| Run | Score | What happened |
|---|---|---|
| Scripted demo, layout A | 1.0 | Success at step 1,865 (93 s). The carton stayed within 1 mm of its place; the hardest knock any egg took was 0.35 m/s. |
| Scripted demo, layout B | 1.0 | Success at step 1,862, same script. Hardest knock 0.53 m/s. |
| Control: lid pulled with the base not held | 0.0 | The lid turned 2.3 degrees and stayed shut. The carton came up with it, tipped over and ended on its side with all six eggs still in it. |
| Dream-1, 2,400 steps | 0.0 | The carton was never opened. The right gripper went into the empty basket and stayed there; the left gripper reached the carton's end near the finish. |

The `--drop` control was stopped after its first egg: let go 6 cm above the cloth, that egg was marked broken on
landing and the score stayed 0.

`python test_scoring.py` checks 17 cases of the scoring rule on hand-made states, among them an egg on top of other
eggs, an egg still held, a broken egg, and the whole carton set down inside the basket.

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`); there is no randomness in the task. On an L40S the 2,400-step Dream-1
trial took 11 minutes including start-up, and a demo run takes about 5 minutes.

## Fidelity

- Each egg is a rigid capsule, 43 mm across, 57 mm long, 58 g: the size and mass of a large hen's egg, but with two
  equal ends. A rigid egg cannot crack, so breakage is judged from impact speed as described above. Grip force is
  not judged in simulation; in a real trial an egg crushed by the gripper fails the inspection.
- The carton is rigid. Its walls are 2.5 mm thick and its cups are square in plan, with a narrower seat at the
  bottom that centres the egg's end. A real moulded carton has thinner, slightly flexible walls and rounded cups.
- The snap is a spring at the hinge that holds the lid shut and lets go for good once the lid has turned 4 degrees,
  which takes 3 N at the pull tab. It does not snap shut again.
- The basket is fixed to the table and its cloth lining is a rigid plate. A real egg sinks a little into cloth and
  stays where it is put; here damping on each egg's spin (12 per second) stands in for that, and an egg laid on
  the plate can still wander a few centimetres before it comes to rest.
- The robot's finger pads have a friction coefficient of 0.7 against eggshell.

## Real-world build

- Carton: a half-dozen (2 x 3) moulded egg carton with a hinged lid and a two-button snap closure that opens at
  about 3 N of pull on the lid's edge, about 146 x 110 x 69 mm closed and about 32 g empty. Fix a 30 x 35 mm pull
  tab to the lid's free edge and a 30 x 25 mm thumb tab, propped on the table by a rib, to the base's free edge,
  8 cm apart.
- Eggs: six large hen's eggs, 56 to 63 g each, inspected for cracks before the trial.
- Basket: a shallow basket or tray, about 20 x 25 cm inside with a 4.5 cm rim, lined with a folded tea towel and
  held in place with tape or a non-slip mat.
- Place the carton and the basket on the marks given in `task.json` for the layout being run, carton hinge toward
  the robot.

## Third-party assets

None. Every object is built from boxes and capsules in `task.py`.
