# Put a straw in each of five drinks from a lift-handle dispenser

A countertop straw dispenser stands on the table with five rocks glasses of drink to its right. The robot has to
take straws from the dispenser and leave one standing in each glass.

Three things make this harder than moving five small objects:

- **The dispenser gives up a straw only while its handle is held up.** The straws lie side by side on a sloped floor
  behind the front wall, the front one 3 mm from the wall with the next straw against its back, so no gripper can
  get around it. Raising the red handle at the dispenser's left end lifts the front straw 3 cm, clear of the walls
  and of the straws behind. The lifter weighs 40 g and falls back as soon as the handle is let go, taking the straw
  down with it. One hand has to hold the handle up while the other takes the straw.
- **A straw is 8 mm across and has to be taken at one spot.** The lifter's seat is cut away for two fingers over
  3.8 cm of its length, near the straw's left end, and the straw rolls if it is nudged before it is pinched.
- **A straw only stays in a glass if it goes in at a slant, deep enough.** A 146 mm straw in an 85 mm glass rests
  leaning on the rim. Let go with less than half its length inside, it tips out. The opening is 78 mm, the glasses
  stand 6 mm apart and are loose, and a glass that is knocked over is a spilled drink.

## Where this comes up

Putting a straw in each drink is part of finishing nearly every cold drink served over a bar or a counter.
Bartenders (O*NET 35-3011.00, about 0.7 million US workers), waiters and waitresses (35-3031.00, about 2.2
million) and fast food and counter workers (35-3023.00, about 3.7 million) do it many times a shift, usually with
a round of several drinks standing together and a dispenser that hands out one straw at a time for hygiene.

## What it measures

- Using two arms because the mechanism requires it: one hand holds a sprung-back handle while the other takes what
  it presents, five times over, in step.
- Pinching a thin rod at a set place and re-orienting it in the hand's grasp by turning the wrist.
- Placing a long object into a container so that it stays, next to other loose containers, without touching them.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: dispenser, lifter, straws, glasses, `success()` and `score()` |
| `strawkit.py` | Parts built from several boxes and capsules (loose, fixed, or sliding on a rail) for yam_arena task files |
| `demo.py` | The scripted two-handed demo, and one control |
| `test_scoring.py` | Checks of the scoring rule on hand-made states (no simulator needed) |

`strawkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from
this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put one straw from the dispenser in each of the five glasses." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/straws-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

There are two layouts. The default is layout A. Set `STRAWS_LOT=B` before the command for layout B, in which the
dispenser and the glasses stand about 1 cm from their places in layout A.

## Scripted demo

```bash
python demo.py task.py demo_out             # full score: writes demo_*_cam.mp4, setup.png, result.json
STRAWS_LOT=B python demo.py task.py demo_b  # the same script on layout B
python demo.py task.py let_go --let-go      # control: the handle is raised and let go before the straw is taken
```

1. The left hand pinches the lifter's red handle. It leans 40 degrees out to its own side, which keeps its wrist
   clear of the right arm: the handle is 9 cm from where the right hand works.
2. For each glass, the left hand raises the handle 3 cm and holds it. The right hand comes straight down on the
   lifted straw, 2.9 cm from its left end, pinches it and lifts it out. The left hand lowers the handle and the next
   straw rolls onto the lifter.
3. The right hand carries the straw 4.5 cm out in front of the dispenser and there, over open table, turns its wrist
   so the straw slants 45 degrees with its free end down. It carries the straw to the glass with that end just above
   the rim and slides it down along its own length until 8 cm of it is inside. It opens, and the straw tips onto the
   rim and comes to rest leaning in the glass.
4. The glasses are filled left column first, so the hand never works over a straw already standing in a glass.

The demo is open loop. Every pose comes from the layout constants in the task file, which are what the first camera
frame shows with `object_jitter=0`. Object states are read only to write the result file.

## Scoring

Tools: a photo of the end state and the trial video.

The score is the number of glasses served, divided by five. A glass is served when all of these hold at the end:

1. **It stands upright**, within 10 degrees, and was never tipped more than 30 degrees during the trial. A glass
   that was knocked over has spilled and does not count even if it is stood up again.
2. **Exactly one straw stands in it.** A straw stands in a glass when its lower end is inside the glass and at least
   3 cm below the rim. A glass with two straws does not count, and neither does a straw lying across a rim.
3. **That straw is still**: moving slower than 5 cm/s.
4. **That straw has been let go**: no gripper's grasp point is within 3.5 cm of it.

So each glass is worth 0.2, and success is all five. Straws left in the dispenser, on the table or in a hand score
nothing. There are six straws for five glasses, so one may be lost.

## Verification

| Run | Score | What happened |
|---|---|---|
| Scripted demo, layout A | 1.0 | Success at step 1,306 (65 s). One straw leaning in each glass at 49 to 51 degrees; no glass moved; the spare straw still on the lifter. |
| Dream-1, 2,400 steps | 0.0 | The handle was never raised. The left gripper hovered over the dispenser's open top and the right gripper beside the glasses; no straw was taken. |

Two earlier demo runs scored 0.8 and led to changes that are now in the task. In the first, the lifter carried the
next straw up with it and that straw rolled out of the dispenser; the lifter's seat was reshaped so the next straw
rests on the sloped floor, never on the lifter. In the second, the straw's free end dipped into the dispenser while
the wrist turned; the demo now makes the turn over open table.

Layout B and the `--let-go` control were checked against the arm's kinematics (reach, joint limits and clearance
between the arms) but not recorded in the simulator.

`python test_scoring.py` checks 16 cases of the scoring rule on hand-made states, among them two straws in one
glass, a straw lying across a rim, a straw still held, and a glass that was tipped over and stood up again.

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`); there is no randomness in the task. On an L40S the 2,400-step Dream-1
trial took 7 minutes including start-up, and a demo run takes about 4 minutes.

## Fidelity

- Each straw is a rigid capsule, 146 mm long, 8 mm across, 1.0 g: a short wide paper straw. A real straw bends a
  little under a pinch; this one does not.
- The dispenser is fixed to the table and built from flat plates. The lifter slides straight up on a rail and
  returns under its own weight, with a little damping. A real lift-handle dispenser has a hopper that holds a
  hundred straws or more; this one holds six in a single layer.
- Each glass is eight-sided rather than round, 84 mm across, and weighs 330 g, as a rocks glass with about 200 ml of
  drink does. The liquid and ice are not simulated, so a straw rests on the glass's base and rim instead of floating
  among ice, and a spill is judged from how far the glass was tipped.
- Grip force is not judged.

## Real-world build

- Dispenser: a countertop single-straw dispenser fixed to the table, about 16 x 11 x 8.5 cm, whose lifter raises the
  front straw 3 cm when its handle is lifted and drops back when the handle is released. The handle is a red plate
  3 cm wide on an arm 7.5 cm out from the dispenser's left end.
- Straws: six short wide straws, 146 mm x 8 mm (5.75 inch).
- Glasses: five rocks glasses about 84 mm across and 85 mm tall, each filled to about 200 ml.
- Place the dispenser and the glasses on the marks given in `task.json` for the layout being run.

## Third-party assets

None. Every object is built from boxes and capsules in `task.py`.
