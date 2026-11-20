# Put a clog on a seated toddler manikin's dangling foot

A toddler-size care manikin sits on a child's chair facing the robot, feet dangling. Its left foot already wears
a foam clog; the other clog of the pair stands on the floor in front of it. The robot has to put that clog on the bare
right foot with the heel strap behind the heel, and let go.

The table stands in for the floor. Everything is at full size.

## Why it takes two hands

A clog's heel strap closes the back of the shoe into a ring (vamp edge in front, strap behind):

- From behind, the strap is in the way: only the toes pass under it, the thicker middle of the foot does not.
- From the front, the vamp is in the way.
- Straight down does not work either: a level foot is longer than the opening. Its toes have to go in first, pointing
  down, and slide forward under the vamp while the heel comes down in front of the strap.

A seated child's foot hangs level. It points down only while the knee is bent back, and the manikin's lower leg
swings freely at the knee, so something has to hold it there. The way through is the one a carer uses: one hand
bends the knee back and controls it, the other brings the clog under the toes and follows the foot forward as the
knee is straightened.

`sideview.py` is a side-view model of this (the leg's outline against the vamp, footbed, heel rim and strap). With a
2 mm margin it gives, for each knee angle, how far forward the toes can be:

| Knee bent back | Toes from the inside toe end | Gap to the strap |
|---|---|---|
| 30 degrees | 44 mm | 35 mm |
| 20 degrees | 30 mm | 23 mm |
| 12 degrees | 16 mm | 13 mm |
| 6 degrees | 4 mm | 17 mm |
| 0 degrees | 4 mm | 14 mm |

A level foot (0 degrees) that is already in cannot be lifted straight out in that model, and cannot be put straight in.

## Files

| File | What it is |
|---|---|
| `task.py` | The scene, the instruction and the scorer. |
| `clogkit.py` | The manikin, the free lower leg, the clog and its strap: shapes, hinges, fit measurements. |
| `demo.py` | A scripted demo that reaches full score, and two control runs. |
| `sideview.py` | The side-view planning model above (used by the demo's plan and the tests, not by the scorer). |
| `test_scoring.py` | 40 checks of the shapes, the fit measurements and the scoring rule; no simulator needed. |
| `task.json` | The task's description for the board. |

## Running and scoring a trial

```bash
# a policy (Dream-1 here), 1,500 steps at 20 Hz; the score is printed at the end and written to the log
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1500 \
  --instruction "Put the clog on the manikin's bare right foot, with the clog's heel strap behind the heel, then let go of the clog and the leg."

python demo.py task.py out_dir                 # the scripted demo: videos, setup.png, result.json
CLOG_LOT=B python demo.py task.py out_dir      # the second layout
python -m pytest -q test_scoring.py            # the offline checks
```

- `object_jitter=0` is required: the manikin and chair are fixed and the layouts are exact.
- There are no seeds. `CLOG_LOT` picks the layout: A (the clog straight in front of the right foot) or B (2 cm
  further from the robot and 4.5 cm to the right).
- A trial ends at full score held for 1 s, or at the run limit of 1,500 steps (75 s).
- Time per trial on one L40S: about 3.5 minutes for the demo (570 steps, with start-up), about 5 minutes for a
  1,500-step Dream-1 run.

## Scoring

Scored on the last step, in the clog's own frame: lengths back from the inside of its toe end, heights from its footbed.

| Score | Condition |
|---|---|
| 0 | The toes are not under the vamp. |
| 0.25 | Toes under the vamp: their tip within 4.5 cm of the inside toe end, within 2 cm of the clog's midline, between 1 cm below and 3.5 cm above the footbed's level. |
| 0.5 | And the clog hangs on the foot: at least 5 mm clear of the floor, at rest (under 2 cm/s; the leg turning under 0.2 rad/s), both grippers at least 3 cm from clog and leg. The strap is not behind the heel (it is forward on the vamp, or under the heel). |
| 0.75 | Toes under the vamp and the leg comes up through the opening in front of the strap, but the clog is still held, still moving or on the floor. |
| 1.0 | Through the opening in front of the strap, and hanging on the foot, let go and at rest. |

"Through the opening in front of the strap" means all of:

- the shank's middle, at the height of the strap's rivets, lies between 5 mm in front of the vamp's back edge and the
  front of the strap's crossbar, and within 2.5 cm of the clog's midline;
- the heel's back is in front of the crossbar;
- the ankle's middle is no more than 7.5 cm above the footbed (the sole within 4 cm of it).

## Scoring a real trial

Tools: a ruler, a 5 mm card, a phone camera on a stand at the manikin's side.

1. When the robot stops or the 75 s are up, look first: are both grippers at least 3 cm from clog and leg, and is
   anything still moving? Film the last seconds from the side to settle doubts.
2. Does the clog hang on the foot clear of the floor? The 5 mm card passes under it.
3. Toes: measure how far the heel's back sticks out behind the clog's heel rim. With the 13 cm foot, 2.2 cm or less
   means the toes' tip is within 4.5 cm of the inside toe end. The side walls leave the foot 4 mm of play each
   side, so a foot that is between them is within 2 cm of the midline.
4. Strap: from the side, the leg comes up between the vamp's back edge and the strap's crossbar, and the crossbar is
   behind the heel. A strap lying forward on the vamp, or caught under the heel, does not count.
5. Sole to footbed: no more than 4 cm at the heel (this is the 7.5 cm ankle limit).

This procedure has not been tried on a physical manikin.

## The demo

`demo.py` reaches 1.0 on both layouts. It uses only the layout constants and each hand's own measured position (to
read the knee's angle from where the left hand holds the leg, and to correct the right hand's sag). It never reads a
part's state.

| Step (layout A) | What happens |
|---|---|
| 120 | The right hand, leaning 60 degrees forward, has the clog by the sides of its toe box. |
| 155 | The left hand, leaning in from the manikin's right, has the lower leg 10 cm below the knee. |
| 230 | The knee is bent back 30 degrees (31.7 at most); the clog is just off the floor. |
| 365 | The clog has gone along the floor under the toes and up; the toes are under the vamp (0.25). |
| 420 | The knee is being straightened and the clog follows; the heel is down in front of the strap (0.75). |
| 470 | The knee is straight; the clog is pressed up under the sole. |
| 555 | Both hands have let go and drawn back; the clog hangs on the foot, 39 mm clear of the floor (1.0). |
| 570 | Full score has held for 1 s: the trial ends as a success. |

Layout B ends as a success at step 569.

Two things the demo's poses are chosen for, both found in the simulator:

- The clog hand leans 60 degrees forward and holds the toe box, so that its wrist stays low. Held more upright, or
  further back on the clog, its wrist meets the leg hand.
- It comes in from 8 cm above the clog, hovers and corrects for its own sag before coming down, so that its open
  fingers go astride the clog.

## Control runs

| Run | Score | What happened |
|---|---|---|
| `--mode still` | 0.0 | Nothing moves. |
| `--mode front` (one hand pushes the clog onto the hanging foot from the front) | 0.0 | The clog meets the foot and pushes the free leg back (to 38 degrees); the strap is pushed up by the foot; the clog is left on the floor. |
| `CLOG_STRAP=forward`, the two-handed demo with the strap swung forward onto the vamp first | 0.5 | The clog ends hanging on the foot, the strap still on the vamp. |

## Dream-1

One run, 1,500 steps, layout A: **0.0**. One arm reached down to the clog on the floor and hovered over it for most
of the run; the clog was nudged but never lifted, and the leg was never touched.

## Building it for real

- **Manikin.** A one-year-old pediatric care simulator with jointed knees: the Gaumard S110 "Mike and Michelle".
  Distributors' listings give it as 32 in (81 cm) long with "jointed elbows and knees", a waist that bends, and a
  "soft upper body skin over hard upper body". Sit it at the front of a child's chair with a 26 cm seat and hold its
  trunk to the chair's back with a belt; the feet then hang about 7.5 cm above the floor.
  - The listings give no leg or foot sizes. The sizes used here (knee to ankle 19 cm, ankle to sole 3.5 cm, foot
    13 cm long and 5.4 cm wide, lower leg 350 g) are estimates for a manikin of that length, not measurements of
    that model. Measure the manikin first. If its foot is not 13 cm long, use the clog size whose size chart matches
    the foot and scale the toe limits in step 3 of "Scoring a real trial" by the same difference.
  - The lower leg has to swing freely at the knee, as it does here. Whether that model's knee is this loose out of
    the box has not been checked; slacken the joint if it holds a pose.
  - One listing also calls the ankles jointed. The ankle here is rigid at a right angle: tape the manikin's ankle
    if it moves.
- **Clogs.** A pair of toddler size 6 foam clogs with pivoting heel straps (size charts give a 13.2 cm foot for
  that size). One goes on the left foot; the other stands on its mark with its strap back.
- **Floor.** The table, or a mat on the real floor with the arms mounted at its edge.
- **Safety.** Light, soft parts and forces of a few newtons. Nothing sharp, hot or powered.

The clog's sizes here (15.8 x 7.2 cm outside, 14.8 x 6.2 cm inside, 45 g, the vamp's heights, the strap's 6.2 cm
radius) are estimates of such a clog, not measurements of one. So are the leg's mass (350 g), the knee's friction
(0.03 N m) and the strap rivets' friction (0.012 N m). Measure the real parts before a real trial.

## What the simulation simplifies

- Every part is rigid. A real foot flexes at the toes and real foam gives; here neither does, so the clog is cut
  roomy (1.8 cm longer and 8 mm wider inside than the foot) and its strap clears the heel by 1.4 cm instead of
  hugging it. A real clog of the right size is a closer fit, which makes the same manoeuvre tighter, not unnecessary.
- The manikin's trunk, thighs and left leg are fixed; only the right lower leg moves. Its ankle is rigid.
- The foam meets a gripper pad over a 6 mm patch (soft foam on a rubber pad), which lets a pinch on the toe box
  carry the clog level.
- No limit on the force on the leg is enforced.

## Related tasks on the board

The board's care-manikin tasks fit or use something on a fixed head or hand (reading glasses, an ear thermometer, eye
drops, a pulse oximeter), or hold a doll's legs still while a footboard is closed. A pending task ("Give a care
manikin's ankle three passive range-of-motion repetitions") bends the ankle of a leg that lies on a pad and puts
nothing on it. None dresses a limb that moves, and none puts on footwear.
