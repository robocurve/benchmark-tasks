# Pin a notice to a tilted display board with its top edge at the line

A cork display board stands on a desk easel, tilted 30 degrees from the table and facing the arms, with a wooden
ledge along its bottom edge. A laminated A6 notice stands on the ledge, lying flat on the cork. Two black ticks at
the sides of the board mark a line 2.5 cm above the notice's top edge. A pushpin stands needle-down in a pin cup on
the table.

The instruction is: "Pin the notice to the display board with its top edge at the line, then let go."

Teaching assistants put up notices on cork display boards (O*NET 25-9042 Teaching Assistants, Except Postsecondary,
1.46 million employed: "Prepare lesson materials, bulletin board displays, exhibits, equipment, and demonstrations").
The board here is a tabletop easel rather than a wall board, so the face is within the arms' reach; its slope still
makes the notice slide, which is what forces the hold. On a tilted or upright board
the notice slides down as soon as it is let go, so one hand has to hold it where it should go while the other pushes
a pushpin through it into the cork. The pin has to be pressed square to the board, hard enough to take (about 6 N
into cork), without shoving the notice out of place, and the robot then has to let go of both. The task tests holding
a thin sheet in place on a sloped surface with one arm while the other fastens it with a small part held at an angle.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left). The layout is fixed.

| Object | Description |
|---|---|
| Display board (fixed) | Cork face 30 up the face by 32 across, 1.2 thick, tilted 30 degrees from the table; its low edge at x 12, 3 above the table, spanning y -10 to +22. A wooden ledge 1 x 32 x 1.2 along the low edge, 1.2 proud of the face. A pine foot 3 x 32 x 3 and a pine rear strut 3 x 3 hold it. Two black ticks, 2 long and 2 mm wide, 1.2 in from each side of the face, mark the line 17.5 up the face from the ledge |
| Notice (6 g) | Laminated A6 card, white, 14.8 x 10.5 x 0.1. Lies flat on the face standing on the ledge, centred at y 7 (it spans y 1.75 to 12.25). Its top edge is 15 up the face, 2.5 below the line |
| Pushpin (5 g) | Red plastic head 11 mm tall and 9 to 11 mm across over an 11 mm collar; 8 mm steel needle. Stands needle-down in the pin cup, its collar resting on the cup's top |
| Pin cup (fixed) | Grey plastic block 4 x 4 x 2 with a 3 mm hole, at x 25, y -17 |

## Scoring

The task file follows the trial after every control step and `score()` gives the state at the end:

- **0.25:** the pushpin stands in the cork (at least 2 mm in, anywhere on the board) and has been let go (every
  gripper's fingertip centre at least 4 cm from it): credit for taking the pin and driving it in.
- **0.5:** the pin's needle is through the notice at least 3 mm from the notice's edges, its head seated on it
  (within 1.5 mm), and let go, but the notice isn't at the line or hangs crooked.
- **0.75:** the notice is pinned like that with its top edge within 1 cm of the line and its long edges within
  5 degrees of the board's up direction, but a gripper's fingertip centre is still within 3 cm of the board face or
  4 cm of the pin.
- **1.0:** as 0.75 with both grippers clear and the notice still (its top edge has moved under 2 mm and it has
  turned under 1 degree in the last 0.5 s). `success()` ends the episode once that has held for 2 s.
- **0** otherwise, including a notice that has come off the board.

In the sim, "through the notice" means the needle's tip reached the cork inside the notice's outline, at least 3 mm
from its edges, with the pin's head within 1.5 mm of the notice; the task file records that point on the notice and,
once the pin is seated and let go, the notice hangs from it. On a real board, check the hole with the pin pulled out
afterwards, and read the top edge against the ticks with a ruler.

## Run a trial

Following the task board's simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Pin the notice to the display board with its top edge at the line, then let go." \
  --scorer task_score --no-prompt --max-steps 2400 --store-frames --log-dir ~/yam-eval/pin-notice-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A full 2,400-step Dream-1 trial took about 10 minutes of wall time
on an L40S with frames stored (including the model server's queue); the scripted demo ends on success after about
27 s of simulated time and took about 3 minutes.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`. The two arms work at the same time:

- **Left arm:** closes its fingers, presses the notice to the board with the closed fingertips 3.5 cm above its
  bottom edge (the grip turned so the pads lie across the notice), slides it 4 cm up the board (it slips a little
  under the pad and ends about 2.5 cm higher, its top edge on the line), holds it there while the pin goes in, then
  lifts off 4 mm over 1.5 s and backs away to 5 cm.
- **Right arm:** picks the pushpin out of the cup by its head, lifts it, turns the grip 30 degrees so the pin is
  square to the board, brings its tip down onto the notice about 1 to 2 cm below the top edge and 1.5 to 3 cm in
  from the side (the loaded arm lands a little high),
  pushes until the collar seats (the arm is commanded up to 4 cm past touching; the pin stops when the collar
  reaches the notice), eases off 3 mm, opens the fingers over 2.5 s, retreats 5 cm and waits.

Set `PIN_DEBUG=1` to print the pin's depth and tilt, the notice's position against the line and the score terms every
4 control steps (every step while the fingers open). `F_INS` (N, default 6) sets the cork's resistance, `PIN_TILT`
(degrees, 30) the grip's tilt, `CARD_PRESS` (mm, 15) how far the pad is pressed toward the board, `OPEN_S` (s, 2.5)
how long the fingers take to open.

Shortcuts, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the arms do | Score |
|---|---|---|
| `full` | as above | 1.0 (success at step 503, episode ends at 543) |
| `nothing` | wait | 0.0 |
| `pin_only` | the pin goes into the bare cork 8 cm beside the notice; the notice isn't touched | 0.25 |
| `no_slide` | the notice is pinned where it rests, 2.5 cm below the line | 0.5 |
| `keep_holding` | as `full`, but the left arm never lets go of the notice | 0.75 |

## How the cork is modelled

The needle is a collider on a slider in the pin's head, with 6 N of friction on the slider. Its tip stops on the
board face; pushing the head harder than 6 N drives the needle back into the head, and how far it has gone back is
how far the pin is in. The collar stops on the notice, 7 mm in. Once the collar seats, or the fingers open with the
tip at least 2 mm in, the cork holds the pin: its pose is written back every physics substep (240 Hz), moved out
along its axis by what it takes for the collar to clear the notice (usually nothing). Pulling a seated pin out again
is not modelled. A notice with a seated pin through it is tethered at the hole to the pin's axis by a spring-damper
(60 N/m, 0.25 N s/m, at most 2 N): it can slide along the needle, but not away from it. The pin's head clamps the
notice to the cork, which is what keeps a notice from swinging about a single pin: a torsional spring-damper about
the pin (0.5 N m/rad, 0.002 N m s/rad) holds the turn the notice was pinned at, up to 0.003 N m (the clamp's friction
torque), beyond which it slips. The 6 N insertion force is an estimate; pressing a pushpin into cork on a kitchen scale would give the real number, and
`F_INS` sets it.

## Offline checks

`test_task.py` checks the scene geometry, the tether's stability at 240 Hz, the scoring rule, the notice findings on
hand-built poses, the collar-clearance shift and the demo plans, without Isaac Sim:

```bash
PYTHONPATH=<path to yam-isaac-lab-arena> python -m pytest test_task.py -q
```

## Real-world build

Use a 32 x 30 cm cork display board on a desk easel set to 30 degrees, with a 12 mm ledge along the bottom (a strip
of wood glued on), and draw the line or stick two ticks 17.5 cm up the face. Laminate an A6 sheet (about 0.4 mm of
laminate on 160 g card makes a 1 mm, 6 g notice). Use an ordinary pushpin (plastic head about 11 mm, needle 8 mm)
standing in a pin cup or a block of foam with a 3 mm hole. Mark where the notice and the cup go. Score with a ruler
at the end. The task is safe to run: the arm presses a pushpin with about 6 N and the notice weighs 6 g.

## Licence

This task is released under the MIT licence. `partkit.py` is this team's part library (rigid parts built from
several boxes and cylinders, with sprung sliders), released under the same licence. No third-party models or assets
are used. Every object is built from primitive shapes.
