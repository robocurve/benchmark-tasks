# Turn a wide board a quarter turn in place

A flat pine board (24 x 14 x 1.9 cm) lies on the table with its long side pointing away from the arms. The robot
must turn it 90 degrees so the long side faces the arms, keeping its centre within 2 cm of where it started. The
board is wider than the open gripper (9.4 cm) in both directions, so it cannot be picked up. While it lies flat its
edges cannot be pinched from the side either, because a finger would have to go under the table top. The only way
to turn it is to push its edges. Pushing near one corner turns the board but also slides it away, so turning it in
place means correcting that slide: two arms pushing opposite edges at once (a force couple, as the demo does) or a
series of corrective pushes with one arm.

| File | What |
|---|---|
| `task.py` | the task file: one loose box, `success`, `distance`, `score` (`SCORE_OVER = "end"`), `demo` |
| `task.json` | the submission metadata (`media` is added by the upload script) |
| `test_task.py` | offline checks, NumPy and pytest only |

## Run a trial

On the VM with `yam-isaac-lab-arena` set up (its Quickstart), copy this folder to `~/quarter-turn-board`. Always
run with `-E object_jitter=0` (see Seeds).

```bash
export INSTR="Turn the board a quarter turn so its long side faces you, keeping it where it is."
```

Scripted demo (full score expected):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$HOME/quarter-turn-board/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1250 --store-frames --log-dir ~/yam-eval/quarter-turn-demo
```

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$HOME/quarter-turn-board/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/quarter-turn-dream
```

Videos (one MP4 per camera, in each log's `frames/` folder):

```bash
for log in ~/yam-eval/quarter-turn-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score: ...` at the end: 1.0 when the episode ends in success, otherwise `score()` of the
final state. Put that number in `task.json` `scores`, e.g. `"Dream-1 run: 0.0; demo: 1.0"`.

Offline checks (here or on the VM):

```bash
YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
```

`YAM_ARENA` is only needed when the `yam_arena` package is not installed. All 57 checks pass against
yam-isaac-lab-arena at commit bd62771 (about 8 s).

## Runtime per trial

- Demo: 57.0 s of plan (1,140 steps at 20 Hz). The board is still and in the goal band from the squaring push
  (ending at 46.0 s) onward, so the episode should end with success around step 950-1,000, well before the
  plan's end. Plans have used about 60-70 steps more than their waypoints, so `--max-steps 1250` covers the whole plan.
- Dream-1: up to 2,400 steps (2 minutes of simulated time), unless it succeeds earlier.
- Wall clock on an RTX 5090, Isaac start-up included: 90 s for the demo, 214 s for the Dream-1 run.

## Seeds

Use `-E object_jitter=0`. The score measures drift from the board's starting centre as written in `task.py`, so a
seeded reset that shifts the board by up to 3 cm would be scored against the wrong start. There is one fixed layout,
so no seed is needed.

## Assets and licences

None. The board is a `Box` primitive defined in `task.py`. No meshes, textures or library assets are used.

## Scoring in one paragraph

Yaw error is the angle between the board's long side and the y axis (across the table), taken mod 180 degrees,
because the board looks the same turned end for end. Drift is the distance from the board's centre to its
starting centre. The board earns credit only while it lies flat on the table (tilt at most 5 degrees, bottom within
5 mm of the table top). **1.0** (success, after 1 s still): yaw error at most 5 degrees and drift at most 20 mm.
**0.5**: at most 15 degrees and 40 mm. **0.2**: at most 15 degrees and 100 mm (turned, but not in place).
**0** otherwise. `SCORE_OVER = "end"`: the final state decides.

## Shortcuts that score 0 (all tested)

- Sliding the board anywhere without turning it: still 90 degrees off, so 0 at any drift. Also 0: slid 5 cm
  sideways, slid 15 cm forward, slid 20 cm sideways and turned 10 degrees.
- A half turn (180 degrees): the long side still points away from the arms.
- A partial turn: 45 degrees and 70 degrees both score 0. The lowest tier needs 75 degrees of turn.
- Turning it while sweeping it away: turned but more than 10 cm from the start.
- Pushed off the table, off its side, or lying on the floor.
- Propped up on a finger (8 degree tilt), held 2 cm above the table, or standing on its long edge.
- Full score while still moving is not yet success (it must be still for 1 s).

## Design numbers

- **Gripper and pads (from `constants.py` and the URDF's collision meshes).** Fully open, the pads are 94 mm apart
  (`FINGER_OPEN = -0.04695` m per finger). The board's short side (140 mm) is 46 mm wider than that, and the
  test asks for at least 40 mm. With the fingers closed, the bundle between the pad tips and the TCP is 12.8 mm
  across the closing direction and 27.4 mm along the pads. The pad tips are 13 mm past the TCP. Higher up the
  fingers widen: 20 x 40 mm at 24 mm above the TCP.
- **Push height and board thickness.** The lowest safe TCP height is 19 mm above the table, which puts the pad
  tips 6 mm above it. That is the height the first GPU round used to grip a board without the pads touching the
  table, and it allows for the 1-2 mm of wrist sag measured there. At that height a 12 mm board (the idea's first
  sketch) would meet only 6 mm of finger. A 19 mm board (stock 1x6 pine) meets 13 mm of finger, and the TCP sits
  level with its top, so only the narrow part of the fingers touches the edge. The closed fingers taper slightly
  toward the tips, so the contact pushes the edge down onto the table, never up. Between pushes the hand rises
  3 cm (pad tips 17 mm over the board's top) before it moves sideways.
- **Changes from the original sketch (240 x 160 x 12 mm, about 300 g).** Thickness 12 -> 19 mm, for the reason
  above. Width 160 -> 140 mm, so the board is a 1x6 shelving board that needs only one crosscut, with no rip cut.
  It is still 46 mm wider than the open gripper. Mass about 0.32 kg (pine at about 500 kg/m^3).
- **Layout.** Centre at (0.30, 0.00), the long side along x. The board's corners sweep a circle of radius 139 mm,
  from x 0.16 to 0.44, which stays on the table and in front of the arm bases.
- **Demo (both arms in step, closed fingers, 57 s).** The board is never grasped.
  - *Phase A (0 -> 45 degrees, three 15 degree pushes of 3 s).* The right hand pushes the right long edge 6 cm
    forward of the centre, toward +y. The left hand pushes the left long edge 7 cm behind the centre, toward -y.
  - *Phase B (33 -> 92 degrees, four 14.75 degree pushes of 3.5 s).* The right hand pushes the near end 4.5 cm
    right of the centre, along the board. The left hand pushes the far end 4.5 cm left of the centre, the other
    way. Phase B is planned from 33 degrees, not 45, so a hand set down for a board that lagged the plan by up to
    12 degrees still lands on bare table; if the board kept up, the hand meets it during push 1.
  - Each hand tracks a fixed point on its edge round the arc, pressed 4 mm in against the wrist's lag. The yaw of
    each hand turns with the board, so the flat outer face of one finger stays parallel to the edge. In both
    phases the two pushes point in exactly opposite directions and both turn the board the same way. With no net
    push, the board turns about its own centre.
  - *Squaring.* Both hands, 12 cm apart, come down 35 mm in front of the near long edge and push it to 8 mm past
    where a perfectly turned board's near edge would be. Two contacts advancing together on one edge line that
    edge up with them, which removes whatever yaw error the turning left, in either direction. The tested landing
    allows a board up to 14 degrees short of or 4 degrees past 92 degrees. The board ends 0-8 mm forward of its
    start, depending on how much the wrists lag.
  - The hands back off 12 mm from the edge (in 1.5 s) before every rise, go up to 10 cm above the table, wait at
    (0.22, +-0.20), 8 cm beyond the turned board's ends, and hold still for 2 s.
- **Speeds.** Smoothstep moves: the peak speed in contact is 1.76 cm/s (the squaring push; 1.2-1.43 cm/s for
  the turning pushes) and the peak acceleration is 0.0176 m/s^2. Both are inside the first GPU round's dragging
  limits (4.5 cm/s, 0.02 m/s^2). The acceleration bound sets the durations.
- **Reach.** The whole plan solves at 20 Hz on both arms with worst IK error under 0.1 mm and 0.02 degrees, inside
  the joint limits. That includes the interpolation between waypoints, so the arcs are checked, not only their
  end points. The hardest point is the right hand at (0.394, -0.009) at the end of phase A. Neither hand crosses
  more than 2 cm past the midline, and the two hands stay more than 10 cm apart at every waypoint.
- **Expected forces (quasi-static estimate, not measured).** Turning a uniform 0.32 kg board about its centre on
  friction 0.75 takes about 0.18 N m (mean contact radius 74 mm). With the levers used, that is about 1.4 N per
  hand in phase A and 2.0 N in phase B, roughly half the 3.6 N load of the GPU-proven board drag. So expect less
  than that task's 1 cm of wrist lag.

### Why the tolerances are what they are

- 5 degrees and 20 mm for full score. On a 24 cm board, 5 degrees moves the ends by
  10 mm. Both limits can be read off an overhead photo or checked with a protractor and a ruler.
- Mod 180 is needed because the board is symmetric. Either quarter turn, clockwise or anticlockwise, is correct.
- 0.2 for "turned but moved" is capped at 10 cm, so a board swept round in one big push still earns a little,
  and a board sent across the table earns nothing.
- The flat requirement makes a board propped on a finger at the end of the run worth 0, not a full score.

## How it differs from the nearest tasks (novelty)

Checked against the task board (90 pending tasks on 2026-11-02) and the normalized list of LIBERO, RoboCasa(365),
SimplerEnv, BEHAVIOR(-1K), DataCenterBench and RoboDojo, plus RoboTwin 2.0, RoboLab, ManiSkill 3 and CALVIN.

- **RoboDojo `push_T` and ManiSkill / Diffusion Policy PushT** push a small T block (graspable in principle) to a
  goal pose marked by a pad. The goal there is a translation plus a rotation to a visible target, and one pusher
  is enough. Here there is no target on the table and the goal is a pure rotation about the object's own centre.
  The board is too wide for a top-down grasp. The task measures a quarter turn with limited movement of its
  centre. The demo uses opposed pushes from both arms; a sequence of corrective pushes is also valid.
- **CALVIN `rotate_red_block_left/right`, RoboCasa `RotatePan`, RoboTwin `rotate_qrcode`** turn objects small
  enough to grasp or with a handle. Here the board is wider than the open gripper, so it cannot be grasped from above.
- **Board B40 "Slide a crate onto the tape line" and B37 "Push a bottle without tipping it"** are translation
  pushes to a line. Neither asks for a rotation, and neither forbids drift.
- **ManiSkill `TriFingerRotateCube` and in-hand rotation tasks** rotate with dexterous fingers. This task uses
  only parallel grippers with closed fingers.
- **Tasks that turn a small bar a quarter turn where it lies** do it by picking the bar up. This board cannot be
  picked up, so the skill is nonprehensile.

Skill signature: `slide-push / rigid / spatial-precision(20mm,5deg) / either` (two arms are natural; one arm can
do it with many corrective pushes, but each lone push also slides the board).

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 90 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 214 s wall clock.
- Offline tests: 57 passed.
