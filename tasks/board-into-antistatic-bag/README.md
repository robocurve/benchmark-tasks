# Slide a circuit board into an antistatic bag held by the other hand

Bagging a bare circuit board for resale, the way an electronics refurbisher or an e-waste triage line does it. The
board must go into its antistatic bag flat and all the way in, then the bag goes in a crate and is let go of. The bag lies on a slick
mat and the board grips its inside: pushing the board in drags the bag along, so one hand holds the bag while the
other feeds the board.

| File | What it is |
|---|---|
| `board_into_antistatic_bag.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `envelope.py` | `Envelope`: a hollow bag as one rigid body of thin box walls. `surface()`: another friction for a task-file box |
| `contact.py` | Reads PhysX contacts of every arm link with the bag and the board after each physics substep: `released()` |
| `bonds.py` | Hooks a callback on every task-file box as it spawns (`on_spawn`), which `envelope.py` and `looks.py` use |
| `simready.py` | Loads the crate, an NVIDIA SimReady prop (CC-BY 4.0), pinned to one dataset revision |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which keeps its bounds |
| `looks/` | The meshes (`*.usdc`) and their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_board_into_antistatic_bag.py` | Offline checks: the file loads, the bag's cavity, every scoring outcome (bag turned, held, hanging over a wall, not released), the release rule, the demo's order, depths, arm spacing and reach (no GPU) |
| `task.json` | The task's details for the board |

## The bag

`envelope.py` keeps the bag's spawned 7.7 x 15.2 x 1.2 cm box for its bounds (so the scorer reads the bag's
centre) and replaces its collider with seven thin boxes, all shapes of the same rigid body: a 2 mm bottom layer, a 2
mm top layer with a 3 x 1.5 cm thumb notch at the mouth, two 2 mm side seams and a 4 mm closed end. That leaves a
cavity 7.3 cm wide, 14.8 cm deep and 8 mm high, open at the mouth. The board (6.5 x 14 cm, 3.6 mm thick with its
parts) fits it with 4 mm of play each side, 2.2 mm above and below, and 8 mm at the closed end.

Friction: every task object has yam_arena's 1.0 (1.2 static). The mat has 0.25, combined by the minimum, so the bag
and the board slide on it at 0.25, while the board drags on the bag's inside at 1.0. Pushed along the bag's floor,
the 45 g board pulls the bag with up to 0.44 N, and the mat holds the 15 g bag (with the board in it) with 0.15 N:
an unheld bag rides along.

## Assets

| Object | Source |
|---|---|
| crate | SimReady `crate_gray_a08` (CC-BY 4.0), revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, scaled 0.42 to 25.4 x 17 x 7.2 cm (inside 24.4 x 16 cm, floor 2.5 mm up), fixed. Downloaded on first use into `~/.cache/simready` |
| bag, board, mat | Task-file boxes, drawn by meshes modelled in Blender (`looks/make_looks.py`): the pink bag with heat-sealed seams, the notch and a printed ESD mark, the board with its chips, crystal, button, LEDs and small parts, the dark grey mat with its ground snap |

SimReady has no bag, envelope, circuit board or mat. The bag's and the board's meshes fit inside their boxes (a test
checks it). To change the meshes, edit `looks/make_looks.py` and run it in Blender 4.2 or later:
`blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=board_into_antistatic_bag.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the pink antistatic bag with the left hand, slide the green circuit board flat into it with the right hand until it is fully inside, then lay the bag in the grey crate and let go of it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/board-into-antistatic-bag-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=board_into_antistatic_bag.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the pink antistatic bag with the left hand, slide the green circuit board flat into it with the right hand until it is fully inside, then lay the bag in the grey crate and let go of it." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/board-into-antistatic-bag-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The layout: the mat, the board and the bag lie along one diagonal line, 120 degrees from x (from front right to
  back left). Along it, both grippers close across the board and the bag with their wrist cameras pointing the same
  way, so the right gripper's camera points away from the left gripper. The right arm works at y < 0, the left arm
  at y > 0.
- The demo, all straight-down grips: the left arm closes across the bag's width at its closed end, where it overhangs
  the mat, and holds it. The right arm closes across the board's width at its near end, also past the mat's end,
  lifts it 5.7 mm so it rides in the middle of the cavity, slides it 16 cm toward the bag until the fingers are 3 mm
  short of the mouth (the board is then 11 cm in), lets go, closes its fingers behind the board and pushes it the
  last 3.5 cm through the thumb notch, and leaves with open fingers. Then the left arm lifts the bag, turns it along
  the crate, lowers it to 2 cm above the floor, opens its fingers (the bag drops and lies flat on the floor) and
  rises. Both hands end open and clear of the bag. The fingers are sent a few mm past each object's sides, not fully closed, so they squeeze with 20 N (board) and 12.5 N
  (bag): closed with the full 50 N, the light board and bag shoot out of the grip. It reaches 1.0 and ends on
  success at step 409 (20.5 s at 20 Hz), 1 s after the left hand lets go. A trial takes about 80 s on an L40S, including startup.
- The one-handed shortcut, the same demo with the left arm hovering instead of holding the bag (`BAG_GRIP =
  FINGER_OPEN`, `HOLD_Z = HOVER_Z`), scores 0.25: the board drags the
  unheld bag off the end of the mat and stays about half in.
- The same demo with the left hand keeping hold of the bag in the crate (`LET_GO = BAG_GRIP`), so the filled bag lies
  in the crate but is never released, scores 0.5.
- Dream-1: 0.0 (ran to the step limit without finishing).
- Videos: `inspect-robots video ~/yam-eval/board-into-antistatic-bag-demo/<log>.json` writes one MP4 per camera next
  to the frames.
- Offline checks: `python -m pytest -q test_board_into_antistatic_bag.py`.

## Scoring

All positions are in metres, in the station frame (x forward, y to the robot's left, from the midpoint of the arm
bases), and heights are above the table top. Where the board is in the bag is measured in the bag's own frame (along
its length from the mouth, across it, and through its thickness), so it counts the same wherever the bag lies and
however it is turned.

- **0** if the board or the bag fell off the table (centre more than 2 cm below the top) or the state is not finite.
- **1.0 (success, held 1 s)** needs all four:
  - **Board:** the whole board in the cavity (every corner past the mouth's line and within the side seams, its centre
    between the top and bottom layers, 2 mm of slack), its far edge within 1 cm of the closed end.
  - **Bag in the crate, on its floor:** all four corners of the bag's outline between the crate's inner walls
    (24.4 x 16 cm, centred at (0.30, 0.28)), every point below the rim (7.2 cm up), its lowest point within 3 mm of
    the floor (2 mm of slack below it) and its highest point at most 1.6 cm above its lowest, so it lies flat on the
    floor, either face up. A bag hanging over a wall, leaning, on edge, on the rim or held above the floor fails this.
  - **Released:** no robot link touches the bag or the board (see below).
  - **Still:** the bag and the board each under 1 cm/s and 0.5 rad/s.
- **0.5:** the board fully in the bag as above, with the bag anywhere else (held, on the mat, beside the crate, hanging
  over a wall, above the floor, tilted), still touched by a gripper, or still moving.
- **0.25:** the board's centre in the cavity: at least half of the board in.
- **0** otherwise, including doing nothing and a board lying on, under or beside the bag.

The cavity is 8 mm longer than the board, so a board fully in always has its far edge within 1 cm of the closed end.

**Release.** The scorer cannot see the grippers, so `contact.py` reads them from PhysX. It builds a rigid-contact view
between every robot link (the arm and gripper links of the YAM, which all report contacts) and the two free bodies,
the bag and the board, and reads its contact points after every one of the 240 Hz physics substeps. A contact point
with a separation of 1 mm or less (touching or pressing in) is a touch. The bag is released when no link has touched
the bag or the board for 0.1 s. If the view cannot be built or read, nothing counts as released, so the full score
cannot be reached on a failure.

**Real trial.** A tape measure gives how far the board is in (or a look through the bag). The bag must lie flat on the
crate's floor with no part over a wall or on the rim (a ruler across the crate, or a photo from above), both grippers
open and clear of the bag and the board, and the bag and board motionless for 1 s.
