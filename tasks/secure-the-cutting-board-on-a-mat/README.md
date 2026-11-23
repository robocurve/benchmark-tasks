# Secure the cutting board on a non-slip mat before knife work

A hardwood carving board with two end handles lies on the bare counter between the two arms; a foam-rubber non-slip
mat (grip liner) lies on the counter just in front of it, nearer the robot. The instruction is only: "Set up the
cutting board for knife work." Knife-safety guidance asks for the board to be secured before cutting, because a board
that slides under the knife leads to cuts:
[Ottawa Public Health](https://www.ottawapublichealth.ca/en/public-health-topics/resources/Documents/knife_safety_en.pdf)
says to secure the cutting board with a damp paper towel or a non-slip mat,
[Southwestern Public Health](https://www.swpublichealth.ca/media/zqzfcprk/general-kitchen-and-knife-safety-aoda.pdf)
says to make sure the board is secure and not moving, University of Maine Cooperative Extension Bulletin #3108,
[Safe Cutting Boards: Materials, Usage, and Cleaning Tips](https://extension.umaine.edu/publications/3108e/), says to
make sure the board is stable and will not slide around while in use, with a damp paper towel, a clean damp rag or a
non-slip mat underneath, and UC ANR's
[Tips for Teaching Knife Skills](https://ucanr.edu/sites/default/files/2024-02/393200.pdf) (Karin Allen, PhD) says it
is important to keep the cutting board from moving during use and that non-slip mats can secure any board. For restaurant kitchen workers,
[WorkSafeBC Magazine](https://www.amtrustfinancial.com/getmedia/8110b34c-c0b0-4283-81e0-90c96d8e520a/WorkSafe_BC_Magazine_-_Restaurant_Kitchen_Safety.pdf)
says to secure the cutting board to the counter. The instruction does not mention the mat, so the task checks whether the robot puts the board on it unprompted, and gets
it there the right way:

- **Lift, do not drag.** The mat is light foam rubber (0.12 kg). In simulation the 1.8 kg board, slid across the
  counter, pushes it away: in the recorded `drag` run the mat moved 14.65 cm and the board ended with 0.1% of its
  underside over it. That is a simulation result; a real mat may bunch up rather than slide.
- **Two arms are the natural way.** The handles are 32.4 cm apart. The scripted demo lifts the board by both handles,
  one arm each, and it stays within 2.56 degrees of level. In simulation one arm can also carry it by one handle: in
  the recorded `one_hand` run it hung at most 8.17 degrees tilted and scored 0.5, missing full credit only because it
  was set down short (0.404 of its underside over the mat, against 0.70 for 1.0). That too is a simulation result:
  held by one end handle, the board's weight acts about 16 cm from the grip, so a real one-handed pinch must resist
  about 2.9 N m and would likely let the board twist. The scorer does not count arms.
- **Any controlled carry, level or steeply tilted.** How the board is held while it is carried is not scored, only
  that it is not dropped and where it ends. In the recorded `edge_on` run both arms lift it 15 cm, tilt it nearly onto
  its edge (at most 69.28 degrees from level), carry it toward the mat that way, turn it level above the mat and set
  it down: 1.0.
- **Set down, not dropped.** In the recorded `drop` run the arms let go 5 cm above the mat; the board landed flat,
  0.905 of it over the mat, and scored 0.

## Changes in this revision

- Scoring: the permanent zero for carrying the board tilted more than 20 degrees is removed. How the board is held on the
  way (level, tilted or on edge) is not scored. The drop check and the final placement requirements are unchanged, and
  the drop is now the only condition that zeroes a trial for good; everything else is judged on the board's final state.
- New demo variant `edge_on`: both arms carry the board turned 70 degrees about the line through its handles (nearly on
  edge) and set it down flat on the mat. It scores 1.0 (success at step 509; at most 69.28 degrees from level while
  carried), where the previous scorer gave 0.
- The other five variants were re-run with the new scorer and reproduced their earlier results exactly (table below).
- Demo video: the full demo (1.0, 317 steps), then the `edge_on` run (carried up to 69 degrees tilted, on edge at about 0:28-0:31), 1.0 at 509
  steps. `demo_results.json` holds every run's result, `edge_on` included.
- Texts: the knife-safety sources are now public-health and university-extension guidance (a blog source was
  replaced); the real board, the mat, a real-trial build, the end of a real trial and how a folded or bunched mat
  counts in a real trial are described; and the simulation-only results (the one-handed carry, the mat pushed away by
  a dragged board) are labelled as such.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (visual-only parts for the lines on the board's top), and per-step readings: part poses, finger-pad distances, drop and stillness watches |
| `demo.py`, `democore.py` | The scripted full-score demo, a second full-score run that carries the board steeply tilted, and the failure and cheating variants; `demo.py` also writes `setup.png`, the starting layout |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |
| `setup_shot.py` | Writes a 1280 x 704 top-camera screenshot of the starting layout (`python setup_shot.py task.py setup.png`) |

Scores on the task board: Dream-1 run: 0.0 (1,200 steps); demo: 1.0 (317 steps); demo video: that run, then the
`edge_on` run (carried up to 69 degrees tilted, on edge at about 0:28-0:31), 1.0 at 509 steps. The video holds its final frame for 4.5 s at the end.

**Assets:** no third-party meshes, scans or scenes. Every part is a primitive box built with `kitchenkit.py` helpers in
`task.py`, placed in yam_arena's station scene with its YAM arms. The robot and the station come with the program's `yam_arena` embodiment and are not part of this submission: the
YAM arm model (URDF and meshes) is from I2RT Robotics (github.com/i2rt-robotics/i2rt) under the MIT licence, and the
station scene is the one yam_arena provides for this program.

`kitchenkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set up the cutting board for knife work." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/cutting-board-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the recorded scripted demo ends on success at step 317 (about 16 s of simulated time at 20 Hz); the
`edge_on` run ends on success at step 509, and the failure variants run their whole plans, 306 to 386 steps. Run two
at a time on one NVIDIA RTX PRO 6000 Blackwell GPU shared with other jobs (Nov 22), each scripted run took 2.8 to
3.4 min of wall-clock time from start-up to shutdown, and the `edge_on` run 5.8 min (it includes a 32 s
inverse-kinematics search at plan time and records video). A model trial runs up to 1,200 steps, plus the policy's own
response time. Measured on Nov 22 with this version: the Dream-1 run (`dreamlabs/dream-1-beta`, 1,200 steps,
`rows_per_request=12`) scored 0.0 and took 10.9 min of wall-clock time, simulator start-up included, on one NVIDIA RTX
PRO 6000 Blackwell GPU shared with other simulator runs (on Nov 15, with the GPU to itself, the same 1,200-step run
took 3.2 min).

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_drag --variant drag          # any variant below; --no-record skips the video
python demo.py task.py --plan-only                      # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out           # also print the board's state every 10 steps
python setup_shot.py task.py setup.png                  # 1280 x 704 screenshot of the starting layout
```

Each arm pinches one end handle (the left arm the left handle, the right arm the right one), the two lift the board
5 cm together, carry it 24 cm toward the robot to above the mat's centre, lower it onto the mat, let go and go home.
The `edge_on` run pinches the same two handles but lifts the board 15 cm, so that it clears the counter when turned,
and turns it about the line through the two handles (far edge up, top face toward the robot). `demo.py` picks the turn
at plan time: the steepest of 90, 80 and 70 degrees (then lower) at which both arms' inverse kinematics stays on
target along the whole plan; here 70 degrees for the whole carry (90 and 80 degrees over the whole carry did not pass
that check).

| Variant | What it does | Score | Steps | End state |
|---|---|---|---|---|
| `full` | Both arms lift the board by its handles, carry it over the mat, set it down and let go | **1.0** (success) | 317 | 0.899 of the underside over the mat, tilt 0.01 deg, at most 2.56 deg while lifted, mat moved 0.01 cm, released and still, not dropped |
| `edge_on` | Both arms lift the board 15 cm by its handles, turn it 70 degrees about the handle line, nearly onto its edge, carry it toward the mat that way, turn it level above the mat, set it down and let go | **1.0** (success) | 509 | carried tilted at most 69.28 deg from level; then 0.892 of the underside over the mat, tilt 0.0 deg, mat moved 0.00 cm, released and still, not dropped |
| `drag` | The right arm alone slides the board by its handle across the counter onto the mat | 0.0 | 306 | the mat was pushed 14.65 cm away; 0.001 over the mat, the board slid along the counter, not lifted (largest tilt while lifted 0.0 deg), and ended on the counter (its underside 0.4 cm below the mat's top) |
| `one_hand` | The right arm alone lifts the board by its right handle and carries it to the mat | 0.5 | 386 | carried at most 8.17 deg tilted and set down short: 0.404 over the mat, above the 0.40 for 0.5 and under the 0.70 for 1.0; resting at mat height, tilt 1.22 deg, mat moved 1.67 cm, not dropped |
| `half_on` | Both arms set the board down 7 cm short, overhanging the mat's far edge | 0.5 | 386 | 0.628 over the mat, tilt 0.01 deg, at most 2.26 deg while lifted, released and still, not dropped |
| `drop` | Both arms let go of the board 5 cm above the mat | 0.0 | 336 | dropped; it landed flat (tilt 0.01 deg) with 0.905 over the mat |

The numbers are from `demo_results.json`.

## Scoring

Every step, from poses; the score at the end counts.

- **Share over the mat:** the board's underside is sampled at 713 points, a 23 x 31 grid at 1 cm spacing over its
  22 x 30 cm underside with the edges included. The share is the fraction of those points inside the mat's 24 x 28 cm
  outline, measured in the mat's own frame, so it follows the mat if the mat moves. The mat is 2 cm shorter than the
  board along y, so a board centred square on it has 667 of the 713 points over it (0.935).
- **Released and still:** no finger pad within 8 mm of the box enclosing the board and its handles (22 x 35.4 x 3 cm),
  and the board and the mat each moving less than 2 mm over 1 s.
- **1.0:** the board rests flat on the mat: at least 70% of its underside over the mat, top face up within 3 degrees
  of level, the centre of its underside within 6 mm of the mat's top surface, released and still. The top face must be
  up because it is the cutting face; the handle blocks sit flush with it.
- **0.5:** released and still, resting at mat height (the centre of its underside within 1.6 cm of the mat's top
  surface), at least 40% over the mat and within 10 degrees of level (set down under 70% over the mat, or resting
  tilted on the mat).
- **0:** otherwise, and from the moment the board is dropped (it falls 3 cm within 0.5 s with no finger pad within
  2 cm of that box). A drop is the only zero that lasts; everything else is judged on the board's current state.

How the board is held on the way is not scored: it may be carried level, tilted or on edge, as long as it is not
dropped and ends flat on the mat (the `edge_on` run carries it tilted up to 69.28 degrees and scores 1.0). How the
board reaches the mat is not scored directly either: in simulation a board dragged across the counter pushes the light
mat ahead of it and ends off the mat (a real mat may bunch rather than slide). `report()` records each step the share
over the mat, the board's tilt and height over the mat's top, the largest tilt while it is more than 1 cm above its
starting height (reported only, not scored), how far the mat has moved from its start, whether the board is released
and still, and the drop flag.

## Real-world trials

Build the board as a 22 x 30 x 3 cm hardwood board (beech or maple, about 1.45 kg; weigh it and log the
mass; the simulation's 1.8 kg is heavier, and no score threshold depends on the mass) with a 4 x 2.7 x 2 cm hardwood block glued and screwed to the middle of each
end as a handle, sticking out 2.7 cm as in the simulation, tops flush with the board's top. Carving boards with a handle at each end are sold (for example Viking's 20 x 14 in acacia carving board, with two
oversized handles); the blocks reproduce such handles at this board's size. Even a heavy hardwood board needs the mat:
the cited guidance applies to any board, and counters are often wet or greasy during prep.
Cut a foam-rubber non-slip mat (grip liner) to 24 x 28 cm (0.4 cm thick, about 0.12 kg); store it flat and lay it
flat at its mark before each trial, edges not curled. Mark both start positions
with tape and start both arms in the ready pose shown in the screenshot. No knife is needed: the trial ends at the
60 s limit, or once the board has rested flat on the mat for 1 s with both grippers open and clear of it and the board
still. The share over the mat is, to 1 cm, the fraction of the board's 22 x 30 cm footprint (handles excluded) that
lies over the mat: with the board square on the mat, measure the overlap rectangle with a tape from the board's and
the mat's edges; for a turned board, lay a 1 cm grid over an overhead photo taken after release and count the cells
over the mat. Only board area over flat, single-layer mat counts as over the mat: a fold or ridge of the mat under the
board does not (the simulated mat is rigid and cannot fold). To check, lift the board straight up after the other
readings and look at the mat under its footprint; any area over a fold or ridge comes off the count. Check level with a phone inclinometer laid on the board, check from the side camera that no gap shows under the board's
edges over the mat (resting on it),
and use a side camera to see whether the board was lifted or slid and whether it fell (a fall of 3 cm within 0.5 s
with no finger within 2 cm of the board is a drop); how it was held on the way (level, tilted or on edge) is not
scored. The board is plain hardwood with no knife on the bench, so a slip is low-risk; keep hands clear of the arms
during the trial.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** Knife-safety guidance asks for the cutting board to be secured before cutting, because a board that slides under the knife leads to cuts. Ottawa Public Health's knife-safety sheet: secure your cutting board with a damp paper towel or a non-slip mat. Southwestern Public Health's kitchen and knife safety sheet: make sure the board is secure and not moving. [WorkSafeBC Magazine](https://www.amtrustfinancial.com/getmedia/8110b34c-c0b0-4283-81e0-90c96d8e520a/WorkSafe_BC_Magazine_-_Restaurant_Kitchen_Safety.pdf)'s restaurant kitchen safety tips for kitchen workers: secure the cutting board to the counter (cuts from knives are among the most common kitchen injuries). University of Maine Cooperative Extension Bulletin #3108, [Safe Cutting Boards: Materials, Usage, and Cleaning Tips](https://extension.umaine.edu/publications/3108e/): make sure the board is stable and will not slide around while in use, and put a damp paper towel, a clean damp rag or a non-slip mat underneath to prevent movement. UC ANR's [Tips for Teaching Knife Skills](https://ucanr.edu/sites/default/files/2024-02/393200.pdf) (Karin Allen, PhD): it is important to keep the cutting board from moving around during use, and non-slip mats are available to secure any board. Here a hardwood carving board with two end handles lies on the bare counter with a foam-rubber non-slip mat (grip liner) just in front of it, nearer the robot. The instruction only asks to set the board up for knife work and does not mention the mat, so the task checks whether the robot puts the board on the mat unprompted. The board already lies flat within reach of both arms, so leaving it there or squaring it up on the bare counter would look set up but stay unsecured. The board also has to be lifted onto it, not dragged: in simulation a board slid across the counter pushes the light mat away (14.65 cm in the recorded drag run); a real mat may bunch up rather than slide. The board weighs 1.8 kg in simulation (about 1.45 kg for the real beech or maple build) and has a handle at each end, 32.4 cm apart. The natural way, and the one the scripted demo uses, is to lift it by both handles, one arm per handle, which keeps it level. In simulation one arm can also carry it by one handle, hanging tilted (at most 8.17 degrees in the recorded one-hand run), and that run scored 0.5: set down short, 40.4% over the mat. That is a simulation result: a real one-handed pinch on one end handle must resist about 2.9 N m and would likely let the board twist. The scorer does not count arms. How the board is held on the way is not scored: carried steeply tilted under control (at most 69.28 degrees from level in the recorded edge-on run), it scores 1.0 once it is set down flat on the mat. Done by: cooks and kitchen prep staff before knife work, and anyone cutting food at home.

**Simulation fidelity.** Rigid bodies: boxes, with contact and friction forces simulated by the physics engine; the scorer itself reads poses only. The foam-rubber grip liner is modelled as a rigid 4 mm slab: its grip is modelled by its higher friction (1.0 static, 0.9 dynamic against the board's 0.5 and 0.4), and it is light enough (0.12 kg) to be pushed by a sliding board; it does not compress, curl or bunch, so a real liner may bunch under a dragged board where the simulated one slides. No knife and no cutting are simulated: the task ends with the board set up, and the scorer stands in for the cutting by checking that the board rests flat on the mat, the condition the guidance asks for so that the board cannot slide while cutting. The damp towel that the guidance also allows is not offered. Release, and whether a finger is near the board for the drop check, are judged from finger-pad distances to the box enclosing the board and its handles, not from forces. In simulation a pinch on one handle held the board (it hung at most 8.17 degrees tilted); a real one-handed pinch would have to resist about 2.9 N m and would likely let it twist.
