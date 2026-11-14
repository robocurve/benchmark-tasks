# Turn a humanoid's hung upper body round on its gantry hook so that its back faces you

The upper body of Asimov 1, Menlo Research's open-source humanoid (the torso shell, both arms hanging straight, and the head; 18.3 kg), hangs by its shoulder shackles from a bench gantry's swivel hook over the table, facing the arms. It is to be turned round so that its back faces the arms, and left still there. On the swivel it turns freely: the push that starts it loads it with momentum that the hook's drag takes well over a full turn to remove. To leave it at rest facing away, the push must be judged or the spin taken out by hand: a late catch overshoots, and a hard block is a blow to the hand and the robot.

In the demo the right hand pushes the torso's front near its left edge away; the body starts turning at about 45 deg/s, and the right pulls back out of its sweep. The left hand waits outside the sweep. When the body is 70 degrees from the mark, the left moves into the path of the body's left upper arm. It meets the arm's rear face 12 degrees before the mark and gives way with it, braking the spin. The body comes to rest within a few degrees of the mark, and the trial ends 1 s later.

**Why it matters.** Robots that build robots have to handle robots under assembly. Asimov 1 is built hanging from a gantry hook, and its build guide's step 10.6 reads: "Turn the robot around so we can now work on the back cover." The robot is turned round like this whenever its back is worked on: the battery, the charger port, the back cover. Here the upper body hangs alone on a bench gantry. It stands in for the whole hung robot, whose legs would not clear the table within the arms' reach. Fitting the rear cover, the step that follows, is left out.

**Ability tested:** turning a heavy body that turns freely on a swivel so that it ends at rest on a heading, without a blow: a judged push, a catch, or a turn by hand all count. A hard block, or a block the body bounces off, scores 0.

**Nearest tasks.** A hoisted motor lowered onto studs, where the hands correct its yaw as it lands. There the load hangs on a hoist and the yaw is set slowly; here the body spins freely, and the score rests on stopping it on the mark. This task is not a fallen-robot recovery: the robot is under assembly.

**Arms.** The instruction does not ask for two hands. In the demo the right starts the turn and the left stops it.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder, and `upper_body.npz` holds the body's meshes, colliders and mass properties. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 800 \
  --instruction "Turn the body hanging from the swivel hook half a turn, so its back faces you."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode turn                     # full score, with a video
python demokit.py task.py OUT --no-record --mode push,none     # the checks below (push runs to the 800-step limit)
python check_score.py                                        # the scorer on hand-built trials, with no simulator
```

The demo reads the swivel's angle (where the body is) and its rate from one step to the next, as a camera would. It reads nothing from the scorer.

## The scene and what is simplified

The upper body is one rigid body: the maker's URDF links for the torso shell, both arms and the head, merged at the zero pose with the arms rigid and hanging straight; a real trial locks the arms' joints straight. Its mass (18.28 kg) and inertia come from the URDF; its yaw inertia about the hook's axis, through its centre of mass, is 0.257 kg m2. Each link's convex hull is its collider. It yaws on a revolute joint at the hook. The joint's friction is PhysX's coefficient 0.005, so a free spin slows from 45 to 27.5 deg/s in 10 s and is still turning at 8 deg/s after 35 s; a real swivel hook's spin-down should be measured and matched. The shackles and chains are not modelled, so the body neither swings nor tilts.

The gantry is a beam on two posts, outside the body's sweep (its arms reach 21 cm from the hook's axis).

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. `check_score.py` feeds the real `Trial` hand-built states for the honest turn and each lazy or faulty route.

| What happened | Score |
|---|---|
| Turned 180 degrees net (within 10 degrees, either way round), still (under 1 deg/s) and staying there (where the hook's drag would carry it, the angle plus 20 s times the rate, also on the mark), for 1 s; past that, or a full turn more, does not count | 1.0 |
| Otherwise, turned through 90 degrees or more and stopped (under 5 deg/s) at some time | 0.25 |
| A blow, a collision, or left spinning over 20 deg/s at the end | 0 |

A blow: the spin slows or reverses by more than 25 deg/s within 0.1 s, about 11 N pressed 10 cm from the swivel. A collision: the spin changes by more than 5 deg/s within 0.1 s while the body turns faster than 10 deg/s, with no fingertip within 5 cm of it (a hand's fingers or palm on it). Spin changes are read over two steps, so a blow that falls between two samples still counts whole. Distances to the body are measured to its colliders.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 800 steps | 0.0: the model never touched the body, which hung still facing the arms to the step limit |
| `turn` | The whole task (the film) | 1.0, at step 204 (10.2 s): pushed to 45 deg/s, caught on its left upper arm 12 degrees before the mark, still 2.0 degrees short of it (178 degrees net) for 1 s; the spin's sharpest slowing was 16 deg/s within 0.1 s |
| `push` | The push alone, no catch, to the 800-step limit | 0.0: still turning at 8 deg/s after 774 degrees (never at rest) |
| `none` | The arms wait | 0.0 |

`check_score.py` also runs, on hand-built states: the same turn the other way round (1.0), caught with the palm (1.0), a hand left on the body (1.0), at rest 8 degrees short (1.0) or 12 degrees short (0.25), past the mark by 20 or 40 degrees and brought back (1.0), one and a half turns (0.25), stopped at 120 degrees (0.25) or 60 degrees (0), one push left to coast on the drag (24.7 deg/s: on past the mark; 10.5 or 11.5 deg/s: creeping through the mark and on), caught and let go on the mark still turning at 2.9 deg/s, two nudges coming to rest short of it (all 0.25), turned slowly by hand at 10 deg/s (1.0), braked at 14 degrees and then turned by hand to the mark (1.0), stopped by a collision with no hand near (0), stopped dead by a hand in one step or with the stop 30, 50 or 70 % into a step (0), still turning at 25 deg/s (0), on the mark only 0.9 s (0.25), a rigid block at 185 degrees with a 15 deg/s rebound at four points in a step, a 30 deg/s rebound, an elastic reversal, two hands held rigid either side of the mark with the body rattling between them (all 0), and nothing done (0).

## Runtime and seeds

The demo takes 10.2 s of simulated time (204 steps), about 1 to 2 minutes of wall time on one shared GPU at 640 x 360, start-up apart. There is no randomness: `object_jitter=0`, so no seed is needed.

## A real trial

Hang the upper body (or a stand-in of the same mass and yaw inertia, 18.3 kg and 0.26 kg m2, with the shell's outline) from a ball-bearing swivel hook on a bench gantry, so that its centre of mass is 35 cm above the table at (55, 0). First measure its free spin-down: start it at about 40 deg/s and time it. Put a contrasting mark on each shoulder for the camera above, and log yaw at 20 Hz or faster. Keep people out of the body's sweep (its arms reach 21 cm from the hook's axis), and check the hook's rated load.

## Sources

- Asimov 1's URDF, meshes and mass properties: Menlo Research, https://github.com/menloresearch/asimov-1, licensed CERN-OHL-S-2.0. `upper_body.npz` is derived from them and stays under CERN-OHL-S-2.0.
- Build step 10.6, "Turn the robot around so we can now work on the back cover.": Menlo Research's Asimov 1 build guide, https://docs.menlo.ai/asimov/1/build/assembly-steps/torso-module/battery-and-cover-installation/step-10.md
- All other meshes are generated by `task.py` and `mechkit.py`.

## Licence

MIT, except `upper_body.npz` (CERN-OHL-S-2.0, see Sources).
