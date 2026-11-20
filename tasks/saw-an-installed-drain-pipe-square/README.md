# Saw through an installed drain pipe at its mark, square

One task file, `saw_drain_pipe.py`, for the `yam_arena` embodiment of `yam-isaac-lab-arena` on Dream Labs' station
scene, with its cut model `cut_model.py`, the model's simulator hook `cut_runtime.py` and the shared compound-part
helper `yam_parts.py`.
A 2-inch PVC drain line lies strapped to two 2x6 blocks, with a pencil line round it and a 1/2-inch PEX water line
running below it, under the blade's path, its top 32 mm (1 1/4 in) below the drain. A compact 12 V one-handed
reciprocating saw lies on a rest beside it. The robot takes the saw up by its grip, starts it clear of the pipe, sets
the running blade on the line, feeds it down through the pipe square to it, catches it as it breaks through before the
blade reaches the water line, lets go of the trigger and lifts the blade out of the cut.

## Why this task matters

**It is core work in occupations of over two million US workers.** O*NET lists sawing and repairing pipe as core
tasks of two occupations (employment and task statements from O*NET OnLine, 2025):

| Occupation | US workers | Core O*NET tasks it draws on (importance, 0-100) |
|---|---|---|
| [Plumbers, Pipefitters, and Steamfitters, 47-2152](https://www.onetonline.org/link/details/47-2152.00) | 510,600 | "Cut, thread, or hammer pipes to specifications, using tools such as saws, cutting torches, pipe threaders, or pipe benders." (78); "Maintain or repair plumbing by replacing defective washers, replacing or mending broken pipes, or opening clogged drains." (66); "Inspect structures to assess material or equipment needs, to establish the sequence of pipe installations, or to plan installation around obstructions, such as electrical wiring." (67) |
| [Maintenance and Repair Workers, General, 49-9071](https://www.onetonline.org/link/details/49-9071.00) | 1,621,800 | "Repair machines, equipment, or structures, using tools such as hammers, hoists, saws, drills, wrenches, or equipment such as precision measuring instruments or electrical or electronic testing devices." (72); "Assemble, install, or repair wiring, electrical or electronic components, pipe systems, plumbing, machinery, or equipment." (69) |

**It is the job as it is really done.** Adding a fixture, fitting a tee or replacing a cracked length of drain all
start by cutting a line that is already installed: strapped in place, with the supply lines that run beside it, not
clamped on a bench. A tubing cutter clamps on and is turned round a pipe; a cut-in on installed drain is made with a
compact one-handed reciprocating saw, carried and guided by hand, and this task uses that tool and that setting. Every
requirement in the scoring is one a tradesperson is held to:

- **On the mark** (3 mm): the gap left must fit the fitting going in; an off-line cut leaves it the wrong length.
- **Square** (2 degrees): a crooked end does not seat fully in a solvent-weld socket.
- **One clean kerf**: a skated or restarted blade leaves a second cut in a pipe that stays in service.
- **Caught at breakthrough**: when the last wall goes, the support under a pushed saw vanishes and it lunges. Here a
  1/2-inch PEX supply line runs under the blade, 32 mm below the drain: a nicked supply line leaks under pressure.
- **Stopped and lifted out**: by the end, the trigger let go, the blade out of the cut and the saw held clear of the
  table.

**It tests what robots cannot yet do with tools.** Passing needs, in one continuous action with one hand:

- carrying a 1.55 kg, weight-forward running tool (in simulation it loads the YAM's wrist to about 75% of its torque);
- placing its blade within 3 mm of a line and holding it within 2 degrees while it cuts;
- regulating a light feed force (the cut starts at 3 N on the blade and reaches full speed at 10 N; a heavy start lets
  the blade wander off its line, and the harder the push when the last wall goes, the further the saw lunges);
- reacting within a fraction of a second when the cut goes through. In simulation, a plan that re-planned every quarter
  second let the blade plunge 87 mm below the drain, through where the water line runs; re-planning every 50 ms held
  it to 21 mm.

These are the skills of hand-held power-tool work in general (saws, drills, grinders, drivers on installed work), where
the robot must control a running tool through material rather than place it. A robot that passes this task has shown
force-controlled, position-accurate use of a running power tool on installed work, with the breakthrough control that
keeps the job, and the services around it, intact.

**Why the setting is this way.** Drain lines run strapped to framing or blocking, with supply lines beside and below
them; no plumbing code sets an indoor spacing between the two, so they run anywhere from touching to inches apart. The
32 mm (1 1/4 in) gap is fixed for reproducibility: room for a saw that is caught, not for one that plunges.

## Requirements

- `yam-isaac-lab-arena` installed with its `setup.sh` (Isaac Lab-Arena, `inspect-robots` and `inspect-robots-yam`),
  or the Robocurve Brev launchable; a Dream API key in `~/.dreamlabs/credentials` for the Dream-1 run.
- No third-party assets: every object is a primitive (boxes and cylinders) defined in the task files.

## Run a trial

From this folder, with 20 Hz control:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=saw_drain_pipe.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Cut through the drain pipe at the pencil mark with the reciprocating saw, square to the pipe, without the blade touching the water line below it, then let go of the trigger and lift the blade out of the cut." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/pipe-saw-dream

# The scripted demo (full score): the same command with
#   --policy yam_scripted --log-dir ~/yam-eval/pipe-saw-demo
# (its video was recorded at -E cam_width=640 -E cam_height=352, so the kerf shows)

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/pipe-saw-*/*.json; do inspect-robots video "$log"; done
```

If the machine has a desktop display that Isaac Sim can't open an OpenGL context on (`GLXBadFBConfig` at startup),
run with `DISPLAY` unset (`env -u DISPLAY inspect-robots run ...`); the runs are headless. `CUT_DEBUG=1` prints the
cut model's pushes and the saw's pose every few physics substeps.

## Score

`--scorer task_score` gives 1.0 when the task succeeds (held for 1 s) and otherwise the task file's `score()` at the
end of the trial. The rules are in `task.json` and in the file's scoring section:

| Credit | Condition | Check on a real trial |
|---|---|---|
| 0 | the blade touching the water line at any time, the kerf more than 3 mm from the pencil line along the pipe, or more than 2 degrees out of square, or a second kerf 1 mm deep or more anywhere | the trial's video watched for the blade meeting the PEX, and the PEX looked over for a nick or scuff; a tape from the near block's centre to the kerf; a square held on the pipe across the kerf (or on the cut end): 2 mm of light over the pipe's 60 mm |
| 0.8 x share | the share of the pipe's outside circumference the kerf has gone round | a flexible tape laid round the pipe along the kerf |
| + 0.2 | cut through, the trigger let go, the blade lifted out (its teeth 10 mm above the pipe's top, or wholly to one side of it) and the saw not put down on the table (its lowest point 30 mm or more above it) | the pipe in two; the saw stopped and held clear |

In simulation the trial scores 0 if the saw's pose is not finite or its rotation not unit length.

The scoring, the cut model, the layout and the demo's reach can be checked without the simulator:

```bash
python -m pytest -q test_saw_drain_pipe.py
```

## Runtime and seeds

- Run limit: 1,800 control steps at 20 Hz (90 s).
- The scripted demo (`demo()`) uses the right arm only; the left arm stays at its ready pose. The right hand takes
  the saw off its rest, carries it over the pipe and squares it over the line from three reads, pulls the trigger
  with the blade clear of the pipe, reads once more just over the line, sets the running blade on it, leans on it
  gently (4 N) until the teeth are 3 mm in, then follows the cut down at 8 N (4 N through the last wall) a
  control step (50 ms) at a time; once the pipe is cut through it stops pushing and brings the saw back up under the
  pipe (the blade drops about 21 mm below the drain as it breaks through, 11 mm short of the water line, which it never
  touches), lets go of the trigger and lifts the blade out. The plan lasts 60 s; the
  trial reached 1.0 and ended as a success at step 1,142 (57 s of simulated time). On an RTX 5080 a trial took
  about 3 minutes of wall-clock time, startup included.
- Dream-1 (`--policy dream`, 320 x 176 cameras): 0.0. Over the full 1,800 steps it reached for the saw on its rest
  several times but never lifted it, so no cut was started.
- No randomness: the layout is fixed (`-E object_jitter=0`), so no seed is needed.

## How the files work

- **The cut** (`cut_model.py`). PhysX cannot cut a solid, so the blade's collider is set not to collide with the pipe
  (a PhysX filtered pair, authored when the saw is spawned; the blade still meets everything else). It rests instead
  on a small kinematic pad (6 x 4 x 4 mm, hidden in the pipe) that the model moves, before every physics substep,
  under the uncut pipe the straight blade meets first, so PhysX itself takes the blade's push (a spring applied by the
  model chattered against the hand's grip). The model keeps each kerf's floor in 49 strips across the pipe. The push
  on the pad, read from PhysX contacts, is the feed: the floor sinks where the blade bears, only while the trigger is
  pulled, the blade lies within 5 degrees of its kerf and the feed is at least 3 N, faster with more feed, up to
  15 mm/s from 10 N (offline, 10 N takes 3.9 s through the pipe; a 12 V saw cuts 2-inch plastic pipe in about 4 s).
  Where a wall is cut through, the pad drops to the far wall, and the blade with it. A running blade pressed on the
  pipe's outside with 2.5 N starts a kerf in its own plane; anywhere off that kerf, it starts a second one. Below the
  pipe's surface a blade pushed sideways out of its kerf is pushed back by a spring (the kerf's sides). While it cuts,
  the work drags the blade toward its tip with 0.4 times the feed (the teeth face the saw and cut on the pull
  stroke), which the shoe, against the pipe, takes.
- **The water line.** A 1/2-inch PEX line (15.9 mm outside) runs the pipe's length below the drain, its axis 60 mm
  left of the drain's (under the blade's outer half, where the top camera sees it), through the blocks (a 7/8-inch hole
  bored in each, on a real rig), its top 32 mm below the drain's bottom. It is an ordinary
  solid the blade meets (kinematic, as the drain is). The contact view that reads the feed also reports the blade
  pressing on it, and any touch in the trial (0.01 N or more) scores 0: a running blade nicks PEX. Without control at
  breakthrough the saw drops far below the pipe (87 mm when the demo re-planned only every quarter second), so the
  task tests catching a power tool as the cut goes through, not only cutting.
- **The trigger** (`cut_runtime.py`). The paddle trigger is a red block standing 6 mm proud of the grip's underside.
  The hand holds the grip from behind, one pad on its top and one on the trigger. The trigger counts as pulled while
  PhysX reports the robot pressing its face with at least 35 N (a rigid contact view, as in
  `tasks/move-live-wire-with-stick`): carrying the saw, the pads 4 mm in, the trigger takes ~28 N (the squeeze plus
  the saw's weight resting on it); squeezed, the pads 9 mm in, ~48 N. A real paddle trigger takes about 10-20 N; its
  travel is not simulated.
- **The kerf** shows as small dark marks (kinematic, without colliders, 3 mm wide so the cameras see them) moved onto
  the pipe's outside, in the kerf's plane, as the cut passes each point of its circumference.
- **Carrying the saw in the demo.** Each arm joint is driven as a spring (80 N m/rad at the shoulder, 10 at the
  wrist) with no feed-forward for a load, so the 1.55 kg saw would sag the hand some 15 cm and 20 degrees. Every pose
  the hand holds the saw at is led by K^-1 J^T W in the joints (W: the saw's weight and its moment about the grip, and
  the push on the blade while cutting), and the demo reads the saw's pose at fixed points to square it over the mark.
- **Compound parts.** The task-file API has boxes, spheres and upright cylinders, so the saw, pipe, straps and marks
  are `yam_parts.Part`s, from the repository's shared helper, copied unchanged.
