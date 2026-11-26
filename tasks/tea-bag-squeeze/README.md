# tea-bag-squeeze

**Instruction:** "Take the tea bag out of the mug: squeeze it against the inside of the mug with the
teaspoon first, then put the bag and the spoon on the plate."

A mug of brewed tea stands on a sheet of white paper with the tea bag still in it and a teaspoon standing
in it. The bag's string runs over the rim and once round the mug's handle, so its paper tag hangs beside
the handle. A side plate lies beyond the mug.

It takes both hands at once:

1. the left hand pinches the tag, frees the string from the handle, and draws the bag up the mug's left
   inside wall by its string until the bag's top is just below the rim, and holds it there;
2. the right hand takes the teaspoon by its handle, stands it up in the mug and presses the bag against
   the wall with the spoon's bowl: firmly enough to squeeze the bag out, gently enough that the mug
   (620 g, free on the paper) neither slides nor tips;
3. the left hand lifts the squeezed bag out on its string, carries it to the plate without a drop falling
   on the paper, and lays it down;
4. the right hand draws the spoon out and lays it on the plate too.

A bag taken out without squeezing drips across the paper on its way to the plate.

## Scene

Positions are in the station frame (m): x away from the arm bases, y to the left, bench top z = 0.7475.

| object | type | size / mass | pose |
|---|---|---|---|
| paper | drawn on the bench | 460 x 460 mm, white | centred (0.29, 0.02) |
| mug | free, glazed stoneware | 94 mm across, 78 mm tall (86 mm bore, 72 mm deep), handle 36 mm out to the left; 620 g with its 220 ml of tea, 38 mm deep | centred (0.22, 0.00) |
| tea bag | free | wet pillow 52 x 46 x 9 mm, 14 g | standing in the tea against the left inside wall |
| string + tag | free | 105 mm string from the bag's top to a paper tag 26 x 30 mm, 2 g | tag hanging beside the handle at (0.22, 0.099), top edge 73 mm above the bench |
| teaspoon | free, stainless | 130 mm: bowl 30 x 42 mm, handle 10 x 4 mm; 28 g | standing in the mug, leaning on the rim to the right, 35 degrees from upright |
| side plate | fixed | 190 mm across, 6 mm high | centred (0.395, 0.00) |

Both arms work, each seen by its own wrist camera, and the whole scene is in the overhead camera's view.

## Scoring

- **0.25** the tea bag lies at rest on the plate.
- **0.5** also it was squeezed out first: pressed between the teaspoon's bowl and the inside of the mug,
  over the mug, for 1 s in all, and no gripper finger ever touched the bag.
- **0.75** also no drop of tea fell on the paper, and the mug stayed put (moved less than 10 mm, never
  tilted more than 10 degrees).
- **1.0** also the spoon lies at rest on the plate (its bowl over the plate) and both hands are 8 cm or more
  from the bag, its tag and the spoon, held for 1 s.
- **0**, whatever else, if the mug slid more than 30 mm or tilted more than 30 degrees at any time (the tea
  spilled). Otherwise 0.

In a real trial: the video shows the squeeze and that the fingers stay off the wet bag; drops are counted
on the white paper; the mug's shift is measured with a rule from its outline drawn on the paper.

Run limit: 1,800 control steps (90 s).

## Recorded runs (seed 0)

- **Scripted demo:** 1.0, the trial ending on success at step 610 (30.5 s): squeezed for 3.45 s at
  1.6 to 1.9 N, no drop on the paper, the mug moved 4 mm. Replaying its recorded trajectory through the
  scorer (`runtime/rescore.py`) gives the same score.
- **Control:** the same demo with the squeeze left out (the spoon is brought up to the bag but not pushed
  on) ends at 0.25: the bag reaches the plate, but 4 drops fall on the paper on the way.
- **Dream-1** (`dreamlabs/dream-1-beta`): 0.0 after 225 steps (11.25 s), when the run ended with
  `DreamSessionExpired:expired`; a first attempt ended the same way at step 177.
- Doing nothing scores 0.

## Scripted demo

The left hand comes down on the hanging tag with its wrist leaning out to the left, pinches it, lifts it
off the handle and moves it up and over until the string, taut over the rim, has drawn the bag up the
wall; it holds the tag there. Meanwhile the right hand takes the spoon by the top 5 cm of its handle,
fingers along the handle and jaws on its flat faces, stands it up in the mug with its wrist leaning back,
brings the bowl up to the bag and pushes it on until the measured force of the spoon on the bag is 1.9 N,
holding that for about 3 s. It eases off; the left hand lifts the bag out, carries it over the far rim to
the plate, lowers it until it stands, lays it over and lets the tag go; the right hand then draws the
spoon out, carries it across over the mug, lowers its bowl onto the plate and lets it fall flat.

The demo reads measured poses to find the tag and the spoon, the measured force of the spoon on the bag
to regulate the squeeze, and each arm's measured TCP to trim out its sag, as a scripted demo may. It never
moves an object except through the grippers.

## Simulation approximations

- **Squeezing.** The wet bag is a rigid pillow. A PhysX rigid contact view on the bag gives the real
  contact force of the spoon, the mug and each finger pad on it. The bag is being squeezed while, over the
  mug, the spoon presses on it with at least 1.5 N and the mug's wall presses back with at least 1 N;
  after 1 s of that in all it counts as squeezed out. Lowered back under the tea it is wet again. A
  finger pad pressing on the bag with more than 0.2 N is a finger on the bag.
- **Dripping.** There is no liquid. A bag that is out of the tea and not squeezed out lets a drop fall
  from its lowest corner every 0.5 s (up to 40); a drop that falls outside the mug and the plate lands on
  the paper, is counted and is drawn there. A squeezed-out bag does not drip (a real one may still let a
  drop go).
- **String and tag.** The string is a 105 mm length limit between the bag's top and the tag's top: it
  cannot stretch and can hang slack, is drawn as beads, touches nothing, and pulls in a straight line (so
  it runs "over the rim" without rubbing on it). The tag starts hanging on a hinge beside the handle,
  standing in for the string's turn round the handle; the hinge lets go at 1 N.
- **Mug and tea.** The mug is a rigid body with the tea's weight in it (620 g); the tea is drawn as a
  surface and does not slosh or spill. Spilling is judged from the mug's slide and tilt.
- Rigid bodies throughout, PhysX at 240 Hz, control at 20 Hz.

## Reproduction

`runtime/` holds the exported runner. From this task directory, with the `yam-inspect-isaac:6.1.0`
image (or `python3 runtime/build_image.py --image yam-inspect-isaac:6.1.0`):

```bash
mkdir -p /tmp/rc-task-home/tmp ./outputs
docker run --rm --gpus device=0 --network host --shm-size 8g \
  --user "$(id -u):$(id -g)" -v "$PWD":/task:ro -v "$PWD/outputs":/outputs:rw \
  -v /tmp/rc-task-home:/tmp/task-home:rw -e HOME=/tmp/task-home -e TMPDIR=/tmp/task-home/tmp \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e OMNI_KIT_ACCEPT_EULA=YES -e ACCEPT_EULA=Y \
  --workdir /outputs --entrypoint /isaac-sim/python.sh yam-inspect-isaac:6.1.0 \
  /task/run.py --policy scripted --episodes 1 --livestream 0 --viz none
python runtime/rescore.py --task task.py outputs/TASK/RUN/episode-01/trajectory.jsonl \
  --expect outputs/TASK/RUN/episode-01/result.json
```

`--policy dream` runs Dream-1 instead (needs `DREAM_API_KEY` and a budget ledger mounted at
`/dream-budget`, made once with `runtime/dream_budget.py init`). On an arm64 host the base image has no
WebRTC streaming library: drop that `COPY`/`RUN` pair from `runtime/Dockerfile` and run the container as
the image's own user instead of `--user`. Seed 0; the scene settles for 30 ticks after reset before
control starts. One trial takes about 3 minutes of wall time on one GPU. Each episode writes four 20 fps
camera videos, setup and final PNGs, the measured state trajectory and result.json. The scorer reads only
measured object poses, the robot's measured TCPs, and the squeeze, drip and touch records the task
reports.

## Files

`task.py` (scene, squeeze/drip/string model, scripted demo and its two-arm controller), `scoring.py`
(scorer), `tea.py` (geometry and measurements), `simkit.py` (compound rigid parts and joints), `runtime/`
(runner). No third-party assets: every shape is built from primitives in `tea.py`. No hardware
validation has been performed.
