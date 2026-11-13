# carton-tape-seal

**Instruction:** "Tape the carton shut."

A filled shipping carton stands on the bench with its two long top flaps folded down over the short
end flaps, but the board's memory makes them spring back up about 10 degrees, so the seam between
them gapes. Beside it stands a hand tape gun loaded with 48 mm packing tape.

One strip of tape has to go along the seam with the flaps flat under it, lapping down both end faces
as tabs, and be torn off. The gun is set down at the near end (its free tape drops down the near end
face) and run along the seam with the roller pressed on. Run on past the far edge, it is lifted so the
stretched tape tears on its serrated blade, and the hanging ends are smoothed down the end faces.

## Scene

Positions are in the station frame (m): x away from the arm bases, y to the left, bench top z = 0.

| object | type | size / mass | pose |
|---|---|---|---|
| carton | free, corrugated board 4 mm | 160 x 160 x 50 mm outside, filled: 2.5 kg (plus its flaps) | centre (0.25, -0.05) |
| short end flaps | board, part of the carton | 60 x 152 mm each, folded in flat along the near and far ends | under the long flaps |
| two long top flaps | board, spring hinges along the left and right top edges | 160 x 79 mm, 30 g each | folded down, sprung up about 10 degrees; seam along x at y -0.05 |
| tape gun | free | 350 g; red steel frame, 80 mm roll of 48 mm tan tape, 22 mm rubber roller on a free axle at the front, serrated blade just ahead of it, 34 x 28 x 45 mm grip behind the roll; 75 mm of tape free past the roller | standing on roller and foot at (0.17, -0.22), front facing +x |

## Scoring

Measured with a rule along the strip:
- **0.25** tape stuck along at least half the seam.
- **0.5** stuck along the whole 160 mm of the seam, its centreline within 10 mm of the seam everywhere,
  and torn off the gun.
- **0.75** also tabs stuck at least 24 mm down both 50 mm end faces.
- **1.0** also both flaps flat (neither more than 3 degrees open where tape went on, nor at the end),
  the gun let go of (no closed hand within 10 cm of it) and both hands clear of the carton, held 1 s.

Run limit: 1,600 control steps (80 s). The scripted demo takes 558 steps (28 s).

## Scripted demo

The left arm presses the flaps down at the far half of the seam (as a packer does; the roller pressing
the rigid flaps would also close them, see below). The right arm picks the gun up by its grip, sets
the roller down on the seam 10 mm from the near edge and runs it along the seam in eight steps,
pressing the roller 1.5 mm into the board. Once the tape holds the flaps, the left hand lets go and
moves round the left of the carton, out of the gun's way. The gun runs on 40 mm past the far edge and
is lifted, tearing the tape, then is stood back on the bench and let go. The left hand smooths the tail
down the far end face from the edge, goes round to the near end and smooths the hanging tape down it,
and stands clear. The plan is about 30 s; the trial ends on success when the score has held 1 s.

## Simulation approximations

- **Tape** is not a film. The strip is 30 cells of 8 mm (5 down each end face, 20 along the seam).
  A cell sticks where the measured roller axle runs within 3 mm of its working height over it. Cells
  the roller passed while lifted bridge, and the free ends hang; both are drawn 4 mm off the board. A
  hanging cell on an end face sticks when a fingertip (pad within 15 mm of the face) is on it and the
  cell between it and the edge is already down, so a tab is smoothed on from the edge outward. The
  strip's offset across the seam is recorded per cell. There is no adhesion force, tension or
  wrinkling.
- **Tearing** is a position rule, not blade contact: run on 8 mm past the far edge and lifted 12 mm,
  the tape tears, leaving a tail up to 60 mm long (the blade's distance from the edge, capped) hanging
  down the far end.
- **Flaps** are rigid boards on spring hinges (0.004 N m per degree toward 18 degrees open; they sit at
  about 10 degrees under their own weight). Being rigid, a flap pressed anywhere along the seam closes
  along its whole length, so the roller alone can hold them flat as it passes; a second hand holding
  them is how packers work, not something the simulation enforces. Once three cells are stuck the
  hinges' targets are set closed: the tape holds them. A flap standing open more than 3 degrees when a
  cell went on is recorded as a gap, and the flaps' angles are checked again at the end.
- **Roller:** a separate rubber body on a free-turning axle, so it rolls along the board (yellow end
  caps and white stripes show it turning).
- **Carton size:** a small 160 x 160 x 50 mm parts carton, within the bench reach of the arms; the
  sealing method is the same as on larger cartons.

## Reproduction

`prepare` exports the runtime into `runtime/`. From this task directory, with the
`yam-inspect-isaac:6.1.0` image (or `python3 runtime/build_image.py --image yam-inspect-isaac:6.1.0`):

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

Seed 0; the scene settles for 30 ticks after reset before control starts. Control is 20 Hz and
physics 240 Hz. Each episode writes four 20 fps camera videos, setup/final PNGs, the measured state
trajectory and result.json. The scorer reads only measured object poses, robot TCP/gripper state
and the task's own measured quantities (tape cells).

## Files

`task.py` (scene, tape model, scripted demo), `scoring.py` (stateful scorer), `tape.py` (geometry and
measurements), `simkit.py` (compound rigid parts, joints and the spline demo controller). No
third-party assets. No hardware validation has been performed.
