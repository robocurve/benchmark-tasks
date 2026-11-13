# hose-clamp-barb

**Instruction:** "Connect the hose to the pump's outlet and clamp it."

A coolant pump is bolted to the bench with its brass hose barb pointing at the right arm. A rubber
hose lies in front of it with a worm-drive clamp run down loosely near its end, and a stubby 8 mm nut
driver stands in a holder.

The hose has to be lifted to the barb, lined up with it and pushed over the barb's bead all the way
to the outlet boss: a hose that stops on the bead blows off under pressure. The clamp, run down near
the hose's end beforehand, rides on with it, so pushed home its band sits on the barb's shank behind
the bead. Its screw is then driven tight with the nut driver: the socket seated on the 8 mm head, the
driver turned clockwise, let go, the hand turned back and the driver gripped again, without knocking
the socket off the head. The screw needs more than one turn of the wrist. Then the driver comes away
and the hands stand clear.

## Scene

Positions are in the station frame (m): x away from the arm bases, y to the left, bench top z = 0.

| object | type | size / mass | pose |
|---|---|---|---|
| pump | fixed, painted steel | casting 110 x 100 x 110 mm; base 150 x 130 x 12 mm (centred 10 mm further +y); motor housing 80 mm dia, 50 mm long, behind | casting centre (0.28, 0.085) |
| outlet boss + barb | fixed, brass | boss 26 mm dia, front face at y 0.025; barb 34 mm long: 11.2 mm shank, 13.6 mm bead 10 mm from the tapered tip | axis along -y, 60 mm above the bench at x 0.27; tip at y -0.009 |
| hose | rigid tube (rubber) | 12.2 mm bore, 19 mm OD, 200 mm, 55 g; white mark 34 mm from its open end (level with the barb's tip when home) | lying along y, open end at (0.27, -0.12) facing the barb |
| worm-drive clamp | stainless | 9 mm band, 8 g; screw housing 9 x 11 x 13 mm on the -x side, 8 mm hex head up | band centre 12 mm from the hose end, run down till it touches |
| nut driver | rigid | 8 mm socket (14 mm OD, 16 mm long, 9 mm deep bore), 20 mm shaft, 30 x 30 x 48 mm square grip, 90 g | socket-down in the holder |
| driver holder | fixed | 40 x 40 x 40 mm block, 16 mm square hole | (0.20, 0.20) |

## Scoring

Measured on the hose's and the clamp's poses, the screw's turns and the hands' TCPs:
- **0.25** the hose pushed over the bead: at least 12 mm onto the barb, lined up with it.
- **0.5** pushed fully home: the hose end within 3 mm of the outlet boss (the white mark then level
  with the barb's tip).
- **0.75** also the clamp's band wholly on the barb's shank behind the bead (band centre 6.5 to 17.5 mm
  from the hose end; the bead is 22 to 26 mm in) and its screw run down snug (0.4 turn: the band grips
  the hose).
- **1.0** also tight (0.8 turn), the driver's socket at least 20 mm off the screw and both hands at
  least 6 cm from it, held for 1 s.
- Snugging the clamp anywhere else (off the barb's shank, or on a hose that is not home) caps the
  score at 0.5.

In a real trial: measure the gap between the hose end and the boss (or the mark against the barb's
tip), the band's position with a rule, check snug as the band no longer turning on the hose by hand,
and tight as 3 N m on a torque screwdriver.

Run limit: 1,600 control steps (80 s). The scripted demo's plan is 920 steps; it reaches 1.0 and the
trial ends on success at step 795 (40 s).

## Scripted demo

The right arm takes the hose 100 mm back from its end, lifts it to the barb's height, lines it up
and pushes it over the bead to the boss, lets go and stands clear. The left arm takes the nut driver
from its holder, holds it over the screw while it lines the socket up with the head (reading the
measured socket-to-head offset, as a scripted demo may), lowers the socket over the head, then turns
the driver clockwise half a turn of the wrist, lets go, lifts the open hand a little, turns it back
and grips the driver again below the middle of its grip: three strokes in all, about 0.3 turn of the
screw each (the open hand drags it back a little as it turns). It then lifts the driver off, stands
it back in its holder and stands clear. The two arms work one after the other.

## Simulation approximations

- **Hose:** a rigid octagonal tube. The bead is drawn only, and the barb's collider is a slimmer plain
  pin and cone (9.6 against 11.2 mm) so a rigid bore can go on where a rubber one would stretch. The
  rubber's grip is a slide joint along the barb, switched on once the hose is lined up with it
  (centre within 3 mm, axis within 12 degrees) and 6 mm on; from then it holds the hose on the barb's
  axis, with a damping drive for the drag going on (about 3 N at 12 mm/s, far less than a real hose's
  push-on force), a spring holding it home once pushed within 4 mm of the boss, and a stiff hold once
  the clamp is tight.
- **Clamp:** holds its place on the hose by a stiff slide joint (it is run down till it touches before
  the trial), so in simulation it always rides home with the hose; the band-position condition guards
  the real trial, where it can slip. Its band is drawn standing off the hose until the screw has turned
  0.25 turn, then pulled down on it; from 0.5 turn it holds the hose on the barb firmly.
- **Screw:** turns only with the socket seated over the head (centred within 2 mm, rim 2 to 10 mm down
  it, shaft within 12 degrees), by the driver's measured twist against the clamp body; turned the
  wrong way it loosens. Snug and tight are turn counts (0.4 and 0.8 turn) standing in for the feel and
  the 3 N m of a real trial. The head is a round 8 mm boss; there is no hex lock, nothing limits
  over-tightening, and no reaction torque is put on the driver as the clamp tightens.

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
and the task's own measured quantities (screw turns).

## Files

`task.py` (scene, hose/clamp/screw model, scripted demo and its controller, which steers the left
hand by the measured socket-to-head offset), `scoring.py` (stateful
scorer), `hose.py` (geometry and measurements), `simkit.py` (compound rigid parts, joints and the
spline demo controller). No third-party assets. No hardware validation has been performed.
