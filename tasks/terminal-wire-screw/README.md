# terminal-wire-screw

**Instruction:** "Land the stripped wire in the empty terminal and tighten it."

A control-panel mounting plate lies flat on the bench on two battens, as panels are wired on the
bench, with five 35 mm2 screw-clamp terminal blocks on a DIN rail. Four have their conductors landed;
the middle one is empty. The panel's incoming 35 mm2 conductor comes up out of the cable duct beside
it, stripped 18 mm, and springs back off its line. A screwdriver stands in a holder.

The stripped end has to be lined up with the empty block's clamp opening and pushed in until the
insulation butts on the block (no bare copper showing, no insulation under the clamp), then held
there, because the conductor springs back, while the other hand drives the clamp screw down tight
with the screwdriver: blade seated in the screw head, turned clockwise, let go, the hand turned back
and the grip taken again. Then the screwdriver comes away and the hands stand clear.

## Scene

Positions are in the station frame (m): x away from the arm bases, y to the left, bench top z = 0.

| object | type | size / mass | pose |
|---|---|---|---|
| mounting plate | fixed, galvanised steel | 300 x 440 x 3 mm on two 40 mm timber battens | centre (0.23, -0.08) |
| DIN rail and terminal row | fixed | TS35 rail along x; five 35 mm2 blocks 15.2 x 60 x 52 mm between end stops | rail at y 0.03; empty block at x 0.23 |
| clamp opening (empty block) | | 9.5 x 9.5 mm, 22 mm deep, centred 15 mm above the rail | entry face at y 0, facing -y |
| clamp screw | turns about z | M6 slotted: head 10 mm, slot 1.2 mm wide (drawn as a flared round recess 3 mm deep), 14 mm down a bore (8 mm lip, 11 mm below) | 12 mm in from the entry face |
| cable duct | fixed, slotted | 40 x 50 mm along x | y -0.26 |
| incoming conductor | sprung at the duct exit | 35 mm2: 11 mm over insulation, 7 mm copper, 220 mm of insulation, stripped 18 mm, 65 g | leaves the duct at (0.23, -0.245); rests about 8 degrees off line toward the arms and 8 degrees drooping, insulation ending 29 mm short of the block |
| screwdriver | free | slotted, 1.0 x 6.5 mm flat tip (drawn as a 6.4 mm round tip), 7 mm round shaft 45 mm long, 30 x 30 x 50 mm square grip, 90 g | tip-down in a 40 mm holder at (0.18, 0.20) |

### Screw and screwdriver

The real part is a 35 mm2 feed-through screw-clamp block such as the Phoenix Contact UT 35: an M6
slotted clamp screw (10 mm head, 1.2 mm wide slot), rated 3.2-3.7 N m, 18 mm strip length. The tool is
the matching slotted screwdriver, 1.0 x 6.5 mm flat tip to ISO 2380-1 (for example Phoenix SZS
1.0 x 6.5), here a stubby one with a 7 mm round shaft. The screw sits 14 mm down an access bore whose
top lip narrows to 8 mm: the 6.5 mm tip and 7 mm shaft pass it with about 0.5 mm a side, so the lip
centres the tip over the slot, and the 11 mm bore below clears the 10 mm head. The tip's 1.0 mm
thickness enters the 1.2 mm slot once it is turned into line with it.

## Scoring

Measured on the conductor's and screwdriver's poses, the clamp screw's turns and the hands' TCPs:
- **0.25** the bare end lined up with the opening and at least 8 mm into the clamp.
- **0.5** fully in: the insulation within 2 mm of the block face (no bare copper showing).
- **0.75** also the clamp screw run down onto it (0.3 turn).
- **1.0** also tight (0.6 turn), the screwdriver's tip at least 20 mm off the screw and both hands at
  least 6 cm from it, held for 1 s.
- Running the clamp down on nothing, on a short-inserted end or on a misaligned conductor caps the
  score at 0.5.

In a real trial: check the insulation against the block face (no copper showing), clamped as the
conductor not pulling out by hand, and tight as about 3.5 N m on a torque screwdriver.

Run limit: 1,600 control steps (80 s). The scripted demo reaches 1.0 and the trial ends on success at
step 676 (34 s).

## Scripted demo

The right arm takes the conductor 130 mm back from its stripped end, lifts and swings it into line
with the opening (steering by the measured position of the bare end, as a scripted demo may), pushes
it in until the insulation butts on the block and holds it there. The left arm takes the screwdriver
from its holder, brings it over the screw (steering by the measured tip-to-screw offset), seats the
blade in the screw head, turns it clockwise about 165 degrees, lets go, opens the hand fully, turns it
back and grips again over the blade: two strokes. It lifts the screwdriver off; the right hand lets go
of the clamped conductor and both stand clear.

## Simulation approximations

- **Conductor:** a rigid rod from the duct exit on a sprung slide (the slack it can be drawn out by,
  about 1 N home) and a sprung swivel, so it springs back toward its rest line when let go. It does
  not bend.
- **Clamp:** the screw turns only with the blade's tip seated in its head's recess (within 2 mm of the
  axis, down on the recess floor, blade within 12 degrees of vertical), by the screwdriver's measured
  twist about its own axis; turned back, it loosens. From 0.3 turn, with the bare end 8 mm or more in,
  the conductor is held where it is by stiffening its joints; the clamp cage itself is drawn, not
  moved. Clamped and tight are turn counts (0.3 and 0.6 turn) standing in for the feel and the
  3.5 N m of a real trial. No reaction torque is put on the screwdriver.
- **Screw head and tip:** the slot and the flat tip are drawn and collided round (a flared round
  recess and a 6.4 mm round tip), so the tip seats at any angle and cannot cam out; lining the blade
  up with the slot is not required.

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

`task.py` (scene, conductor/clamp model, scripted demo and its controller, which steers both hands by
measured offsets), `scoring.py` (stateful scorer), `terminal.py` (geometry and measurements),
`simkit.py` (compound rigid parts, joints and the spline demo controller). No third-party assets. No
hardware validation has been performed.
