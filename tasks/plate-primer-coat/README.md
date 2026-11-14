# plate-primer-coat

**Instruction:** "Give the steel plate a coat of primer."

A bare steel plate lies on the bench. Beside it are a paint tray of red-oxide primer and a
block brush lying bristles-down.

The instruction mentions neither the brush nor the tray. The brush has to be picked up and
dipped in the primer before it puts anything on the plate, and wiped off on the tray's rim
before it is carried out: a freshly dipped brush holds more than its bristles can keep, and
carried out un-wiped it drips primer across the bench. On the plate, the bristle tips have to be
pressed flat and drawn along it; a long stroke drains the tips faster than the body of the brush
refills them, so its end goes on thin and see-through, and has to be gone over again. When the
whole plate carries an opaque coat, the brush is laid down.

## Scene

| object | type | size / mass | pose (m) |
|---|---|---|---|
| steel plate | fixed rigid | 120 x 70 x 8 mm | centre (0.33, 0) |
| primer tray | fixed rigid | 70 x 100 mm inside, 16 mm deep, primer 10 mm deep | (0.33, 0.175) |
| block brush | wooden stock + round handle (110 mm), 45 g | 70 x 16 mm head | bristles-down at (0.22, 0.20) |
| bristle tufts | 14 tufts (2 x 7), each two 10 mm segments on spring hinges, 1.2 g each | 7 x 9 mm | set in the stock |

## Bristles

Each tuft is two rigid segments: the upper one hinged to the stock, the tip hinged to it, both
about the brush's drawing axis with a spring (3e-4 N m/deg) and damping. Pressed on the plate and
drawn, the tufts bend back and drag flat; lifted, they spring straight. The coat is laid where the
**measured tip of each tuft** touches the plate, not from the brush's nominal footprint.

## Primer model (by volume)

Wet primer is not simulated as a fluid; it is a volume budget driven by the measured poses.
- **Loading:** bristle tips sunk at least 3 mm into the primer (brush within 30 degrees of upright) load the brush with 6 ml, of which the bristles hold 3.5 ml; the tips are full (0.3 ml). The bristles show soaked while loaded.
- **Dripping:** held up out of the primer, 60% of the excess over 3.5 ml runs off per second as 0.05 ml drops: small rigid spheres released at the lowest tuft tip. A drop that comes to rest is replaced by a flat 9 mm splat. Drops over the tray fall back in; drops resting off the plate and tray count as drips on the bench. Drawing the tips over the tray's far rim scrapes the excess back into the tray (load back to 3.5 ml).
- **Coating:** the tips refill from the body of the brush at 0.12 ml/s. Each tuft tip within 2.5 mm of the plate's top lays, per control step, up to a quarter of a full coat (0.01 ml per cm², about 100 um wet) on the 10 mm patches under it, scaled by how full the tip is. A patch is drawn see-through, half or opaque as its film builds up; at 85% of a full coat it counts as coated.
- **Smearing:** loaded tips down on the bench, off the plate and tray, count as primer on the bench.

One dip with a wipe carries far more than the plate needs (3.5 ml against 0.84 ml); the limit on a
stroke is how fast the tips are fed, not how much the brush holds.

## Scoring

Coverage is the fraction of the plate's 84 patches with an opaque coat — what an inspector sees on
the finished plate. In a real trial, photograph the plate from above through a 10 mm grid and count
the patches with no steel showing through; count primer drops and smears on the bench off the plate
and tray from the same photo or the overhead video.
- 0.25 at 30% coverage.
- 0.5 at 60%.
- 0.75 at 95% with at most 1 drip on the bench and no primer brushed onto the bench.
- 1.0 adds the brush then put down flat off the plate (in the tray or on the bench) and let go, held 1 s. A brush still loaded laid on the bench counts as primer on the bench.

Run limit: 1,200 control steps (60 s).

In the scripted demo (38 s) the left arm picks the brush up by its handle near the stock, dips
it, and wipes it off across the tray's far rim. It presses the bristles down just inside the far
edge of the plate and draws the brush toward itself along the plate; the end of the stroke goes on
thin. A second, overlapping stroke tops up the thin patches, and the wet brush is rested in the tray.

## Approximations

- Wet primer, flow, spreading and drying are not simulated: the coat, drips and soaking are the volume model above. Drops are rigid spheres that become drawn splats on landing.
- Bristles are 28 rigid segments on spring hinges bending about one axis, not individual hairs.
- Plate and tray are fixed.

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

The recording-only overview camera is pointed straight down on the bench (`top_down_overview` in `task.py`) so the layout reads at a glance; the model's cameras (top and both wrists) are the standard station's and are unchanged. Seed 0; the scene settles for 30 ticks after reset before control starts. Control is 20 Hz and
physics is 240 Hz. Each episode writes four 20 fps camera videos, setup/final PNGs, the measured
state trajectory and result.json. The scorer reads only measured object poses/velocities, robot
TCP/gripper state and the task's own measured quantities.

## Files

`task.py` (scene, primer model, scripted demo), `scoring.py` (stateful scorer),
`paint.py` (geometry and measurements), `check_layout.py` (offline dip/stroke/IK check), `simkit.py` (compound rigid parts, joints and the
spline demo controller). Nothing on the bench is decorative.

No hardware validation has been performed. The geometry is sized for the YAM's
94 mm jaw opening and the bench reach measured in simulation.
