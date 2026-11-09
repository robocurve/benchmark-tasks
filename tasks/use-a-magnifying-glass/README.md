# Use a magnifying glass to inspect a target

Pick up the magnifier and align it with the left wrist camera and printed
chart. The key skill is positioning and orienting an optical tool relative to
the camera so that fine detail becomes readable.

The scripted demo scores **1.0** and Dream-1 scores **0.0**.
The demo completes 530 actions (26.5 seconds); Dream uses the full 1,200-action
budget (60 seconds). On an L40S, total process runtimes were
130.0 seconds and 274.0 seconds, respectively, including startup and
reference capture. Both use seed 0. See [VALIDATION.json](VALIDATION.json).

## Setup

Use the standard two-arm YAM station. Coordinates below are metres: x points
away from the arm bases, y left, and z up. The wooden tabletop is
110×130×6 cm, centred at (0.36,0,0.7175), with its top at z=0.7475. The arm
bases are (0,±0.305,0.750). Both arms start in the standard ready pose with
open grippers. There is no object jitter.

- **Magnifier:** 120 g total; plastic handle 80×20×18 mm; plastic frame 68 mm
  outside diameter, 60 mm aperture and 18 mm thickness. The 60 mm glass lens
  has two spherical surfaces of 100 mm radius, 1 mm edge thickness, about
  10.21 mm centre thickness and refractive index 1.5. Its effective focal
  length is approximately 102 mm. The handle centre starts at
  (0.20,−0.27,0.757), yaw −90°. The lens centre is (0.20,−0.20,0.757), with
  its axis vertical and handle pointing toward −y. It settles about 0.5 mm
  onto the table before the first observation.
- **Target:** a fixed white plastic plate, 120×100×7.5 mm, centred at
  (0.35,0.05,0.75125). A matte print covers its top. Generate it with
  `inspection_target.py`; print at exactly 120×100 mm. Card +X/+Y point
  toward world −x/−y, respectively (180° about z). The print lies at
  z=0.7551 in simulation.

The chart has four coloured circles, radius 1.1 mm, on the corners of a
19 mm square. Each line group contains five 1.2 mm black bars separated by
four 1.2 mm gaps; bars are 3.2 mm long. The groups are perpendicular. Two
3 mm black/white swatches set contrast references. Four 1.5 mm black squares
at card coordinates (±50,±40) mm check that the reference view matches.
The generated chart and JSON specify every mark.

Reset by returning the tool and arms to these poses; the target is reusable.
Keep the camera settings and target fixed during reference capture. No exact
camera world pose or lens angle is prescribed for success.

## Run

Activate the competition's YAM / Isaac Lab-Arena environment. The tested
versions and repository revisions are in [ENVIRONMENT.json](ENVIRONMENT.json).
From this folder, use a new output directory for every trial:

```bash
python run.py --policy demo --seed 0 --out /tmp/magnifier-demo
python run.py --policy dream --seed 0 --dream-rows-per-request 8 --out /tmp/magnifier-dream
python inspection_target.py --output-dir /tmp/magnifier-chart
```

Dream uses its normal local API credentials. The run limit is 1,200 actions
(60 seconds at 20 Hz), including its observation warm-up. The scripted demo
finishes after 530 actions (26.5 seconds). It uses measured tool pose for
small final arm corrections. Dream receives only the standard camera and
joint observations. Both modes use the same scene, instruction and dynamics.
A replaced Dream session stops the run rather than silently starting a new
trial. The recorded runtime and scores are in [VALIDATION.json](VALIDATION.json).

The native wrist feed is 640×360. The unchanged YAM wrapper resizes it to
224×224; this is the image used for the detail check. The standard Dream
adapter subsequently resizes that image to 320×176 and encodes JPEG quality
85 for the model. The separate review camera is never a policy input.

## Score and evidence

[JUDGING.md](JUDGING.md) gives the photo, ruler and video procedure. The score
combines handle pickup and an unsupported hold with measured optical gain
and newly readable detail. A same-pose lens-free reference prevents a closer
camera or digital zoom from substituting for the lens. Reference images are
captured only after the robot finishes; they are not supplied to the policy.

The review videos show the complete control sequence and the actual wrist
feed at 2× display size. Any appended reference comparison is clearly labelled
as evaluation after the trial, outside control time. Measurements use original
pixels. Final numeric scores, hashes and review attestations are in `evidence/`.
Original per-frame PNG/NPZ files stay in the run directory; the reproduction
commands regenerate them. The public review media are uploaded separately.

## Fidelity

The closed biconvex mesh uses native RTX refraction (RealTimePathTracing,
six transmission/specular bounces). It is one dynamic rigid tool with normal
contact: no attachment, magnet or object-force helper. The frame uses 32
convex sectors; the glass collider is a convex hull. Total mass is 120 g;
friction 0.7, restitution 0, 0.2 mm contact offset and zero rest offset are
uncalibrated assumptions. Glass does not break.

The cameras are ideal pinholes: physical focus, sensor noise, optical coatings
and lens defects are omitted. A static PathTracing check agreed with the
selected real-time lens's enlargement within 0.4%; no physical magnifier
trial or hardware transfer is claimed. Sampled pose checks do not certify
all self-collisions or motion between samples.

Task code, chart and procedural lens/frame geometry are MIT licensed; see
[LICENSE](LICENSE). Standard robot and simulator assets come from the
competition environment and are not redistributed here.
