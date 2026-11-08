# Tie an overhand knot

Tie one overhand knot in the blue cord. Pull the ends apart to at least
117 cm for one second with the whole cord at least 1 cm above the table, then
let go and leave the knot intact for five seconds.

The scripted demo scores **1.0** and finishes in **138.30 seconds**. The recorded
Dream run scores **0.0** after all 3,000 actions. Both were independently
replayed. Dream completed 371 requests in one uninterrupted session, with
no retries.

## Setup

Use the standard two-arm YAM station and one blue, round polyester cord:
10 mm diameter, 131 cm outside length, and 97.11 g mass. Its 130 cm capsule-axis
length plus rounded caps gives the outside length. Terminal links have 12.5 mm
axes and are yellow and red. There are no clamps, pegs or attachments.

The wooden tabletop is 110×130×6 cm, with its top at z=0.7475 m. Coordinates
are metres: x points away from the robot, y to its left, and z up. The YAM
base-frame origins are (0,+0.305,0.750) and (0,−0.305,0.750): 61 cm apart and
2.5 mm above the tabletop. Both arms start in the standard ready pose with
open grippers.

Lay the cord flat in an uncrossed U. Its two straight legs are 30 cm long,
joined by a 70 cm semicircular section. The yellow and red end-joint centres
start at (0.060,+0.0428461,0.753) and (0.060,−0.4028461,0.753), respectively.
Both straight legs end at x=0.360; the far bend reaches x=0.5828461.
`config.py` generates the exact 105 joint centres at 12.5 mm spacing.
To reset, untie the cord and restore this layout.

The 28 cm tabletop ruler and 20 cm height guide are render-only annotations
with 5 mm ticks. They have no collision geometry. Their positions and the
separate review camera are specified in `visuals.py`; the review camera is
not a policy input.

## Run and score

Activate the competition's YAM / Isaac Lab-Arena `yam.env`; versions are
pinned in [ENVIRONMENT.json](ENVIRONMENT.json). From this folder, use a fresh
output directory for each run:

```bash
python run.py --policy demo --seed 0 --physics-hz 240 --max-steps 3000 --log-dir /tmp/knot-demo
python run.py --policy dream --dream-rows-per-request 8 --seed 0 --physics-hz 240 --max-steps 3000 --log-dir /tmp/knot-dream
python replay.py /tmp/knot-demo/trajectory.jsonl
python replay.py /tmp/knot-dream/trajectory.jsonl
python -m unittest discover -s . -p 'test_*.py'
```

Dream needs its normal API credentials. It requests a fresh prediction after
eight action rows and stops if the server session is replaced, preventing
silent loss of episode memory. Both policies use the same layout,
seed and 3,000-action limit (150 seconds at 20 Hz), including final release.
Physics runs at 240 Hz for 78 seconds, then 960 Hz for every policy, without
extra actions or forces. On an NVIDIA L40S, the demo took 29.3 wall-clock
minutes and Dream took 27.1 minutes. Runs save source, configuration, actions,
geometry, video and scoring certificates for independent replay.

The demo lays a crossing, raises a local arch, passes the red terminal under
it, then recovers both ends to lift, pull and release together. It uses ordinary
joint/gripper commands and measured rope geometry. Dream receives the standard
camera and joint observations.

## Judging and evidence

[JUDGING.md](JUDGING.md) gives the ruler-and-timer protocol: a final released
overhand earns 0.5; the preceding suspended span check raises this to 1.0.
There is no evaluator pull or claim about knot tension or strength.

Demo 020 meets the span check from 130.75–131.75 seconds and the release check
from 133.30–138.30 seconds. During those intervals, its corrected tip span is
at least 117.0074 cm, the cord is at least 10.8722 cm above the table, and its
released clearance from the robot is at least 3.7941 cm. All recorded robot
poses passed inter-arm, table and nonadjacent same-arm mesh checks. Dream
had 16 sampled fingertip/table penetrations, at most 0.294 mm; it formed no
knot and scored zero. See
[VALIDATION.json](VALIDATION.json) and [the final-run evidence](evidence/final-runs.json).

The cord uses 104 rigid capsules with bending resistance, free axial twist
and nonadjacent self-contact. Mass follows 74.7 g/m in the
[Marlow 10 mm Doublebraid specification](https://shop.marlowropes.com/content/files/datasheets/2026/marlow-doublebraid-datasheet-0326.pdf).
Friction (0.8), bending stiffness (0.000390625 N m/rad) and damping
(0.0000390625 N m s/rad) are uncalibrated assumptions. Fibres, stretch and wear
are not modelled. The solver uses 64 position and 16 velocity iterations,
0.5 mm contact offset and zero rest offset.

The closest recorded nonlocal cord-axis separation is 9.509743 mm against
the 9.5 mm validity limit: a narrow margin. Audits inspect 20 Hz samples.
One fixed-seed success does not establish repeatability; no real-cord trial
has been performed.

The active scorer is `bimanual_scoring.py`. Some retained helper names and
regression tests refer to older jig/load experiments; the current runner
rejects those modes, and no external-force assay is reachable in this task.

## License

Task code and procedural cord geometry are released under the MIT license
in [LICENSE](LICENSE). The standard robot and simulator assets come from the
competition environment and are not redistributed here.
