# Seat and secure a supported 24/40 condenser joint

This task models a routine two-handed glassware assembly. One arm lifts a 150 g Liebig condenser from two cork
cradles, places its jacket in a fixed cork-lined V support, and seats its 24/40 female joint on the still head's matched
24/40 male taper. While that arm holds alignment, the other installs a yellow size-24 POM Keck clip over both exposed
joint lips. Both grippers must release and move clear; the condenser must remain supported, seated and clipped.

The task is not a generic insertion. It tests matched ground-glass geometry, support of a long fragile part, and
installation of the separate fastener while the first arm maintains alignment. A past clip event is insufficient:
full score continuously rechecks the released physical end state.

## Reproducible physical setup

- Still head: fixed borosilicate body on a clamped stand. Its horizontal 24/40 male joint is centred at
  `(x=0.28, y=0.04, z=0.8325) m`, has 40 mm ground length, 24.0 mm large-end and 20.0 mm small-end diameters, and a
  33 mm retaining lip.
- Condenser: 150 g, 280 mm overall Liebig condenser with a matched 24/40 female joint. The simulator uses 0.50 mm
  diametral clearance and a 2.5 mm socket wall. It starts level on two cork cradles with its socket lip at
  `(0.19,-0.03,0.7825) m`.
- Support: a fixed chromed stand topped by two cork pads forming a V under the far jacket, centred at
  `(0.28,-0.165) m`. It carries the condenser's vertical load after assembly; the joint and clip are not used as a
  cantilever.
- Clip: yellow size-24 POM Keck clip, 8 g, with a 51 × 20 × 6 mm spine and two 22 mm arms. It starts upright at
  `(0.19,0.12) m` and captures the male and female retaining lips.
- Fixed layout: `object_jitter=0`.

The stepped taper and thin lip shells render the dimensions above but do not collide: PhysX contact margins on the
coarse thin primitives otherwise stop the surfaces before mating. The colliding V support carries the glass, and the
constraint activates only when the measured socket seat and two-lip clip-capture poses are both valid.

The real apparatus should use commercially matched, undamaged 24/40 glassware and a size-24 clip. Wear eye protection,
keep the assembly unheated and empty, support the condenser before removing the clip, and replace chipped glass.

## Scoring

The scorer uses body poses and measured velocities:

| End state | Score |
|---|---:|
| Female joint was seated at least once, without a drop | 0.25 |
| Joint is currently seated and clip is currently over both lips | 0.50 |
| Both grippers stay at least 6 cm away while the current assembly retains the 150 g condenser under its 1.47 N weight for 2 s, then both TCPs finish within 5 cm of start | 1.00 |

Current seating requires the socket lip from 2 mm inward to 4 mm outward of the male shoulder, transverse axis error
below 2.5 mm, angular error below 5°, and the jacket in the support. Current clip placement uses one tolerance set:
centre error below 6 mm across and along the joint, spine-height error below 4 mm, and tilt below 20°.

Before retention, downward speed over 0.7 m/s or tilt over 35° counts as a drop; total speed over 1.2 m/s at any time
is a conservative rigid-body proxy for a likely glass strike. Either forces score 0. Any unlisted outcome scores 0.
The snap rule only activates the simulated constraint; it does not award final credit by itself.

## Run

From this folder in the standard Isaac Lab Arena environment:

```bash
INSTRUCTION="Lift the Liebig condenser into its padded support, seat its 24/40 female joint fully on the still head's matching 24/40 male joint, and hold it aligned while the other hand installs the yellow size-24 Keck clip over both joint lips. Release both objects and return both arms to their starting poses. Do not drop or strike the glass."

# Scripted full-score demonstration
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clip_a_condenser_onto_a_still_head.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/condenser-demo

# Required Dream-1 attempt
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clip_a_condenser_onto_a_still_head.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/condenser-dream
```

The scripted plan uses both arms and takes about 25 simulated seconds. It lifts and supports the condenser, seats the
matched joint, installs the clip, releases both objects, clears both grippers for longer than two seconds, and returns
both arms home. Dream-1 receives the full 2,400-step budget. There are no random seeds because the layout is fixed.

Render each completed log with `inspect-robots video LOG.json`. Submission recordings must be H.264, 1280 × 720,
`yuv420p`, and no more than 50 MB each.

## Reset

Support the condenser, remove the clip, separate the joint without levering it against the stand, return the condenser
to both cradles, and stand the clip at its marked start. Confirm the support pads remain centred and inspect both joint
lips before every trial.
