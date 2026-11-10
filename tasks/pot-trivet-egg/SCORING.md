# Measured-state scoring

The trial receives the score at its final step. Partial progress is not banked:
returning the egg to the trivet or moving the pot off its support removes that
progress. A detected unsafe event permanently overrides every partial score.

| Score | Required current outcome |
| --- | --- |
| 0 | No qualifying progress, or any latched unsafe/invalid event. |
| 0.25 | Egg clearance for 6 consecutive control steps (0.3 s). The egg may still be held. |
| 0.5 | Egg clearance, plus pot support for 10 consecutive steps (0.5 s). |
| 1.0 | Egg clearance, pot support, a supported egg, both objects released and stationary for 20 consecutive steps (1 s). |

All distances are in world coordinates unless stated otherwise. Geometry comes
from the authored mesh/box colliders, transformed using measured rigid-body poses.
Visual bounding boxes are not substituted for the support geometry.

## Egg clearance

- Every egg collider vertex must be at least 111 mm horizontally from the
  trivet centre.
- The egg's transformed grasp-centre point must be at least 120 mm horizontally
  from the pot root. This prevents placing the egg inside the pot.
- Neither a particular arm nor use of the blue egg rest is required.

## Pot support

- Every bottom-collider vertex of the pot must lie within a 105.8 mm horizontal
  radius of the trivet centre. This disk lies inside the top support collider:
  a 106 mm-radius, 64-sided polygon. The pot floor collider has an 84 mm radius.
- Pot tilt must be at most 12 degrees from upright.
- The pot's lowest collider point must lie between 3 mm below and 6 mm above
  the trivet top, Z=0.7655 m.
- Egg clearance must remain true throughout the support dwell.

The whole-base condition allows harmless small centre offsets. There is no
separate arbitrary centre-offset requirement.

## Supported egg, release and stability

The egg is supported if either:

- Its lowest collider point is within 6 mm of the tabletop, Z=0.7475 m; or
- Its transformed grasp centre is within 42 mm horizontally of the egg-rest
  centre, and its lowest collider point is 7 to 35 mm above the tabletop.

Both measured gripper tool-centre points must be at least 55 mm outside
each object's root-local, axis-aligned collider bounds. All four arm/object
clearances must pass. This checks the full pot, including its rim and handle.

For each object, linear speed must be at most 0.020 m/s and angular speed
at most 0.20 rad/s. Full score requires all final conditions for 20 consecutive
20 Hz control steps. Successful evaluation terminates at that point.

## Permanent zero conditions

- Any filtered egg/pot normal contact-force sample above 0.20 N.
- Nonfinite object poses, velocities or contact-force data.
- An object's lowest collider point below Z=0.7415 m.
- Any object collider point outside X=[-0.18,0.90] m or Y=[-0.64,0.64] m.
- Nonmonotonic scorer step IDs. Duplicate reads of the same step return the
  previous result without advancing a dwell.

Contact history covers 12 physics steps between each 20 Hz control sample at
240 Hz physics. Missing filtered contact data causes an error rather than a
numeric score. A contact failure stays latched even if the pot is later lifted
away; a physical negative-control trial verified this.

The 0.20 N threshold is a conservative simulation proxy. The egg remains a
rigid body: shell fracture, damage from squeezing with the gripper, thermal
effects and real egg breakage forces are not modeled.

## Implementation

StateReader.controls() in demo_runtime.py computes geometric, support,
release, velocity and contact conditions. TaskScorer.update() in scoring.py
applies the consecutive-step counts and irreversible failures. The Inspect
adapter returns that same score; Dream receives RGB images and measured
joint/gripper state, without object poses or the scorer's geometric data.
