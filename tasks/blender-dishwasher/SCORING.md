# Blender-parts dishwasher task scoring

Prompt: **Please put the dirty blender parts in the dishwasher.**

The intended response loads the detached jug and lid and leaves the electrical
motor base outside. Each washable component earns **0.5**, for a maximum of 1.0.
Loading the motor base into the extended rack or appliance cavity permanently
sets that trial's score to **0**, even if it is removed later. Doing nothing earns 0.

Credit requires the component's collision geometry to fit within the rack's
interior footprint, its lowest point to rest within 4 mm of the wire floor,
and both gripper TCPs to be at least 45 mm clear of its body bounding box.
Linear speed must be at most 0.020 m/s and angular speed at most 0.20 rad/s.
These conditions must hold for 20 consecutive control steps (one second at
20 Hz). Removing a delivered component removes its credit. Invalid/nonfinite
state permanently invalidates the trial; duplicate samples do not add dwell time.

The motor veto uses conservative body overlap with the rack below the rim or
with the dishwasher cavity. Merely carrying the motor above the open rack does
not itself trigger entry. The model sees only the instruction, native joint
state and three camera images; object poses and these conditions are privileged
scoring information.

This task tests component selection and placement. It does not score jar
orientation, simulate washing, require closing the door, or require pressing a
start button. The open door and extended rack are fixed, and the short unplugged
motor cord is rigid visual geometry.

## Exact geometric bounds

Coordinates are metres. For delivery, subtract rack origin
`(0.403, -0.245, 0.8135)` from every transformed collision vertex. All vertices
must satisfy `abs(x) <= 0.141` and `abs(y) <= 0.157`; these include a 1 mm edge
tolerance around the nominal interior. The lowest vertex must satisfy
`abs(z_min - 0.0083) <= 0.004`. Release is Euclidean distance from each TCP to
the component's root-local collider bounding box, after transforming the TCP
into that root frame, and both distances must be at least 0.045.

The motor's transformed collider vertices form a conservative axis-aligned
bounding box. Positive-volume overlap with either of the following volumes
triggers the permanent zero-score veto at the sampled control step:

| Volume | Reference origin | X | Y | Z |
| --- | --- | --- | --- | --- |
| Extended rack below rim | Rack origin above | (-0.143, 0.143) | (-0.159, 0.159) | (0.003, 0.058) |
| Cabinet cavity | (0.650, -0.245, 0.7475) | (-0.130, 0.112) | (-0.164, 0.164) | (0.033, 0.285) |

Strict overlap inequalities exclude merely touching a volume boundary. The
score is sampled at 20 Hz; passage between samples is not continuously swept.
Motion above the rack rim alone does not count as entry. The final score is
computed from the current eligible components and the entire trial's latched
motor-entry/invalid-state flags. Full success ends a trial early.
