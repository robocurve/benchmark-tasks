# Fixture definition and measurement worksheet

The simulated work is restoring the barrier after access to a benchtop chuck enclosure. The stopped chuck has a 78 mm body and three radial jaws around a blunt workpiece. The door's tongue operates a spring plunger. A separate sliding bolt bears in the eyelet once the operator-facing door is released. The seating face gives the door a physical alignment datum. The latch, frame and interlock are a custom research assembly. No equivalence to an installed lathe guard or electrical safety system has been established.

## Geometry

`parts.csv` gives the center, orientation and dimensions of each collider exported from the shipped USD. Coordinates are in millimetres relative to each named rigid body. The first six columns identify the part and its mounting center. The source is `tasks/restore-machine-guard/build_asset.py`. The assembled origin is at the hinge axis, +315 mm station X, +104 mm station Y and 20 mm above the tabletop.

| Group | Body home relative to fixture | Assembly or material representation |
|---|---|---|
| Fixed frame, headstock, chuck and stop | (0, 0, 0) | Fixed rigid body, 1.6 kg simulation mass. Headstock carries a stationary chuck. The stop bracket connects the striker post to the door seating face. The nominal shut panel rear face is +2.0 mm X and the stop face is +2.1 mm X. |
| Leaf, keeper and handle | Hinge about local Z, starts at −53° | Rigid 170 × 145 × 8 mm transparent panel, with keeper, tongue and stand-off grip. Combined model mass 0.18 kg. Door grip is 20 × 25 × 28 mm. |
| Shaft, grip and carriage | (−14, −218, +68) mm, travel +Y | Grooved shaft, 8 mm nominal OD and 6.2 mm at groove. The 125 mm rearward grip offset is measured relative to the carriage at home. Riser and tail are connected colliders on the same body. Combined mass 0.036 kg. |
| Plunger | (+25, −120, +125) mm, travel +X | Independent spring-return slider, combined mass 0.008 kg. Its 7 mm stroke bottoms independently of the sliding bolt. |
| Passive drives | Opening, retracting, returning toward zero respectively | Hinge 1.2 N·m/rad and 0.12 N·m·s/rad, at rest at −53°. Bolt 18 N/m and 7 N·s/m. Plunger 85 N/m and 2 N·s/m. Model static/dynamic friction 0.65/0.50 and restitution 0.04. |

At the keeper center, 179 mm from the hinge along Y, 0.46° of door yaw produces about 1.44 mm of lateral offset. Nominal side clearances around the 8 mm bolt are 1.0 mm and 0.5 mm. A plunger reading can therefore be consistent with a blocked shaft. The v10 native trace establishes the hinge angle and shaft travel. Evaluating both axial faces of the 10 mm keeper at the observed angle gives a minimum clearance of approximately -0.495 mm. V11 logs shaft contacts by individual keeper wall and records the pre-action clearance used by the demonstrator. A positive switch reading alone gives no evidence that the bolt crossed the eyelet.

## Evidence for any future physical trial

Use an inert bench mock-up for this benchmark. An evaluator would need to record the actual part numbers, as-built dimensions, attachment method, masses, friction and spring curves, switch operating and release points, and the passive retention behavior. The simulation clamps rigid components together and uses ideal joint guides. The package contains simulation dimensions and model parameters. A physical unit has yet to be built or calibrated.

Record a synchronized side view of the bolt/keeper and an overview of the opening, include a scale in the motion plane, and record the switch travel independently when occluded. Measure shaft travel, door angle, plunger stroke and dwell against the scored thresholds. Record instrument resolution, calibration uncertainty and repeat spread. A reading whose uncertainty spans a threshold is indeterminate. Fingertip normals and contact forces are simulator reference diagnostics. Ordinary camera footage cannot independently provide those force readings.

| Observation | Record for each reset, unlatched control and full reference |
|---|---|
| Specimen and setup | Assembly ID, drawings/revision, photos, instruments, calibration, measured bearing/guide fit |
| Initial condition | Door angle, shaft travel and plunger travel before approach and after settling |
| Mechanical progression | Two-sided handle engagement, tongue contact, keeper entry and gross motion during release |
| Terminal state | Door angle, bolt travel and independent plunger position over the full release interval, hand clearance and duration |
| Comparability | Replicates, measurement uncertainty and discrepancies from the model, including whether the unlatched door reopens under its own spring |
