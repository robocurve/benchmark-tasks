# Task definition and review rationale

Category: **Impact on jobs**.

The intended work sequence is careful removal of loose backfill around an
already located, de-energized pipe: use a spade with one hand on the D-handle
and the other on the shaft; take repeated shallow scoops; lift, carry and dump
the spoil in the designated tray; expose a continuous inspection length. Spade contact with the fixed pipe is
allowed; pipe damage is not scored. This tabletop apparatus isolates that manipulation sequence.
It does not represent opening an entire construction trench or locating an
unknown utility.

## Occupational relevance

[O*NET construction laborers](https://www.onetonline.org/link/details/47-2061.00)
lists shovel-based trench digging and backfilling as a core duty.
[O*NET pipelayers](https://www.onetonline.org/link/details/47-2151.00) likewise
lists hand/tool trench excavation as a core duty. Those are stronger grounds
for the selected category than claiming that everyone regularly digs soil.
Local excavation around a buried pipe and controlled spoil placement distinguish the intended skill
from merely scooping objects between bowls.

[Plumbing and pipelaying helpers](https://www.onetonline.org/link/details/47-3015.00)
also have trench/hole digging among their work activities.
[Landscaping workers](https://www.onetonline.org/link/details/37-3011.00)
perform digging and sprinkler installation. These support the related plumbing
and irrigation-maintenance context; they do not establish that pipe exposure
is frequent for every worker in those broad occupations.

## Relevant reviews

- [PR 1057](https://github.com/robocurve/benchmark-tasks/pull/1057#issuecomment-6819019238)
  was rejected for replacing soil with a shallow lattice of large wooden cubes
  and permitting gripper sweeping without the spade. This task tracks actual
  tool-carried material and requires two grips during work. Robot contact with soil in the bed is monitored at physics substeps and
  latches failure. Pipe contact is allowed and logged diagnostically. Merely revealing the pipe earns no credit.
- [PR 684](https://github.com/robocurve/benchmark-tasks/pull/684#issuecomment-6819693438)
  exposed a route-of-transfer loophole and a particle-count scoring problem.
  Here, credited spoil must have been lifted on the blade with both hands and
  end up in the tray. Thresholds use exposed length and physical mass, rather
  than counts of simulator particles.
- [PR 666](https://github.com/robocurve/benchmark-tasks/pull/666#issuecomment-6818414068)
  identifies generic scoop/pour duplication, single-particle transfer bypasses,
  non-monotonic scores and unclear asset licenses. Progress thresholds here
  are monotonic, require repeated loaded trips, and measure contiguous crown exposure. The spade is procedurally authored in this folder.

Public PR search found the closed pipe task above. The authenticated task board
redirected to sign-in, so current board-wide exclusivity has not been verified.
No PR or media submission is made by the local demo runner.

## Numeric draft rubric

The marked evaluation span is trench coordinate u = -20 to +100 mm. Exposure
means a continuous section of the central 16 mm crown is visible from above.
The implementation uses 5 mm longitudinal bins and ray intersections with the actual oriented grain meshes.

| Score | Contiguous exposed length | Spoil delivered by spade | Loaded trips |
|---|---:|---:|---:|
| 0.25 | 20 mm | 30 g | 1 |
| 0.50 | 40 mm | 70 g | 2 |
| 1.00 | 80 mm | 120 g | 3 |

Each counted trip must contribute at least 20 g still in the tray. Credited soil
must be off the blade and move less than 20 mm/s for at least 0.25 seconds.
Full success also requires the blade's overhead footprint to be clear of the
fixture and no more than 5 g remaining on the blade, so the final scoop is
emptied and the tool cannot hide the exposed pipe at termination. Final spilled
mass may not exceed 30 g. Robot contact with soil in the bed below
the rim, invalid particle state, or tool use without both grips
for more than 0.20 s invalidates all progress. Scores use the end state, so
previous progress cannot override a later failure.

A physical reproduction uses a ruler/overhead photograph for crown exposure,
a tared tray/scale for spoil and spillage, and video for grip and transfer
verification. Use an isolated, unpressurized test pipe. Spade contact is
permitted, and no pipe-fracture or safe-field-force prediction is claimed.

## Fidelity limits that still need evidence

The current bed uses noncohesive angular convex grains with friction. Angular
damping is 0.1, rather than the spherical prototype's rolling-suppression value
of 6.0. Individual grains have mineral density 2,650 kg/m³; their exact
icosahedral volume sets their mass. The shallow saved 3,655-grain fill weighs 2.669 kg.
Its maximum initial cover above the crown is about 23 mm, before reset settling.
It has no moisture, cohesive fracture, continuum compaction, roots or calibrated
digging resistance. It must not be described as validated real-soil excavation.
In particular, finer grains and a successful video alone do not resolve the
soil-fidelity criticism in PR 1057. Repose, bulk density, penetration resistance
and repeatability against a specified physical soil preparation remain necessary
before claiming that criticism is fully addressed.

The fixed planter and pipe represent a clamped test fixture. The free spade is
held only by physical finger contact. The demonstration may use proprioceptive
feedback but cannot teleport material, attach the spade to the hands, modify
scoring from its action sequence, or change the robot's force limits.

Acceptance and novelty are reviewer judgments, not conclusions established by
passing the local scoring tests.

All off-tray spillage counts toward the same 30 g limit, including grains below
the table. Airborne soil brushing a wrist after lifting is logged separately;
robot contact with grains still in the fixture below its rim invalidates the
trial. Thus gripper sweeping/picking remains prohibited without treating dirt
falling onto a hand during legitimate tool use as bare-hand excavation.
