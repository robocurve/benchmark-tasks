# Pouring-spout outcome rubric — sustained-opening-v3

The task starts already unsealed and ends before pouring. Measurements use the
carton's frame: front wall x = −35 mm, width along y, shoulder z = 0. Both the
triangle's apex and the actual upper paper lip matter: a folded-down lip must
not count as a usable mouth. No arm role, view or grasp is prescribed.

| Score | Required ending outcome |
|---|---|
| 0 | No qualifying progress, or an invalid/broken physical record. |
| 0.25 | Body tilt at most 5° from upright; upper lip at least 35 mm above shoulder; mouth width at least 20 mm; both apex and upper lip at or beyond the front wall. |
| 0.5 | Same conditions, mouth at least 30 mm wide, and both apex and upper lip at least 8 mm beyond the front wall. An opening that is still held or unsupported earns at most this score. |
| 1.0 | All 0.5 geometry conditions hold continuously during the final three seconds, with no robot/carton contact and actual carton-body/table support throughout. |

The instruction's **stable** means that the released, table-supported carton
retains a usable opening for this whole interval. Small settling, intact outer
flap motion, or other motion that preserves every required condition is allowed.
There is no panel-angle, footprint-drift, velocity or peak-to-peak micro-motion
limit. The task evaluates a persistent usable form, not a precision positioning
operation. Any opening, uprightness, release or support interruption restarts
the interval. Closing or re-grasping after success loses full credit.

## Visible measurement references

Mouth width is the absolute y separation of the two outer upper inner-fin
corners. Apex and lip projections are their forward distances from the front
wall. Upper lip refers to the central shared upper edge vertex, not an outer
sealing flap. The code transforms these local reference vertices by each
body's current pose:

- apex: spout (0, 0, 35) mm;
- central upper lip: inner-fin-minus (0, 0, 59) mm;
- mouth endpoints: inner-fin-minus and inner-fin-plus (−35, 0, 59) mm.

They are material landmarks, not world targets or assumed final locations.
Tilt is the carton body's vertical axis relative to the table normal.
Numeric boundaries are inclusive. Three seconds span 721 poses at 240 Hz;
every native physics sample must qualify. The score is the ending outcome,
not the best score attained earlier. Partial credit depends on ending geometry;
it does not require release or a completed hold.

## Explicit revision from the submitted rubric

The submitted all-panel 1 mm / 1° stillness rule is removed. There is also no
replacement 1 mm visible-edge or footprint-motion requirement. A grader checks
the functional dimensions with a ruler and square and watches the released
opening persist for three seconds. Hidden panel rotations are irrelevant.
This is an explicit rubric revision, not a claim that native 320 × 176 policy
images can resolve millimetre motion of occluded paper.

## Integrity and evidence validity

An actual torn crease or detached panel invalidates the episode. In the rigid
simulation, a reported broken joint or >1 mm separation of any joined crease
end is the numerical proxy, checked throughout the episode. This remaining
1 mm constant is a **simulation-integrity guard**, not a physical stability
measurement. Nonfinite poses, missing body/contact coverage and saturated
contact buffers invalidate persistently. The separate 0.2 mm solver-convergence
diagnostic is not part of scoring and is not a claim about real paper stretch.

Robot/table and interarm contacts are recorded for review, not extra scoring
restrictions. [GRADING_PROTOCOL.md](GRADING_PROTOCOL.md) defines practical
physical checks and uncertainty handling. No calibrated paper forces, tearing
model, leak-tightness or liquid pouring is claimed.
