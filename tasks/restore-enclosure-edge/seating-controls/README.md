# Does an empty guard cycle reveal incomplete seating?

The question is whether an apparent repair is enough to release a guard to the next shift. A robot can press the loose rubber, cycle the door, watch it reopen and still leave the rigid shoulder exposed. The orange cylinder tests the repaired span. The worker's gloved hand stays on the loading shelf.

## Paired interventions

Six deterministic MuJoCo trials used the articulated YAMs, 500 Hz physics, 20 Hz actions, the same initial state, fixture and outcome scorer. In the two short-stroke conditions, the left arm stopped both its final press waypoints 18 mm or 24 mm sooner along the feed axis. Approach, timing, retraction and cycle-button actuation were identical. Within each pair, one trial used the reference instrument placement and the other ran empty. The normal-stroke pair serves as the baseline. Each short stroke produced physical fingertip contact with the lip.

| Seating press | Largest lip rotation before cycling | Empty cycle | Loaded cycle at repaired span | Loaded score |
|---|---:|---|---|---:|
| Normal | 74.9° | Lower-limit return, score 0.5 | Edge operated, peak probe travel 18.44 mm, reopened and parked | 1.0 |
| 18 mm short | 6.0° | Lower-limit return, score 0.0 | Edge did not operate, instrument cutoff at 30.06 mm | 0.0 |
| 24 mm short | 4.2° | Lower-limit return, score 0.0 | Edge did not operate, instrument cutoff at 30.06 mm | 0.0 |

All three empty cycles ended within 0.47 mm of the upper stop. In the two shortened attempts, the lip moved under the fingertip and rolled back after withdrawal. The detector remained electrically connected throughout. A powered return in free air therefore gave the same visible outcome for two different mechanical states. The empty-cycle scores include the evaluator's independent measurement of the clip position. The loading test gives the robot physical evidence of whether the sensing element responds at the damaged span.

These are supplemental articulated simulations. The native station and hosted Dream evidence are reported separately in `validation.json`. The travel limit describes a bench instrument. The experiment measures the protective function and does not estimate human injury severity or failure rates in production.

## Reproduction

The accompanying measurements include each control's compressed action trace, solver contacts, force-independent travel witness, source hashes and trace digest. In the full project, `diagnostics/bench.py` uses the published task's fixture, reference actions and evaluator. To repeat a short press, take `plan("reference")` for the loaded test or `plan("empty_cycle")` for the empty test. Subtract the stated distance from the X position of the left-arm waypoints named `roll the loose clip foot into its retainer` and `finish seating`. Preserve Y, Z, yaw, jaw command and waypoint duration. Run the plan from reset and repeat with zero offset. The full submission bundle includes the runner and verifier under `control-reproduction/`.
