# Dry all surfaces of long hair with a fixed head — incomplete feedback draft

This is an incomplete draft for early feedback on task relevance, novelty and physical fidelity. It is not presented as meeting the competition feasibility or media requirements. No completed model result or full-score demonstration is claimed. `task.json` and board media are deliberately absent until valid evidence exists; please keep this PR in draft.

Use the arms to operate a handheld dryer and expose long hair sections, drying both sides and hidden layers evenly. The mannequin head stays fixed to its stand; it must not rotate to present hair to a stationary dryer.

Implemented: a world FixedJoint for the head, a 2 mm/1 degree motion invalidation guard, twelve passive articulated locks with sixty passive joints, surface moisture/heat accounting with geometric airflow occlusion, and incomplete switch/pickup controllers. Fixed-head geometry and guards passed 42 CPU/USD checks. Earlier native geometry/switch probes used an older head configuration and do not validate this fixed-head version.

Measured score for the current fixed-head task: not available. No complete drying demonstration or genuine model attempt exists.

Unfinished: native fixed-head validation, stable switch operation, drying sweeps and lock manipulation, calibration of airflow/thermal approximations, and full demonstration/media. Hair is represented by coarse articulated bundles rather than individual fibers or fluid simulation. Review feedback is requested on whether this approximation can support the claimed all-surface drying outcome.
