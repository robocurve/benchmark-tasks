# Revision evidence

All trials below use the revised task, evaluation seed 0, 20 Hz and a 1,500-step limit. Checkpoints come from the official Inspect Robots scorer on Brev.

| Policy or control | Score | Steps | Observed outcome |
|---|---:|---:|---|
| Reference | 1.0 | 789 | Tube unlocked, lowered and released in the carton. |
| Outside-carton control | 0.5 | 935 | Both ends rested on the bare tabletop, clear of the carton. |
| High-release control | 0.0 | 865 | The drop veto latched after the tube left the hands. |
| Dream-1 | 0.0 | 1500 | 147 model requests, full episode. |

Both placement levels require four continuous seconds with no hand contact and both ends supported on the same surface. Each end must be within 4 mm of support height. Renewed lamp contact, motion above the 0.02 m/s threshold, or a change of surface restarts the timer. The drop and sustained holder-damage vetoes apply to the entire trial. The tests exercise all of those transitions and both vetoes.

On the full-score run, the peak spring-drive estimate was 20.87 N per finger. The largest single-finger normal-resultant contact readings were 24.8 N against the tube, 10.6 N against the carton and 8.1 N against the fixture. These are simulator measurements. Hardware grip force, tube friction and holder breakout torque remain unmeasured.

From the task’s `isaac/` directory, run the scripted command in the README with `--stage demo`, `--stage outside`, or `--stage neg-drop-high` between `official_run.py` and `--`. The README also gives the Dream-1 command. The published videos contain all three cameras in chronological order.
