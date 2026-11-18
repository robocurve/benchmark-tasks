# Finish and fit a loose necktie

Snug a loose Simple knot, hold the narrow blade and slide the knot up to the
collar, then release it neatly. This is the finishing sequence of an already
threaded tie. The wide blade tightens the outer wrap; the narrow blade is the
standing part used to fit the knot to the collar. They are separate grasps and
motions in both the model and the new demonstration.

The 18 November revision demonstrates both stages. The demo scores **1.0** in
810 actions (40.5 seconds); Dream-1 scores **0.0** after its full 1,200-action limit.
Both scores were independently replayed from measured native hand poses.

## Fixed setup and reset

Coordinates are metres: x away from the arms, y toward the left arm, z up.
The table is z=0.7475. The rigid plastic neck is centred at (0.400,0,1.250),
110 mm diameter and 100 mm high; its lower reference edge is z=1.200.
Two white 50 × 55 × 8 mm collar flaps and a black centre mark define the fit.
The steel stand uses a 180 × 160 × 12 mm base, 25 × 25 × 455 mm post and
210 × 25 × 25 mm crosspiece. Their exact fixed poses are in task.py.

Use a reversible woven polyester tie: 1.26 m long, about 1.2 mm thick,
70 mm wide at its wide end and 35 mm at the narrow end. Prepare a loose
Simple (Oriental) knot: facing the collar, reverse face outward, wide end W
on the left and narrow end N on the right. Bring W behind N left-to-right,
across its front right-to-left, up through the neck opening from underneath,
then down behind the front band and in front of N. Leave it loose.
See [prepared-knot.svg](prepared-knot.svg) for the threading order; it is a physical preparation
diagram, not a claim of simulated internal cloth folds.

Set the knot centre to (0.319,0.025,1.090), 74 mm wide and 57 mm high.
Its top is 81.5 mm below the collar edge. The 250 mm exposed wide blade
hangs vertically from (0.313,0.025,1.0615). The approximately 85.5 mm narrow
blade emerges behind it at (0.327,0.025,1.0615), hanging vertically behind the wide blade. Neither blade is in a
keeper loop. Match the front and side setup views to the nearest 5 mm using
a ruler. If the chosen tie will not retain that loose reset after the preparer's
hands are removed, use another reversible tie of the specified size; do not
add clips or supports. All material properties remain uncalibrated assumptions.

Both hands start in the normal YAM ready pose, open. Inspect the tie and restore
the same loose dimensions before each trial. No person is used.

## Run and score

Use the pinned environment in ENVIRONMENT.json with the task folder available.

```bash
python tasks/fit-a-loose-necktie/run.py --mode demo --out artifacts/tie-demo
python tasks/fit-a-loose-necktie/run.py --mode dream --out artifacts/tie-dream
python tasks/fit-a-loose-necktie/rescore.py artifacts/tie-demo --out artifacts/tie-demo/rescore.json
python tasks/fit-a-loose-necktie/rescore.py artifacts/tie-dream --out artifacts/tie-dream/rescore.json
python -m unittest discover -s tasks/fit-a-loose-necktie -p 'test_*.py'
```

Both policies use the same reset, model and 1,200-action limit (60 seconds at
20 Hz). The demo commands only bounded robot joint targets. It first holds the
knot while drawing the wide blade, releases and regrips the narrow blade, then
slides and centres the knot. JUDGING.md defines outcome-based partial credit
using photographs, a ruler and a five-second released hold.

Each run records all measured hands, material state, actions, commanded reaction forces,
source hashes, native front/rear/side video and policy-camera video. The submitted
videos show all three judging views at normal speed. The released demo knot is
52 mm wide, 6 mm below the collar and within 1 mm of its centre mark. It remains
released for 7.55 seconds. The wide tip is 358 mm below the collar; the narrow tip
is 74 mm higher. These measurements agree with the recorded views.

To audit the included records, pass `tasks/fit-a-loose-necktie/evidence/demo` or
`evidence/dream` (under the same task folder) to `rescore.py`. The replay preserves
the native float32 rotation precision; no scoring threshold changed for replay.

On one shared A40, the demo took 5.7 minutes and Dream took 22.0 minutes, including
rendering and model waits. Dream request/session details are recorded in its run
report and transcript; recovered sessions are not an uninterrupted baseline.

## Model and limitations

The prepared knot keeps its topology. Finite jaw volumes capture visible
material; grasped material can slip if the hand moves beyond it. Wide-blade
feed changes wrap slack. A distinct narrow-blade grasp permits upward fitting;
the old wide-blade-only fitting strategy cannot reach full credit. Total
neckband, wrap and blade length stays 1.26 m. Released blades drape under a
quasistatic approximation, and the knot remains in the front plane.

A regularized 3 N sliding-friction approximation applies equal and opposite
loads to the native grippers during relative feed. This is an engineering
assumption for a loose polyester knot, not a measured tie calibration. Internal
folds, arbitrary rethreading, detailed textile/finger contact and skin are not
simulated. The model does not claim full knot tying or dressing a person.

The distinction between snugging and fitting follows the final steps of
[Ties.com's Simple-knot instructions](https://www.ties.com/how-to-tie-a-tie/simple)
and the [NHS dressing guide](https://rhc.nhsggc.org.uk/media/gnapxmrd/step-by-step-guide_right-handed.pdf).
