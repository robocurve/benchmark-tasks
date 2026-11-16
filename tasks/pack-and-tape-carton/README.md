# Seal a preloaded carton with an ordinary tape roll

**Incomplete feedback draft:** [DRAFT_STATUS.md](DRAFT_STATUS.md) records current proof and missing work. No full completion is claimed.

The two cuboid products start inside the carton, and all four top flaps are folded closed but untaped. The robot must peel/pull material from a normal loose packing-tape roll, press one connected strip along the seam and at least 35 mm down each end, cut it with the guarded cutter, then release the tools and leave the seal retained. There are no pre-laid strips, software tape deposition or extra grasp handles.

This is a development task, not a completed demonstration. The earlier pickup/placement prefix is removed. Starting contents and closed flaps earn zero credit. Native preloaded/open-flap stability passed, but its closure probe failed; the revised closed-flap baseline and ordinary-roll manipulation still need native validation.

## Setup and approximation

Carton: 220 × 180 × 100 mm, 3 mm board, twelve rigid panels with crease joints. Contents: blue 80 × 50 × 30 mm, 100 g cuboid and orange 60 × 40 × 30 mm, 70 g cuboid, both on the two-layer bottom. The weak top-crease rest is 10 degrees open; joints are not locked shut. Tape roll: 48 mm width, 50 micrometre film, 100 mm outer diameter, 76.2 mm open core. The outer 360 mm is 72 finite rigid film cells; remaining wound supply is conserved analytically. The end starts flush, not as a prepared tab. Tool: ordinary 55 g guarded cutter on a pad. Sizes, fixed poses and colors are in config.py/task.py.

Adhesion requires measured contact pressure of at least 0.5 N for 100 ms, on the adhesive face within 0.5 mm. Native force-coupled bonds and blade crossings—not controller phases—supply coverage and cut measurements. These adhesive parameters are uncalibrated approximations, not a validated material model. Coarse rigid film/hinges replace thin-film deformation.

## Score

The two products must remain contained, the flaps closed within 5 degrees with inner flaps below outer flaps, and the tape connected and within its strain/capacity bounds. Initial placement or initial closure alone earns nothing.

- 0.25: at least 11 of 44 seam cells have at least 80% width bonded from one connected material component with at least 55 mm actual payout.
- 0.5: all 44 seam cells meet that coverage.
- 0.75: one blade-cut, detached strip supplies the complete seam and both 35 mm end wraps.
- 1.0: retain the complete seal with hands clear, tools parked and carton speed below 10 mm/s for 2 seconds.

Broken creases, drops, excess strain, nonfinite state or material-accounting violations invalidate the run. A nonzero score does not establish full completion.

## Validation commands

Use the pinned Isaac Python and a fresh output directory:

```sh
/data/IsaacLab-Arena/.venv/bin/python run.py --mode settle-probe --no-video --out /home/ubuntu/yam-eval/sealing-closed-settle-001
```

`closure-probe` remains an explicitly separate open-flap engineering diagnostic; its layout is not the revised sealing setup. `bench` and `adhesion-probe` use diagnostic loads/fixtures and cannot receive robot task credit. Camera-free runs capture the composed scene and actual native actor poses for clearly labelled offline previews. They do not provide RGB observations or a completed model run.

Full ordinary-roll peeling, application and cutting remain unproved. No full-score demo or Dream score is claimed.

Ordinary-roll checkpoint (not sealing; fresh output directory):

```sh
/data/IsaacLab-Arena/.venv/bin/python run.py --mode roll-pickup-probe --no-video --out /home/ubuntu/yam-eval/sealing-roll-pickup-001
```

The first native prefix timed out after 372 controls/18.6 simulated seconds, during closure of the fingers. One opposing-contact frame was observed; no lift, peel or seal was demonstrated. Its contact snapshot is pre-action, while actor poses are post-step; timestamps are retained. The force gate sums all 73 configured roll-family filters, so it does not depend on individual filter-column ordering, which is not independently read back. The checkpoint is explicitly unscored.
