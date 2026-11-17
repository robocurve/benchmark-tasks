# Cut a cigar cap cleanly

Cut the cap from an unlit cigar without crushing or opening its wrapper, then set it down. The skill is aligning the cap with a conventional cutter and controlling the closing load.

## Setup and reset

Use one intact, capped cigar, approximately 136 mm long and 18.5 mm in diameter; a straight double-guillotine cutter with a 24 mm aperture; a 320 × 340 × 20 mm wooden board; and an 84 mm diameter, 18 mm deep waste dish. Both hands start open. Fit clean, replaceable protective gripper-pad covers. No lighting or smoking is involved.

The board is centred at `(0.34, 0, 0.7575)` metres. The cigar lies along the table's Y axis at `(0.34, 0.09, 0.7775)`, capped end towards the cutter. The open cutter is centred at `(0.34, -0.04, 0.772)` and the dish at `(0.48, -0.05, 0.7505)`. `geometry.py` specifies all parts and fixed coordinates; the setup image shows the settled layout.

Use a new, unlit cigar from the same size and batch for each trial. Keep it in its packaging with a 69% humidity pack at 18–22 °C. Reject pre-split or loose wrappers. Empty the dish, inspect the blade edges and protective covers, open the cutter and restore the photographed layout.

## Judging

- **1.0:** the cap is fully separated, the cut remains within the cap, and there is no wrapper split or loose flap longer than 2 mm. The intact cigar rests on the board or tabletop, with both hands released for three seconds.
- **0.5:** the same clean, intact separation has occurred, but the released outcome has not been reached.
- **0:** all other outcomes, including crushing, a wrapper split or loose flap over 2 mm, or cutting into the cigar body.

Inspect the cut face and wrapper in close photographs with a ruler. In simulation, full severance needs both aligned blades to pass the cap's centre under load; the separated cap must be at least 20 mm clear. Any broken wrapper joint or radial compression over 1 mm invalidates the run. These are disclosed damage proxies, not claimed measurements of actual tobacco.

## Simulation

The cutter, sliders, cigar and yielding wrapper sectors are native colliding bodies. Only blade–cap contact uses an analytic cutting law: the blades exert opposing 10 N resistances with equal reactions on the cap. Each blade must accumulate at least 0.06 J of loaded travel before the cap bond is released. Closing the cutter empty or pushing an unaligned cigar cannot earn credit. The body remains collision-active during cutting and withdrawal.

The cap is a separable 2 mm disc and the wrapper has four compliant sectors. Individual tobacco leaves, arbitrary tear propagation and blade wear are not simulated. Resistance and damage limits are uncalibrated engineering estimates. This establishes a contact-driven manipulation demo, not measured performance on physical cigars. Ordinary joint commands and frictional grasps move the objects; there are no hand attachments or object-pose edits. Dream and the demo use the same scene and material law.

The procedural geometry is original and covered by the included MIT license. [Colibri's double-guillotine cutter](https://colibri.com/collections/all-products/products/cut-color-blades) is a mechanism reference, not a dimensional scan or imported asset.

## Run and score

Use the pinned Python 3.12 / Isaac Sim 6.1 / YAM environment in `ENVIRONMENT.json`:

```sh
python tasks/cut-a-cigar-cap/run.py --mode demo --out /tmp/cigar-demo
python tasks/cut-a-cigar-cap/run.py --mode dream --out /tmp/cigar-dream
python tasks/cut-a-cigar-cap/rescore.py /tmp/cigar-demo
python tasks/cut-a-cigar-cap/rescore.py /tmp/cigar-dream
```

Configure Dream through its normal credential store or the loopback bridge in `dream_policy.py`. Never put credentials in this folder. `make_plan.py` regenerates the joint plan; `demo_control.py` uses measured object poses to align the cigar and blend smoothly into withdrawal.

The limit is 1,000 actions at 20 Hz (50 seconds); the demo uses 760 (38 seconds). The starting layout is fixed, with no random seed. Every run saves unaltered native videos, setup/final images, continuous body-state traces, material events and frozen source files. Rescoring checks source hashes, action bounds, event history and the final outcome. The extra judging cameras are not policy inputs.

Validated on 17 November 2026: demo **1.0 / 760 actions**; Dream-1 **0.0 / 1,000 actions**, 21 requests. Both complete native traces independently rescore to these results and the videos were inspected. Wall times were 12.3 and 21.5 minutes on a shared RTX PRO6000 validation batch. `VALIDATION.json` records measurements and frozen source hashes.
