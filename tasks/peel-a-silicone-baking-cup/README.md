# Peel a baking cup off a muffin

Peel the silicone baking cup off the muffin without breaking it. Leave the muffin and cup separate on the tray.

The native demo scores 1.0 in 640 actions (32 seconds). Its frozen-source audit passes, and native front and side views show peeling, elastic cup recovery and intact release. Dream-1 scores 0.0 after the complete 1,000-action limit; its frozen-source audit and native video review also pass.

## Setup and reset

One cooled plain muffin, 72 mm across and 64 mm high, inside a flexible silicone baking cup (60 mm rim, 52 mm base, 38 mm wall height) with two 16 × 24 × 1.5 mm moulded pull tabs. It rests at the centre of a 320 × 280 × 4 mm stainless-steel tray fixed to the tabletop between the arms. Both hands start open. The cup has not been loosened. The fixed dimensions and coordinates are in geometry.py.

The muffin is 45 g and the cup is approximately 27 g in the native scene. The tray centre is 340 mm forward of the station origin. `geometry.py` fixes every part and coordinate. This is a generic dimensioned tabbed cup, not a scan of a specific product. [OXO's reusable cups](https://www.oxo.com/silicone-baking-cups-12-pack.html) provide an example of the ordinary pull-tab design.

Use plain muffins from one batch, cooled to 18–22 °C in the same washed and dried cup model. Use a fresh muffin each trial and do not pre-peel or oil the cup. Inspect the cup for tears and wash the tray and removable covers between trials.

## Scoring

1.0 if the cup is fully off and the intact muffin and cup rest on the tray with their centres at least 75 mm apart for 3 seconds after both hands release. 0.5 if adhesion has released around at least three quarters of the cup wall; 0.25 for at least half; otherwise 0. A torn cup, split muffin or detached piece larger than 5 mm scores 0. Judge the released circumference from close views and gentle inspection; use a ruler for separation.

The native cup has sixteen lower-wall adhesion regions. Partial credit counts regions that physically detached under load. The full score additionally checks that the cup bottom detached, the muffin is clear of the cup, both objects are on the tray and the hands released. Adhesion release alone cannot earn full score. A broken cup seam or muffin-cap bond overrides all progress with zero.

## Material approximation

The cup has 32 wall panels joined by passive compliant seams, two moulded tabs and a bottom. Sixteen force-limited wall bonds and one bottom bond resist removal. Four damageable cap sections form a continuous muffin top around its core. Native contacts and friction handle gripping and peeling; no object is attached to either hand.

Material coefficients are illustrative, not calibrated to a particular muffin recipe or retail cup. Angular drive values use USD's per-degree convention. The approximation retains flexible-wall peeling, tool-free bimanual support, loss of grip and damage. The scripted demonstration uses observed settling to correct ordinary joint targets, grips the pull tab while it rests on the tray, and peels while lifting the supported muffin. Dream uses the same physics and its standard cameras/joint observations.

## Run and rescore

Use the pinned Python 3.12 / Isaac 6.1 / YAM environment in `ENVIRONMENT.json`, with the adapter installed. From the repository root:

```sh
python tasks/peel-a-silicone-baking-cup/make_plan.py
OMNI_KIT_ACCEPT_EULA=YES python tasks/peel-a-silicone-baking-cup/run.py --mode demo --out /tmp/cup-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/peel-a-silicone-baking-cup/run.py --mode dream --out /tmp/cup-dream
python tasks/peel-a-silicone-baking-cup/rescore.py /tmp/cup-demo
python tasks/peel-a-silicone-baking-cup/rescore.py /tmp/cup-dream
```

The limit is 1,000 actions (50 seconds at 20 Hz). Fixed layout; no random seed. The CPU rescorer checks frozen source hashes, native break history, contiguous frames, bounded joint commands and the outcome. It is an evidence audit, not a physics replay. Native video inspection is also required. `VALIDATION.json` records the audits and measurements. On an RTX PRO 6000 Blackwell the demo took 366 seconds and Dream took 604 seconds including 21 policy requests. Runtime varies with API latency.
