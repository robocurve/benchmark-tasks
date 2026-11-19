# Peel a baking cup off a muffin

Peel the silicone baking cup off the muffin without breaking it. Leave the muffin and cup separate on the tray.

The native demo scores 1.0 in 640 actions (32 seconds). Its frozen-source audit passes, and native front and side views show peeling, elastic cup recovery and intact release. Dream-1 scores 0.0 after the complete 1,000-action limit; its frozen-source audit and native video review also pass.

## Setup and reset

One cooled plain muffin (nominally 72 mm across, 64 mm high and 45 g) sits inside a flexible silicone cup: 60 mm rim, 52 mm base and 38 mm wall, with two 16 × 24 × 1.5 mm moulded pull tabs. The cup is centred on a fixed 320 × 280 × 4 mm stainless-steel tray, 340 mm forward of the station origin. Both hands start open. Fit clean, snug, food-safe removable silicone covers over the gripping pads; no loose folds or exposed adhesive. Wash and dry these covers and the tray between trials.

`geometry.py` fixes the nominal geometry and coordinates. The cup is a generic dimensioned tabbed cup, not a product scan. [OXO's reusable cups](https://www.oxo.com/silicone-baking-cups-12-pack.html) illustrate the ordinary pull-tab design; a substitute must match the stated dimensions within 2 mm and be used throughout a comparison.

### Prepare a batch ahead

Use the plain, all-purpose-flour and vegetable-oil version of [King Arthur's basic muffin recipe](https://www.kingarthurbaking.com/recipes/mix-and-match-basic-muffins-recipe), with no flavourings, toppings or additions. Weigh 240 g flour, 100 g sugar, 14 g baking powder, 3 g salt, 227 g whole milk, 50 g vegetable oil and 100 g beaten egg without shell. Keep ingredients at 18–22 °C. Combine the dry and wet mixtures separately, then fold them together for about 20 seconds; do not beat smooth.

Our portion/cup protocol differs from the recipe's greased-pan instructions: use clean, dry, **ungreased** silicone cups, with no paper liner or release spray. Tare each empty cup and record its mass. Add **52 ± 1 g batter**, put cups on one metal baking sheet and bake on the middle shelf of a preheated **220 °C conventional oven** for **15–20 minutes**. Check one sacrificial muffin from the centre of the batch with a wooden skewer; stop when it comes out without wet batter. Record the actual bake time and use it for subsequent batches in that oven.

Cool on a rack **inside the cups**, uncovered, for at least 60 minutes to **18–22 °C**. Do not flex, peel or lift trial muffins. Weigh each filled cup and subtract its recorded tare. Accept **40–50 g**, **68–76 mm maximum diameter** and **58–68 mm overall height** (ruler beside the muffin, including the portion inside the cup). Reject visible cracks, a collapsed centre or a cup wall already separated over more than one quarter of its circumference. These are preparation acceptance ranges, not measured results from a physical bake.

Check one extra muffin per batch by peeling it normally: it must remain intact and the wall must require peeling rather than falling away. If this fails, reject the batch. This check is performed on the spare, never on a trial muffin. Recipe, cup model, portion, oven, bake time and cooling interval stay fixed for the comparison. Use accepted muffins **2–4 hours after baking**, held in their cups in a covered tray after cooling. Do not refrigerate or freeze them for these trials. Prepare enough accepted cups ahead; resetting is a fresh muffin/cup swap, followed by cleaning the tray and covers. Replace torn cups.

The simulation uses the nominal dimensions and 45 g mass within these ranges. This revision specifies physical preparation; it does not claim that the coarse adhesion/crumb coefficients have been calibrated to the recipe. The recorded native demo, baseline, scorer and media are unchanged.

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
