# Price-mark the boxed stock: one price label on each box's marked spot, labeller back on its pad

Instruction: "Price-mark the three boxes: each box should end with exactly one price label on the printed price spot of its front face, and the labeller lying back on its pad."

Files in this folder: `task.json`, `price_label_gun_mark_boxes.py` (task file: scene, label rule, scorer, scripted demo),
`labkit.py` (multi-shape parts with sprung joints), `pitch_demo.py` (full-orientation waypoints for the scripted demo),
`arena_assets_fix.py` (library asset path fix), `test_price_label_gun_mark_boxes.py` (offline tests), `README.md`.

## Environment

Lab-Arena 0.3.1 with Sim 6.1 (official launchable), inspect-robots with the `yam_arena` embodiment, station
scene, 20 Hz control. Task file SHA-256 (first 12): 097bcec949ee. The task file needs only numpy and yam_arena to import.

## Offline check (no GPU)

    PYTHONPATH=<yam-isaac-lab-arena checkout> python -m pytest -q test_price_label_gun_mark_boxes.py -p no:cacheprovider

Thirteen tests: the start layout scores 0; one full trigger cycle with the roller on a spot lays exactly one label there;
three labels plus the labeller back on its pad score 1.0; the tiers 0.25 and 0.5; a squeeze in the air, a squeeze with the roller hovering 2 mm off the face, a press without a
full squeeze, a tilted or skewed labeller lay nothing; a second cycle on a box moves its label and the score follows the final place; a label 15 mm off the spot
does not count (8 mm does); the pad credit needs both grippers clear and at least one label; `demo()` is built from
constants only and fits the run limit.

## Run and score one Dream-1 trial

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/price_label_gun_mark_boxes.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/price-label-dream
```

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/price_label_gun_mark_boxes.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/price-label-demo
```

`demo()` (constants only): one gripper closes across the labeller's head, the other closes lightly across the handle back
and the trigger front; both lift it 14 mm off its pad and carry it together. At each box the roller is brought square
onto the printed spot (target 10 mm past the face: the arms' compliance and a little slip in the jaws turn it into a light press), the trigger is
squeezed through its full travel over 2 s and allowed to return over 1.6 s, and the labeller backs off 20 mm. It is then laid back
on its pad, both grippers open and the arms park clear. About 35 s of the 120 s limit.

## Scoring

Judged at the end of the trial (the run ends early once 1.0 has held for 1 s).
- A full trigger cycle (at least 90 % of the travel, then back below 20 %) with the labeller's roller touching a box's front face (within 1 mm of it), inside the face, and the labeller flat and square to the face within 15 degrees puts that box's label where the roller was; each box shows at most one label, and a later cycle on the same box moves its label to the new place.
- 0.25 per box whose label centre ends within 10 mm of the printed spot's centre.
- 0.25 when the labeller lies back on its pad (within 20 mm of its start, flat within 10 degrees, resting on it within 4 mm of the pad surface) with both grippers more than 150 mm from it, only if at least one box scored.
Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Real trial: photograph each box face with a ruler taped beside the printed spot and measure the centre of the label applied last; the labeller's place on the pad against a drawn outline.

## Measured runs

Isaac (abl-warm-31, Lab-Arena, object_jitter=0), probe 2026-11-17T05:30Z of this task file before freeze: scripted demo
1.0 at step 640 (labels -4.8/-5.3/-6.2 mm along the face from the spot centres); Dream-1 0.0 over 2,400 steps (the labeller is
never picked up). MuJoCo mj_check: DEMO 1.0 at step 674, all named checks pass. task.json scores: 'Dream-1 run: 0.0; demo: 1.0'.

## Runtime and seeds

Runtime per trial: about 35 s of simulated time for the demo; run limit 120 s (2,400 steps at 20 Hz).
Seeds: none (fixed layout, object_jitter=0); with jitter, seeds move only the labeller (the only loose object).

## Reset between trials

Peel the labels off the boxes, lay the labeller back on its pad in the drawn outline, trigger released.

## Real-world build

A standard one-line 8-character hand price labeller, e.g. the Monarch 1131 (USD 80.79 gun only, USD 92.78 with a label
roll, Staples item 865735, https://www.staples.com/monarch-1131-labeler-1-line-black-925072/product_865735, read
2026-11-17; 0.44 x 0.78 in labels), real_trial_cost_usd 93; three small closed cartons with a 30 x 20 mm spot printed or drawn on each front face, and a
dark mat as the pad. Sources:
- Labeller size and mass class (235 x 145 x 51 mm, about 0.2 kg empty; 21 x 12 mm labels): one-line labeller seller
  listing (https://pricemark.com.au); a maker page of the same class lists 19.8 x 11.2 mm labels and 14.6 oz loaded
  (https://shopmonarch.averydennison.com/monarch-1131-labeler). The model uses the lighter 0.285 kg class mass; a
  Monarch 1131 is listed at 14.6 oz (414 g) loaded -- both are well inside what two grippers carry.
- Trigger force: no maker figure found. Hand-tool guidance: "Maximum grip force for a trigger should not exceed 4 pounds
  (1.8 kg)" (ehstoday.com, Ergonomic Guidelines for Selecting Hand and Power Tools). A whole-hand labeller lever also
  drives the print and peel train, so 25 +- 10 N at the trigger pad is used and disclosed as an estimate; the MuJoCo
  stage-0 probe held and squeezed it up to 35 N and failed at 45 N.
- Stockers and order fillers: BLS OEWS https://www.bls.gov/oes/current/oes537065.htm; O*NET 53-7065.00.

## How it is modelled (fidelity)

| Mechanism | Model | Source |
|---|---|---|
| Trigger | hinge on the handle, 20 deg travel (about 21 mm at its pad), return spring 25 N at the pad at full travel | no maker figure: disclosed estimate 25 +- 10 N; hand-tool guidance 4 lb (ehstoday.com); MuJoCo stage-0 held and squeezed it up to 35 N, failed at 45 N |
| Print and feed | one full cycle (>= 90 % travel, back below 20 %) issues one label | how a one-line labeller works (Monarch 1131 page above); feed, ink and backing paper not modelled |
| Applicator roller | 16 mm cylinder at the nose; the label is laid only while it touches the face (within 1 mm), square within 15 deg | the label goes where the roller is pressed (labeller use); the 1 mm band is contact tolerance, not a hover allowance |
| Label | rigid 21 x 12 mm tile on three slide joints of its box, placed by the rule where the roller was | position-rule precedent: merged #1893 (carton-tape-seal) |
| Boxes, pad | fixed bodies | warning: a real carton is light and is steadied by the press |

Two-hand adaptation of a one-hand tool: the two parallel grippers cannot wrap a pistol grip, so the labeller is carried
lying on its side, one gripper holding the head and the other closing across the handle back and the trigger front to
squeeze it. A person does the same job one-handed.

The labeller is one rigid body (head with the roller, handle) with a hinged trigger on a return spring; printing, label
feed and backing paper are not modelled. A label is a rigid 21 x 12 mm tile on three slide joints of its box (along the
face, up the face, out of the face) that the label rule drives to the roller's contact point, as the tape cells of
merged #1893 (carton-tape-seal) stick by a position rule; there is no adhesion or film. Boxes and pad are fixed bodies.

## Nearest tasks (read 2026-11-17, robocurve/benchmark-tasks)

- #2289 (open; ruled "Not accepted (TK-GYJTRN)" 2026-11-16T21:58Z for clarity: "The simulated labels and tabs have colours
  but no readable weekdays"; revision received 2026-11-17T03:54Z): "peel a day label off the dispenser at its bar and press
  it flat onto the tub's lid" -- the fingers peel a label tab and press it down. Here nothing is peeled by hand: the
  labeller's trigger stroke prints and feeds the label and its roller rolls it on, so the skill is holding a tool's roller
  square on a printed spot through a full trigger squeeze, three times, and putting the tool back.
- #1952 (open; ruled "Not accepted (TK-THAL8U)" 2026-11-15T18:18Z: "Shirt should be modeled more accurately, currently
  oversimplified compared to real-world context"): a tagging gun pushes a barbed plastic fastener through its hollow
  needle into a care label. This task has no needle, no piercing and no fastener: a roll-on label under a light press on
  rigid cartons.

## Known warning: start jitter

mj_check ROBUSTNESS (start jitter +-5 mm / 3 deg on the labeller, friction +-30 %): 7/7 at 1.0; extra seeds 5-14 and
friction +-40 %: 11/12 (seed 10: the right jaw lands on the trigger and nothing is labelled). The official run uses
object_jitter=0. The demo is sensitive to how firmly the trigger jaw rests between squeezes: a rest 1.0-1.5 mm off the
touching span gives 7/7; 0-0.5 mm gives 5-6/7, 2 mm 3/7. Construct notes (warnings): the label is a rigid tile the rule
places where the roller was (precedent merged #1893); printing, feed, peel and adhesion are not modelled; the boxes are
fixed bodies; the three items are identical cartons (mixed item shapes would need a model change).

## Licence

All geometry is built from primitives in the task file (no third-party assets). labkit.py, pitch_demo.py and
arena_assets_fix.py are our own helpers (see their headers).
