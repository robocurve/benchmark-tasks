# Open a drink can by its stay-on tab: pinch the tail inside the lid's recess and lift it until the seal breaks

A closed drink can stands on the table, held upright. The instruction is only "Open the can." Opening one is
everyday, but physically fiddly for a robot: the tab's tail lies nearly flat inside a lid recessed 6 mm below the rim,
the fingertips have to get round it without touching the rim, it has to be lifted on its arc about the rivet (not
straight up), and it takes a pull to break the seal before the panel folds into the can. There is no rule to know,
only the manipulation.

## The real step
The stay-on tab is a lever riveted to the lid: lifting its tail presses its nose down on the scored panel until the
seal breaks and the panel folds in. Wikipedia ([Beverage can](https://en.wikipedia.org/wiki/Beverage_can)): the tab
"uses a separate tab attached to the upper surface as a lever to depress a scored part of the lid". Done by: nearly
everyone, every day.

What goes wrong in real life: the tail slips out of the fingers before the seal goes, or the tab is pulled straight up
and bends or breaks off with the can still shut. A look at the lid (panel in, tab up) shows the result.

What we simplified: the can is fixed in place. The tab, panel and seal are a jointed mechanism, not metal: the tab and
panel are rigid plates on hinges, and the seal is a sprung catch under the panel's edge that the panel cams aside. The
tab is thicker than a real one and its tail is a 3.6 mm round edge standing 2.4 mm off the lid, so the 6 mm fingertips
can pinch it; the tab and panel weigh 3 g each (real ones are lighter) for stable contacts.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_drink_can.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/open-drink-can-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_drink_can.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/open-drink-can-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/open-drink-can-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the can has been open, with its tab up, for 1 s;
otherwise it scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| can | a closed 12 fl oz (355 ml) aluminium drink can, red, 6.6 cm across, 12.27 cm tall, upright in a holder (fixed) | (26, 0) |
| lid | 5.1 cm across inside the rim, recessed 6.3 mm below the rim's top | on the can |
| tab | stay-on tab, 1.7 x 1.2 cm, riveted at the lid's centre; rolled tail (3.6 mm across, underside 2.4 mm off the lid) away from the arms (+x), nose over the panel | on the lid |
| panel | the scored opening, 1.7 x 1.4 cm, on the side nearer the arms (-x), hinged 4 mm from the rivet | in the lid |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`. A fresh closed can for every
trial.

## Scoring
Measured on the can at the end: how far the tab's tail is raised about its rivet, and how far the panel is pushed
into the can about its hinge.

| Score | When, at the end |
|---|---|
| 1.0 | the panel pushed in at least 45 degrees and the tab raised at least 10 degrees: opened with the tab. Held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | the seal broken (the panel in at least 10 degrees) with the tab raised at least 3 degrees |
| 0.25 | the tab raised at least 10 degrees but the seal holding |
| 0 | anything else, including a panel poked in with the tab still flat |

The tab only has to be up 10 degrees because the panel turns about 4 degrees for each degree of tab: it is fully in
with the tab at about 20 degrees. The panel and tab stay where they are left (the panel can't come back up), so the
open can is open whether or not the fingers still hold the tail.

## The scripted demo
The left arm comes down over the tab's tail with its fingers 10 mm apart and turned to close along the can's x axis,
inside the lid's recess and clear of the rim, pinches the round tail, and lifts it on its arc about the rivet in 8
degree steps to 80 degrees, holding the last aim for 2 s. The tab and seal hold the arm back, so the tail lags the
aim; the seal breaks, the panel folds in and the trial ends on success while the arm is still pulling. The right arm
stays clear.

## Mechanism module
`mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request #223 (`tasks/spring-plunger-panel`),
by way of the accepted `tasks/install-processor`, under the MIT License (its header carries the licence). It adds a
fixed mechanism with hinged and sliding links, springs and friction to a `yam_arena` task file, and reports each
joint's position to the scorer.

## Runtime and seeds
- The scripted demo's plan takes 16.8 s of simulated time (336 steps at 20 Hz), within the 600-step limit; it ends
  on success at about 11 s, while the arm is still pulling.
- Official runs (2026-11-13): the demo 1.0, ending on success at about 11 s (twice, before and after the rim was
  smoothed); traced with the pinch aimed 1 mm off the tail either way (+x and +y, then -x and -y), also success, with
  the tab up 15.1 and 18.6 degrees and the panel in 56 and 69 degrees; Dream-1 0.0 over the full 600 steps.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_open_drink_can.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
