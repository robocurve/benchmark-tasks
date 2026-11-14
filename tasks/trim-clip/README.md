# Refit a splash-shield panel with a push-in clip: hold its hole over the body's and push the clip home through both

A loose plastic splash-shield panel lies on a steel body flange, its hole out of line with the body's; a push-in trim
clip stands in a tray. The instruction is only "Fasten the splash-shield panel to the body with the clip." The work is
in the hands: one arm has to slide the panel until its hole sits over the body's and hold it there, while the other
puts the clip into both holes and pushes it home, past the catch that holds it. Let go of the panel, and it slides out
of line as the clip goes in; stop short, and the clip doesn't hold.

## The real step
Splash shields, fender liners and underbody panels are held on by push-type retainers, and refitting them is routine
in automotive service: the [BLS](https://www.bls.gov/ooh/installation-maintenance-and-repair/automotive-service-technicians-and-mechanics.htm)
counts about 805,600 automotive service technician and mechanic jobs in 2024. A typical part is a nylon "Fender Splash
Shield Retainer ... Fits 8mm Hole, 20mm Head" ([Lawson Products P53958](https://www.lawsonproducts.com/products/fender-splash-shield-retainer-nylon-20mm-p53958)).
Done by: automotive service technicians, body repairers.

What goes wrong in real life: the clip goes through the panel but misses the body's hole, or is left standing proud, and
the panel rattles or falls off on the road. A look (head flush, panel tight) checks it.

What we simplified: the flange is fixed. Real retainer barbs are on the clip and flex; here the clip is rigid and the
barbs are two sprung jaws in the hole that the clip's collar pushes apart and then sits under. The holes are 11.4 mm
square, not 8 mm round, so the rigid collar can pass both with up to 1.5 mm of misalignment. The clip is an octagonal
prism made of boxes and weighs 5 g (a real one about 2 g).

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=trim_clip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the splash-shield panel to the body with the clip." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/trim-clip-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=trim_clip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the splash-shield panel to the body with the clip." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/trim-clip-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/trim-clip-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the panel has been fastened, and still, for 1 s;
otherwise it scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| body | fixed: a grey steel flange 16 x 22 cm, 2 mm thick, 4 cm above the table on four posts, with an 11.4 mm square hole; under the hole, the clip's catch (two sprung steel jaws) | centred at (30, 2); hole at (28, 9) |
| panel | loose splash shield, black polypropylene, 8 x 20 cm, 2.5 mm thick, 50 g, flat on the flange; an 11.4 mm square hole 7 cm left of its middle; a 15 mm lip along its right-hand edge | middle at (29, 1.2): its hole 10 mm farther out and 8 mm right of the body's |
| clip | loose push-in clip, black nylon: octagonal head 20 mm across, 3 mm thick; 7.6 mm stem; 9.6 mm collar; 19 mm long in all; 5 g | head up in a tray hole at (22, 20) |

Every loose object has friction 1.2 static, 1.0 dynamic. Both grippers start open at the ready pose. One fixed layout:
`-E object_jitter=0`.

## Scoring
At the end. **Lined up:** the panel flat on the flange (within 3 degrees, its underside within 1 mm of the flange) with
its hole within 1.5 mm of the body's. **Home:** the clip upright (within 5 degrees) with its axis within 1.5 mm of the
hole's, its collar past the catch (so the catch holds it), and its head's underside within 1 mm of the panel.

| Score | When, at the end |
|---|---|
| 1.0 | lined up and the clip home, everything still, held for 1 s, which ends the trial (if time runs out first, this state scores 0.9) |
| 0.5 | lined up with the clip through and held, but its head more than 1 mm proud |
| 0.25 | lined up with no clip through it |
| 0 | anything else, including the panel out of line, tilted or off the flange |

## The scripted demo
The right arm pinches the panel's lip, slides the panel into line and holds it. The left arm, turned a quarter turn so
its wrist camera faces the arms' bases, lifts the clip from its tray by the head, carries it high over the holes and
lowers it straight down until its collar rests on the catch, lets go, closes its fingers and pushes the head home with
the fingertips. The right arm lets go of the lip last.

What the traces taught (official sim, 2026-11-14): a clip made of a square and a turned square is an eight-point star,
which the fingers slid off; carried low, the left arm caught the panel; with the hole and the lip 11.5 cm apart the two
arms met as the left came over the hole; and with 10.8 mm holes the collar stopped on the edges of a panel 2.2 mm out.
The demo aims the slide to correct where the panel comes to rest (2.1 mm and 0.9 mm).

## Mechanism and part modules
`mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request #223 (`tasks/spring-plunger-panel`),
under the MIT License (its header carries the licence); it builds the fixed body with its sprung jaws. `parts.py`
(from the accepted `tasks/install-processor`) builds a loose part from several boxes, for the panel and the clip.

## Runtime and seeds
- The scripted demo's plan takes 19.4 s of simulated time (388 steps at 20 Hz), within the 2,400-step limit; it ends
  on success at about 14 s, once the head has been home and still for 1 s.
- Official runs (2026-11-14): the demo 1.0, ending on success at about 14 s (the panel within 0.2 mm of lined up);
  traced with both arms aiming 1 mm off either way (+x and +y, then -x and -y), also success (panel 0.8 and 1.2 mm
  off, the clip home); Dream-1 0.0 over the full 2,400 steps.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_trim_clip.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
