# Snap one banana off a bunch without dragging the bunch or denting the fruit, and lay it in an open lunch box

Take one banana off the bunch and put it in the lunch box.

## Scoring

Judged at the end from the trial video, a photo and a ruler. Score 0 if: no banana has come off its stem; two or more have; a banana or the crown has fallen off the table; a banana still on the bunch has a dent deeper than 5 mm or a split skin; the detached banana's flesh is exposed. Otherwise exactly one banana is off and the score is the lowest of three numbers. Place: 1.0 the banana lies wholly inside the box, below the rim, let go and still; 0.5 it rests partly on or over the rim or wall, or is inside but still held; 0.25 it is anywhere else (on the table, still held outside the box). Damage to that banana: 1.0 no dent deeper than 5 mm and no split; 0.75 a split under 10 mm at the shoulder; 0.5 a split 10 to 20 mm or a dent 5 to 8 mm deep; 0.25 a longer split or a dent over 8 mm, flesh still covered. Crown moved from its start (tape mark round it, ruler from the mark): 1.0 up to 10 mm, 0.75 over 10 and up to 30 mm, 0.5 over 30 and up to 60 mm, 0.25 more; a crown turned more than 10 degrees about the vertical (protractor, its cut face against the tape line) scores at most 0.75. In simulation a dent is a pinch over 40 N held 0.2 s (5 to 8 mm) or at the gripper's 48 to 50 N limit (over 8 mm); the peel cannot split.

How `score()` reads each condition (banana frame: origin at the stem's root on the crown, the fruit along -y,
curved side down; box frame: origin at the centre of its floor's underside, y along its long side):

- **Off the bunch:** each banana is joined to the crown by a breakable fixed joint at its stem's root. A banana is
  off when its pose relative to the crown is more than 3 mm or 3 degrees from where the stem held
  it (`bunch_kit.detached`).
- **Inside the box:** every one of 100 points on the banana's surface lies within the inner walls (156 x
  221 mm) and below the rim (70 mm). **Over the rim:** any point over the box's outline. Otherwise on the table.
- **Let go:** both grippers' centre points at least 30 mm outside the banana's bounding box. **Still:** moving
  slower than 10 mm per second.
- **Dent:** a hand whose centre point is within 10 mm of the banana's surface, fingers more than 10 mm apart, squeezing
  over 40 N for 0.2 s (a dent 5 to 8 mm deep: damage 0.5), or 48 N or more for 0.2 s (the gripper's limit,
  a dent over 8 mm: damage 0.25). The squeeze is the finger drive's force, 5000 N/m times how far the fingers are held
  short of their target, capped at 50 N. Remembered for the rest of the run.
- **Crown moved:** the crown's centre from where it lay at the first scored step, sideways only: up to 10 mm 1.0, up to 30 mm 0.75, up to 60 mm 0.5, more 0.25.
  **Crown turned:** its heading about the vertical from that step; more than 10 degrees caps the score at 0.75.
- **Fallen:** the crown or a banana more than 50 mm below the table top.

`success()` is a score of 1.0. What loses credit: an unheld bunch slides and turns with the banana being pulled
or bent before its stem lets go, and a crown moved more than 10 mm or turned more than 10 degrees costs 0.75 to 0.25; a hard pinch dents the fruit
(0.5 or 0.25); a banana dropped on the rim or the table, or still held, earns 0.5 or 0.25; taking two bananas,
denting one left on the bunch, or dropping fruit off the table scores 0. Our wrong-way run pinches the same banana and pulls it 80 mm straight along its length with nothing holding the crown, then lets go. It was recorded on the previous version of this task, whose physics, scene and demonstration are identical (only the crown tiers were tightened since). The whole bunch slid 22 mm and turned 5 degrees with the banana, then the stem let go at about 6.5 N of pull (the dragged bunch's reaction bends and twists the stem, long before the 80 N pull limit), and the banana was left on the table: 0.25 by the place tier. Even if that banana had then been boxed cleanly, the 22 mm drag would hold the score to 0.75: full credit needs the crown kept within 10 mm of its mark.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=take_one_banana_off_the_bunch_into_the_lunch_box.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Take one banana off the bunch and put it in the lunch box." \
  --scorer task_score --max-steps 2400 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. Each trial allows 2,400 control steps (120 s at 20 Hz);
the scripted demonstration takes 20.5 s. `pytest` here checks the scorer on named layouts.

## The scene

Products, sizes and positions are in task.json's `setup` (mm, from the midpoint between the arm bases; x forward,
y to the robot's left; heights above the table top). Both arms start at their ready pose.

- **Bananas (four free bodies, 150 g each):** a centre line on a 200 mm radius arc, sections lofted from
  10 stations (15 mm across at the stem to 36 mm at the middle, a five-ridged
  section), 9 convex pieces: green-brown stem, green neck, yellow peel, dark tip. Each lies curved side
  down with its curve's plane leaning 35 degrees. Friction 0.7/0.6. 32 position solver iterations,
  1.5 mm contact offset, depenetration capped at 1 m/s.
- **Crown (150 g):** a rounded block 90 x 32 x 50 mm with the cut stalk's pale face.
- **Stems:** a PhysX fixed joint from the crown to each banana's stem root, with break torque 0.6 N m and
  break force 80 N (`bunch_kit.py`); a broken stem stays broken for the rest of the trial.
- **Lunch box (150 g) and lid (62 g):** an open box 170 x 235 x 70 mm, walls and floor
  3 mm, a 4 mm lip; the blue lid with its four clips lies flat beside it.

## Scripted demonstration

The left hand presses the crown to the table with its closed fingertips (aimed 15 mm into its top, about
15 N in simulation); the right hand pinches the outer banana nearest the robot 150 mm out from its stem and lifts, bending
it up about its stem until the stem snaps. The demonstration reads (never moves) the banana's and the crown's
poses once at the start and the held banana's and the box's poses once over the box.

| Time (s) | Right hand |
| --- | --- |
| 0.0 to 2.5 | over the outer banana |
| 2.5 to 4.0 | fingers down either side of it |
| 4.0 to 5.0 | pinch it 15 cm out from the stem |
| 5.0 to 7.5 | bend it up until the stem snaps |
| 7.5 to 9.0 | lift it clear of the bunch |
| 9.0 to 9.5 | steady |
| 9.5 to 12.0 | carry it over the lunch box, turned along it |
| 12.0 to 13.5 | lower it into the box |
| 13.5 to 14.3 | let go |
| 14.3 to 15.5 | lift the fingers out |
| 15.5 to 17.5 | park |
| 17.5 to 20.5 | hold clear |

| Time (s) | Left hand |
| --- | --- |
| 0.0 to 2.5 | over the crown, fingers closed |
| 2.5 to 4.0 | fingertips on the crown |
| 4.0 to 5.0 | press the crown to the table |
| 5.0 to 9.0 | hold the bunch down while the stem is bent |
| 9.0 to 10.2 | lift off the crown |
| 10.2 to 12.2 | park |
| 12.2 to 20.5 | wait |

In our recorded demonstration (no object jitter) full credit came at 14.9 s: one stem broken, the banana lying in the box, let go and still; the crown and the other three bananas ended where they started (0.0 mm, 0.0 degrees). In three diagnostic trials with every object shifted up to 10 mm: 1.0 each; the stem let go at 5.7 s with the right hand lifting 4.6 to 4.8 N (simulator value), the crown moved 0.0 mm under the left hand's 15 N press and the pinch read 14.7 N.

## Simulation and the real objects

Rigid bodies. Each banana is one rigid body joined to a rigid crown by a fixed joint that PhysX breaks for good at 0.6 N m bending (estimate; real stems 0.4 to 1.2) or 80 N pull, so the stem lets go only when the banana is bent or twisted hard enough. The peel cannot split or bruise: a dent is read from the gripper's squeeze (over 40 N for 0.2 s). Banana shape: lofted convex pieces at true size. Box and lid are thin rigid shells.

- Bananas: USDA grade sizes, "medium" 7 to 7 7/8 in long (https://bananaforscale.info/); Chiquita Cavendish
  bananas (https://www.chiquita.com/); ripeness colour stage 6, all yellow.
- Lunch box: Sistema Lunch 2 L rectangular, 235 x 170 x 80 mm closed, 212 g with lid
  (https://sistemaplastics.com/).
- Estimates, each checkable in a minute: the stem's 0.6 N m bending strength (a kitchen scale hooked round a
  banana of the sample cluster 14 cm from its stem, as in the reset notes, never on the trial hand); the banana's 150 g and 36 mm thickness (scale, ruler);
  the box's wall thickness and 70 mm inner depth (ruler); every friction; the dent thresholds (squeeze a sample-cluster
  banana between two fingers on a kitchen scale and look for the dent after a minute).

## Reset

About 3 minutes. One fresh hand per trial; a hand is never used twice. Source: buy retail bunches of six to eight Cavendish bananas at colour stage 6, 1 to 2 days before, all from one shop display, and keep them at room temperature (20 to 22 C, at least 2 h out of any fridge; cold stems hold harder). With a kitchen knife cut straight down through the crown so that four neighbouring bananas stay joined on one crown piece about 90 mm long (the trial hand, as in the setup) and the other two to four stay on the other piece (the sample cluster); cutting the crown snaps no stem. Stem strength is sampled on the sample cluster only, never on the trial hand: hold the cluster's crown on the table, hook a kitchen scale round its outer banana 14 cm from the stem and lift until it snaps; use the trial hand if the scale read 300 to 800 g (3 to 8 N, 0.4 to 1.1 N m), else set that bunch aside. Reject a trial hand with a split skin, a torn crown or a dent over 5 mm. Lay the hand curved side down, all four bananas attached, with the crown on its tape mark at (130, 140). Empty and wipe the box, stand it at (270, -215), lid beyond it. Wipe the table; change the pad covers if sticky. After the trial the bananas are eaten. Cost about 3 dollars a bunch (one trial); box about 8 dollars.
