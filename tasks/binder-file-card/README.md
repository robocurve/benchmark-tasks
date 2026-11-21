# File a punched card in a ring binder: snap the rings open, hang it on all three, press them shut

A standard 1 inch round-ring binder (Letter size) lies open flat on the table, taped down, its spine running away from
the arms and its three rings shut. On the right, beyond the binder's back cover, a three-hole-punched Letter-size card
waits in the front slot of a wire step file (a desktop incline sorter), standing on its punched edge. The instruction states only the goal: *File the punched card
in the binder: put it on all three rings and close the rings.*

Nearly every student and office worker files punched sheets in a ring binder. It is not pick-and-place:

- **The rings are an over-centre toggle.** The ring halves are fixed to two steel plates hinged together under a
  sprung cover. The plates can never lie flat, so the rings are held either shut or open. A ring opens only when its
  two halves are pulled apart hard enough (about 12 N at the tips) to push the plates through the centre, and then it
  snaps open. All three rings move together.
- **Three holes on three thin wires at once.** The card's holes are 7.1 mm across and 108 mm apart. The 4 mm wire
  halves have to go through all three together, which leaves about 1.5 mm of play at each hole, across a 216 mm card
  held by its far edge. The card then has to swing round the wire onto the page.
- **Shut again with the tips meeting.** The halves must be pressed together until the rings snap shut, with nothing
  caught between the tips.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the objects, the ring mechanism's joints, `success()` and `score()` |
| `binderkit.py` | Rigid bodies made of several shapes (boxes and capsules) joined by PhysX joints: hinges and a sprung slide (adapted from the bodykit module used by other tasks in this repository) |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline checks (no GPU): the mechanism's kinematics and snap force, every scoring outcome and attempts to score without doing the task, the runtime's ending, the layout, the demo's reach, and that `task.json` states the numbers in the code |

`binderkit.py` hooks `yam_arena.scene` and the embodiment's reset and step without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and the `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/binder-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Set
`BINDER_DEBUG=1` to print, every 10 steps, each ring's tip gap, which holes have a ring through them, the score's
conditions and the contact forces on the ring plates.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1800 --cam 640x352   # demo_*_cam.mp4, setup.png, result.json
DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record      # prints the CAL numbers (see below)
python -m pytest -p no:cacheprovider test_scoring.py             # offline checks
```

1. Both hands pinch the far ring, the left hand its left half and the right hand its right half. Each comes from
   outside and above, along the half's radius, 50 degrees up from its outermost point, with the fingers closing across
   the wire. Both pull outward until the rings snap open. The plates are shared, so all three rings open.
2. The right hand takes the card out of the step file's front slot by the middle of its top edge, lifting it over
   the slot's lip.
3. It holds the card over the rings, leaning toward the right page and square to the right halves' tips, with its back
   6 mm beyond their ends. It lowers the punched edge between the open tips.
4. It slides the card onto the tips, then swings it 30 degrees about the rings' centre toward the right page, so the
   three right halves go through the three holes.
5. It lets go and draws back. The card falls onto the right page, hanging on the rings.
6. The left hand straddles the middle ring with open fingers and squeezes its halves together until the rings snap
   shut. Both arms go back to their ready poses.

The demo runs open loop: it reads no simulator state. Where the card sits in the hand after it is taken from the
step file is measured once with `DEMO_CALIBRATE=1` and fixed in `CAL` in `demo.py`. The layout is fixed and the simulation
is deterministic. `result.json` reports `score()` on the last step as the trial's score.

## Scoring

Evaluated every control step. The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **1.0** once all of these have held together for 1 s (20 control steps in a row):
  - the card is on all three rings: each ring's wire passes through one of the card's holes;
  - all three rings are shut: each ring's two tips are within 1 mm of each other;
  - no part of either gripper is within 2 cm of the card or of any ring's wire (in simulation, every vertex of both
    grippers' finger and body collision hulls);
  - the card and the ring tips are still: neither moved more than 1 mm over the preceding 1 s.

  `score()` gives 1.0 from the step on which this has held for 1 s. `success()` is true only on steps where `score()`
  is 1.0. The runtime ends the episode once `success()` has stayed true for its own 1 s, so an episode can only end
  with success at 1.0.
- **0.75**: at some point the card was on all three rings with all three rings shut.
- **0.5**: at some point the card was on all three rings (open or not).
- **0.25**: at some point the rings were open: every ring's tips were 10 mm or more apart. A ring part-way open
  (pulled but let go before the snap) springs back shut and earns nothing.
- Otherwise 0. A card on only one or two rings earns nothing beyond the open rings. A card hooked on the tips but not
  through the holes, or lying across the shut rings, earns nothing. Nothing scores a permanent zero.

In a real trial: a ring is on the card when its wire visibly passes through the card's hole (photo). The tip gap is
checked with a ruler or feeler card: shut means the tips touch with no visible gap (under 1 mm). Open means 10 mm or
more, read from the video against the ruler. The 2 cm clearance is checked on the video against the ruler. "Still" means nothing moves on the
video for 1 s. Nothing in the scoring is hidden inside a part.

## How it is modelled

All parts are rigid bodies at real size and mass. The binder is taped to the table and the step file stands on it; both are fixed in the model.

**The ring mechanism** (`task.py`, "the binder"). The two ring plates are free rigid bodies joined by PhysX joints. These
are maximal-coordinate joints, so the closed loop is fine.

- The left plate is hinged to the binder at its outer edge (the cover's curled edge). The hinge has two stops: shut
  (where the tips meet) and open (22 degrees further, where the plates meet the cover; the tips are then 22 mm apart).
- The two plates are hinged to each other along the centre line.
- The right plate's outer edge is held at the cover's height and pushed toward the left plate by the cover's spring. It
  is modelled as a soft spring (2000 N/m) preloaded to 142 N, so its push changes by under 1% over the plates' travel.
- Each plate is 13 mm wide, and they are 11 degrees from flat when the rings are shut. Going from shut to open takes
  the plates through flat, which compresses the spring furthest. That is the over-centre snap, and it comes from the
  geometry and the spring alone: nothing is scripted. Pulling the tips apart takes 12 N at the start (`OPEN_FORCE`, an
  estimate), falls to zero at the centre point (tips 11 mm apart), and then the spring drives the rings open. Shutting
  them is nearly the mirror image (13 N at the start). `test_scoring.py` checks this from the kinematics.
- Each ring half is a 4 mm round wire, a semicircle on a 14.7 mm radius (25.4 mm inside diameter), built from 12
  capsules. The two plates do not collide with each other, so the shut stop is a joint limit at the angle where the
  tips meet. They collide with everything else.

**The card** is a rigid 0.5 mm plate, 279 x 216 mm and 12 g, built from boxes. Its three 7.1 mm holes are real gaps in
its collision shapes: octagons made from the boxes around them. A ring is on the card only if its wire actually passes
through a hole.

**Simplifications.**
- The card is rigid. Real card stock is 0.25 mm thick and flexes, which makes threading it easier.
- The stops are joint limits. Real plates stop against the cover and the tips stop against each other.
- The step file's mesh walls are modelled as solid 2 mm plates (base, a 10 mm front lip, dividers 160 and 230 mm tall,
  a 300 mm back).
- The spring force is an estimate.
- The grippers hold the wire by friction. The wire's material is set to 1.0 with PhysX's "max" combine rule, so the
  rubber pads on the nickel-plated steel wire have a friction coefficient of 1.0 (dry rubber on steel is typically
  0.8 to 1.0). The same rule makes the card slide on the wire at 1.0, more than real paper on steel (about 0.3 to 0.5);
  that only makes the card's swing onto the page harder.

## Numbers and where they come from

| Quantity | Value | Source |
|---|---|---|
| Binder | US 1 in round-ring, Letter: covers 292 x 257 mm, 36 mm (1.4 in) spine, 3 mm board | Product listings for standard 1 in economy round-ring binders (for example the Avery Economy 1" round-ring binder) |
| Ring spacing | 108 mm (4.25 in) | The North American three-ring standard |
| Ring inside diameter | 25.4 mm (1 in) | The binder's size |
| Ring wire | 4.0 mm | Estimate; measure with calipers |
| Mechanism | 250 x 32 mm cover, 7 mm tall; plates 13 mm wide | Estimate from typical 1 in ring metals; measure |
| Plate tilt shut / open | 11 degrees each way from flat (tips 22 mm apart when open) | Estimate (no maker figure found; a 1 in ring opens to roughly its own radius or more); measure the open tip gap |
| Opening force at the tips | 12 N (`OPEN_FORCE`) | Estimate: no maker or published figure found (ring-mechanism patents describe the snap only as strong); measure it as below |
| Card | Letter 279 x 216 mm, 110 lb index card stock (199 g/m2), 12 g | Standard sizes; 0.0603 m2 x 199 g/m2 |
| Holes | 7.1 mm (9/32 in), centres 9.5 mm from the edge, 108 mm apart | A standard three-hole punch |

### Measuring the snap force

Hook a small luggage scale (or a spring scale reading to 0.1 kg) through one shut ring, round one half just below its
tip. Hold the other half's tip with a loop of string tied to a fixed point, pulling the opposite way. Pull the scale
slowly, square to the ring, and note the highest reading before the ring snaps open. Opening force (N) = reading (kg) x
9.81. Do it ten times and put the median in `OPEN_FORCE`: the spring's preload scales with it. Measure the tip gap of a
fully open ring with a ruler to check `OPEN_DEG`.

## Real-world set-up

All parts are bought:
- a standard 1 in round-ring Letter binder, opened flat and taped to the table with two strips of masking tape across
  its covers;
- a Letter-size wire step file (desktop incline sorter, for example the Fellowes Wire Step File 72614, about
  308 x 257 mm and 300 mm tall), standing beside the binder;
- three-hole-punched Letter card stock (or a sheet of 110 lb card stock punched with a standard three-hole punch).

## Reset

Shut the rings with no card on them by squeezing any ring. Stand the card back in the step file's front slot, punched edge
down against the lip, centred on the slot, leaning back on the divider. The card is not consumed.

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`), so there are no seeds to choose. In the recorded demo (scores printed every 100 steps) the rings are open
by step 200, the card is on all three rings by step 500, the rings are shut with it on by step 800, and the score
reaches 1.0 at step 855; the runtime ends the episode with success at step 874 (44 s). A trial runs at most 1,800
steps (90 s).
