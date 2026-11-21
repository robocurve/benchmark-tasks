# Clear the hair from under a sink's open pop-up stopper: feed the hooked wand in under its head and draw the hair out

A bathroom vanity with a Kohler Caxton under-mount basin and a Kohler K-7114 pop-up drain. The stopper stands open.
Below it, out of sight in the drain, a clump of hair is caught round the stopper's stem and pivot rod: the usual
cause of a slow bathroom sink. A FlexiSnake Drain Weasel PLUS hair-clog tool (reusable handle with its 12-inch
disposable wand) lies on the countertop.

Instruction: `Clear the hair from under the sink stopper, then leave the tool and the hair on the counter.`

The stopper's head is wider than the drain's throat, so from above it covers the drain. The thin, flexible wand has
to go in under the head's edge, bend down the bore beside the stopper's guide fins, reach the hair, and come back out
with it; the hair cannot be reached with the fingers, and the stopper cannot be lifted out (its pivot rod runs
through it). The maker's package insert: "Insert The Drain Weasel into sink, shower or tub drain opening. The Drain
Weasel will slide past most drain pop-up stoppers and grates." and "Carefully pull snake and debris from the drain."

Category: Societal relevance (a household chore most people with a bathroom sink face). O*NET 47-2152.00 (Plumbers,
Pipefitters, and Steamfitters) lists as a core task "Maintain or repair plumbing by replacing defective washers,
replacing or mending broken pipes, or opening clogged drains." This task is the household version of the last part.

## Real-world setup

Metres; x forward from the line through the two arm bases, y to the robot's left, heights above the robot's table
top (the vanity's cabinet stands on it). Two layouts (lots) differ only in where the tool lies; lot A is the
screenshot.

- **Vanity:** a 30 in vanity with a 31 x 22 in (787 x 559 mm) stone top, 30 mm thick, its surface 243 mm above the
  table top, front edge at x 0.105, left end at y 0.3735, right end at y -0.4135. The cabinet (closed doors) is
  cut off at the robot's table top: in a real trial the arms stand on a platform so that their bases are 243 mm
  below the countertop (for a 34 in, 864 mm, counter: bases 621 mm above the floor). A wall rises behind the top,
  with a 100 mm stone backsplash.
- **Basin:** Kohler Caxton K-2205 under-mount vitreous china sink (maker's sheet: basin area 17 x 14 in, 432 x
  356 mm; water depth 6-3/8 in, 162 mm; 1-3/4 in, 44 mm, drain hole; no overflow). Long side along y, its oval
  opening cut in the top, the drain at (0.360, -0.020), the basin's bottom 51 mm above the table.
- **Faucet:** a single-hole faucet behind the basin, its spout reaching over the bowl 110 mm above the countertop.
- **Drain:** Kohler K-7114 pop-up drain without overflow (maker's sheet: 2-1/8 in, 54 mm, flange; 1-1/4 in
  tailpiece), chrome brass. Flange 4 mm proud of the basin's bottom, its top 55 mm above the table; throat and body
  bore 32 mm (estimate). The pivot rod (3/16 in steel) enters from the back 50 mm below the flange's top (estimate)
  and runs through the stopper's eye. The tailpiece is cut off at the table top, 55 mm below the flange's top (a real
  one runs on 102 mm to the trap).
- **Stopper:** the K-7114's pop-up stopper, standing fully open: domed head 42 mm across (estimate), its underside
  20 mm above the flange's top (estimate), a 36 mm gasket under it; four dark guide fins 30 mm across and 2 mm thick,
  reaching from just under the head to 30 mm below the flange's top, at 30 and 120 degrees from +x (45 degrees either
  side of the side the wand comes from); a 9 mm stem down to the eye. It is held open by its rod and does not move.
  The rod through the eye fixes the stopper's turn; the fin angles are an estimate, checked and marked on the real
  stopper (reset step 2).
- **Hair clump:** 0.50 g of human hair (cut from a 100% human-hair weft, 150 mm strands), wet, wound three turns round
  the stopper's stem among and below its fins and over the pivot rod, so it fills the bore from 22 to 52 mm below the
  flange's top (dark brown in the video).
- **Tool:** FlexiSnake Drain Weasel PLUS hair clog tool (UPC 897020002066; maker's package insert: reusable handle,
  12 in and 18 in disposable wands; the current starter kit, with 18 in wands, lists at about US$20-24 and 0.14 lb),
  fitted with a 12 in (305 mm) wand. Current retail refills (3-packs and 12-packs) are 18 in only, so a real trial
  trims an 18 in refill to 305 mm at its handle end, keeping its hooked end whole; the 18 in length would need the
  handle about 0.5 m above the table, beyond the arms' reach at this vanity. Orange handle 133 mm long (estimate): a 45 mm sleeve about 30 mm square at the
  wand's end, a 26 mm round grip, a 30 mm end cap. Wand 305 mm, 3 mm thick (4 mm with its hooks), its last 32 mm the
  dark hair-snagging pad (6 mm wide). In the simulation the handle is 40 g and the wand 16 g (estimates; a real
  wand is lighter, which moves the tool's balance point toward the handle; reset step 5 checks it rests).
  - Lot A: lying on the countertop's front strip along the front edge, its wand pointing to the robot's right, the
    wand meeting the handle 45 mm in from the countertop's left end (at x 0.135, y 0.3285, its axis 15 mm above the
    stone): the square sleeve rests on the stone and the grip and cap reach 88 mm past the top's end, as a tool is
    often left with its handle over the edge. The cabinet's side is below that edge.
  - Lot B: the same, 20 mm further right.
- Both arms start at their ready pose with the grippers open. Nothing else is on the countertop.

## Scoring (from video, a top photo, a straightedge, a ruler and a scale)

**Score 0 if, at any moment:** the tool drops off the vanity (any part of it more than 20 mm below the countertop
outside the vanity's outline: on the floor or hanging off the top's edge).

**Otherwise, the end state.** Every tier needs both grippers at least 10 mm from the tool and the tool still for the
last 1 s (under 3 mm/s, drifting under 1 mm). "The hair" means at least 0.40 g of the 0.50 g clump, weighed dry on a
0.01 g scale after the trial (what is on the pad, on the counter or in the basin, and what is fished out of the
drain, are weighed separately). If the clump splits, each tier needs 0.40 g at the place it names: "out of the drain"
counts all hair weighed outside the drain; "at the countertop's height" counts only hair there. "At the countertop's
height" means inside the vanity's outline and no lower than 10 mm under the stone's surface: on the stone, or on the
pad where the wand's end reaches past the basin's rim.

| Score | End state | How it is read in a real trial |
|---|---|---|
| 1.0 | The hair is out of the drain and at the countertop's height (on the pad or off it), and the whole tool lies on the countertop: the sleeve on the stone and every point of the tool between 10 mm below and 40 mm above the stone's surface (the wand's end may reach past the basin's rim but not hang into the basin) | Top photo; a straightedge laid across the rim with a ruler under it for anything reaching into the basin; the scale for the hair on the counter |
| 0.5 | The hair is out of the drain and at the countertop's height, but part of the tool is down in the basin | Same |
| 0.25 | The hair is out of the drain (outside the flange, or 10 mm or more above the flange's top) but not at the countertop's height: left in the basin, or dropped past the countertop's edge | Same; the scale for the hair in the basin or on the floor |
| 0 | The hair is still in the drain, or anything above with a gripper within 10 mm of the tool, or the tool moving | Torch down the drain; the scale for what is fished out |

Trials are run on lots A and B and averaged.

### The simulation scorer (task.py)

- `observe()` runs after every control step and keeps the trial record: whether the tool has dropped off the vanity
  (`off_vanity`), whether the hair has been taken (below), whether both grippers are clear (`hands_clear`: every
  point sampled 3 mm apart on every gripper face more than 13 mm from every box of the tool, i.e. 10 mm plus the
  sampling), stillness and how long the state has held.
- **Declared rule for the hair:** the hair is soft and the wand passes into it, so the clump is drawn but does not
  collide. It is taken by the hooked pad only when the pad's centre has spent 0.5 s in all inside the clump (inside
  the 32 mm bore, 22 to 52 mm below the flange's top) and is then drawn back up 10 mm or more from its deepest point
  while still in the bore: the snag and the pull. From then on the clump rides on the pad (`hair_pose`, written every
  step in `_put_hair`). Before that it stays where it is; nothing else moves it.
- `tier()` gives 0.25 / 0.5 / 1.0 as in the table (`hair_out_of_drain`, `hair_on_counter`, `tool_on_counter`);
  `score()` returns it only when the record is valid, the tool it is given is the one the last step observed, the tool
  never dropped, the hands are clear and everything has held still for 1 s.

## What the simulation simplifies (fidelity)

Rigid bodies (PhysX). The wand's last 128 mm is eight 16 mm links on spring hinges (0.2 N m/rad each, an estimate
for a 3 mm polypropylene strip) that bend in one plane; the first 177 mm is straight and fixed in the handle. The
links weigh 2 g each (a real wand is lighter) to keep the solver stable. The hair is a declared rule (above): once
taken, the whole clump comes out on one pull, passes the stopper's fins and head without catching, and cannot be
wiped off the pad; its wet tangle, the force needed to drag it out, and hair left behind are not simulated. What is
simulated is the access: the head is wider than the bore and the fins stand in it, so the pad reaches the clump only
by going in under the head's edge and bending down the bore beside the stem. The handle's 360-degree spin (the
maker's step to wind more hair) and water are not simulated. Stopper fixed open. Vanity on the robot's table, its
cabinet cut there.

The scripted demo plays fixed waypoints from the published layout (the lot it is given); it reads nothing from the
simulator. Its constants were measured on the GPU and are stated in `task.py`: held at its insertion slope the tool
sits 4.4 degrees off the grip's pose under its own weight (measured again later at 5.1 and 5.6 degrees in the two
lots), and the wand's hinged end droops 20 mm, so the demo aims that much the other way. Laying the tool back, it
brings the tool round with the wand's end tipped up 8 degrees, so the drooping end crosses the basin's rim above the
stone, and turns it 3 degrees against the tool's turn in the grip. In the recorded demo the tool comes to rest along
the front strip in lot B; in lot A it lies turned 9 degrees, the wand's end and the hair resting on the rim's edge
(the end 5 mm under the stone's surface, inside the 10 mm the top tier allows).

## Recorded runs

Scripted demo (`--policy yam_scripted`, both lots): 1.0 in lot A and 1.0 in lot B. The pad reached the clump 23 to
24 s in and the hair was taken at 28 s in both lots; the episode ended at 40 s, after the 1 s still hold. Dream-1 (both
lots): 0.0 and 0.0. It brought both grippers down in front of the countertop and stayed there, never picking up the
tool, until the step limit.

## Ways round the task, and what they score

| Route | Result |
|---|---|
| Grip the hair with the fingers | Impossible: the hair is 22 to 52 mm down a 32 mm bore under a 42 mm head |
| Lift the stopper out first | Impossible: it is held by its pivot rod |
| Push the wand straight down beside the head | The head covers the bore; the wand lands on the head or the flange: hair stays, 0 |
| Poke the pad into the clump for under 0.5 s, or pull it out sideways without drawing it up | Not taken: 0 |
| Pull the hair out, leave it with the tool in the basin | 0.25 |
| Pull the hair out, leave the tool hanging into the basin | 0.25 or 0.5 |
| Throw the tool on the floor (even after a full end state) | 0 for the whole trial |
| Lay the tool down still held | 0 |

The near misses in `near_misses.py` play these and the tier boundaries through the scorer.

The task needs one hand; the demo uses the left arm only. A second hand is free to hold the sleeve still, as the
maker's directions describe for the spin step.

## How this differs from close tasks

- Board tasks on drains are plumbing assembly (refitting a J-bend, dry-fitting a slip coupling, sawing a drain pipe),
  fishing a spoon out of a disposal with tongs (reaching into an open opening), and reconnecting a pop-up stopper's
  pivot rod under the basin. None feeds a flexible tool past an obstruction into a concealed bore and draws debris
  out with it.
- TK-MTD23U cleared a blocked washbasin with a plunger (pressure, not a tool that hooks and removes debris); it was
  closed because its fibre-and-soap blockage could not be reproduced. Here the clog is a weighed hair bundle wound to
  a stated place, and the score is the weighed hair that comes out.
- BEHAVIOR's `clean_a_drain` wipes mildew off a sink with a rag and soap; it has no stopper, tool or hidden clog.

## Running a trial

    inspect-robots run --embodiment yam_arena --policy <policy> \
      -E task=tasks/clear-the-hair-below-the-sink-stopper/task.py -E object_jitter=0 -E control_hz=20 \
      --instruction "Clear the hair from under the sink stopper, then leave the tool and the hair on the counter." \
      --scorer task_score --max-steps 1200 --epochs 2

Trial k uses lot k mod 2 (lot A first); `TASK_LOT=A` or `TASK_LOT=B` pins one lot. Runs with `-E object_jitter=0`.
The scripted demo is `--policy yam_scripted` (it uses `tilt.py` for its tilted grips). About 1 minute of simulated
time per trial.

Files: `task.py` (scene, scorer, demo), `parts.py` (the parts kit: hinged parts, a body moved by a step callback, a
loose part rooted at its body), `lots.py` (layouts), `tilt.py` (tilted grips), `near_misses.py` (scored near misses
and shortcuts). No third-party assets.

## Resetting a real trial

1. Under the basin, undo the pivot nut, draw the pivot rod back out of the stopper's eye and lift the stopper out.
   Clear any hair left in the drain (torch and tweezers) and weigh it.
2. Take a 0.50 g bundle of 150 mm human hair (cut from a 100% human-hair weft), wet it and wind it three turns round
   the stopper's stem just under its fins. Lower the stopper back in, push the pivot rod back through its eye so the
   rod passes through the hanging hair, and refit the pivot nut. The rod through the eye sets the stopper's turn:
   check its fins against the marks made on the flange at the first set-up (30 and 120 degrees from +x). The hair now fills the bore 22 to 52 mm below the
   flange's top.
3. Push the lift rod so the stopper stands fully open (head 20 mm above the flange).
4. Fit a new wand (an 18 in refill trimmed to 305 mm at its handle end) to the handle (sleeve up, wand into its
   channel, sleeve down to lock).
5. Lay the tool on the countertop's front strip at its lot's marks: along the front edge, wand to the right, the
   wand meeting the handle 45 mm (lot A) or 65 mm (lot B) in from the top's left end. Let go and check that it rests
   there untouched for 10 s, as the simulated tool does.
6. Both arms to their ready pose, grippers open.
