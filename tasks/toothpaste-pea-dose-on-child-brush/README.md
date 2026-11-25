# Unscrew a kids' toothpaste tube, put a pea-sized dab on a child's toothbrush and screw the cap back on

`task.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. A child's manual
toothbrush lies on its back, bristles up. A partly used Kid's Crest Bubblegum 4.2 oz toothpaste tube lies on its side,
capped. The robot gets the brush ready for a 4-year-old and leaves the toothpaste capped:
- it unscrews the cap (the cap comes off only by turning it on its thread);
- it squeezes a pea-sized dab (0.15 to 0.35 g) onto the bristles, nothing off them;
- it screws the cap back on fully and lets go of everything.

Helper modules:
- `capkit.py`: the tube's threaded neck and the screw cap as convex-hull colliders (see "The cap and the thread").
- `squeezekit.py`: the paste's flow (see "The paste").
- `yamkit.py`: spawns the brush, the tube (an articulated body with four sprung wall panels) and the cap (a free body).
- `gripkit.py`: gives the YAM's finger pads their rubber friction.
- `yamgeom.py`: the arms' own collision hulls, read at run time from the competition environment, as surface points
  for the contact checks.

`test_task.py` checks the scorer, the thread's geometry and the demo plan offline: `python3 test_task.py`.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/toothpaste-pea-dose-on-child-brush/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the toothbrush ready for a 4-year-old, then leave the toothpaste capped." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/toothpaste-demo
```

- Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.
- Scorer checks: `TOOTH_VARIANT=much` runs the demo with a firmer, longer squeeze (too much), `TOOTH_VARIANT=smear`
  with a short light one, and `TOOTH_VARIANT=nothing` keeps both arms at their ready pose.
- `TOOTH_PROBE=<file>` writes one line per control step: the cap's rise, offset, tilt and turn on the neck, whether
  the nozzle is inside the cap, the pressure, the grams on and off the bristles, the arms' gaps to the tube and the
  cap, and the poses.
- The scorer (`task_score`) reports the score at each step; the trial's score is the one at its end.

## Why it matters (Societal relatability)

Putting toothpaste on a brush is something nearly everyone has done: hold the tube, unscrew the cap, squeeze, screw
the cap back on. Parents of young children do it for them twice a day.

**The dose matters.** For children aged 3 to 6 the CDC, ADA and AAP advise no more than a pea-sized amount of
fluoride toothpaste, about 0.25 g; under age 3, a smear the size of a grain of rice. In the US survey behind that
advice, only 49.2% of children aged 3 to 6 used a pea-sized amount, and over 38% used a half or full brush load
(20.6% half, 17.8% full) (CDC MMWR 2019;68(4):87-90, https://www.cdc.gov/mmwr/volumes/68/wr/mm6804a3.htm). Fluoride
swallowed while the enamel is still forming causes dental fluorosis; among US 16-17 year olds, fluorosis graded "very
mild" or worse rose from 29.7% (NHANES 2001-02) to 61.3% (2011-12) (Wiener et al. 2018,
https://pubmed.ncbi.nlm.nih.gov/29500282/).

**The robotic difficulty is control.**
- One hand holds the tube while the other turns the cap, about 1.1 turns, more than one turn of the wrist: the hand
  has to turn, let go, turn back and grip again, following the thread, then put the cap back on straight and all the
  way.
- The dose is set by how far and for how long a soft tube is squeezed. The paste is a yield-stress material: nothing
  comes until the squeeze passes a threshold, and it keeps coming for a moment after the squeeze eases.
- The nozzle has to be aimed at a 20 x 9 mm bristle field.

## Scoring

Paste is scored in grams: the real trial weighs it, and the simulation converts mL at 1.30 g/mL (pastes run 1.3 to
1.6 g/mL), so the density matters only in simulation.

| Score | Condition |
|---|---|
| 1.0 | 0.15 to 0.35 g on the bristle field and at most 0.05 g anywhere else; the cap screwed fully on; the bristles within 45 degrees of facing up; the tube and the brush both resting on the counter (lowest point within 3 mm of it) with no part of either arm touching them or the cap (in simulation every arm link at least 5 mm clear); each let go resting on the counter, not dropped or thrown; all still (under 1 cm/s) for 1 s |
| 0.75 | the same dose, clean and capped, when any other condition for 1.0 is not met (not yet let go, a tube or brush dropped or thrown, the bristles more than 45 degrees from up, or not yet still) |
| 0.5 | a pea-sized dab with the tube left uncapped or its cap not screwed fully on; or a pea-sized dab with more than 0.05 g off the bristles |
| 0.25 | 0.05 to 0.15 g (a smear: the under-3 dose) or 0.35 to 0.70 g (too much: a half load) |
| 0 | under 0.05 g or over 0.70 g; doing nothing scores 0 (the tube starts capped, no paste out) |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Turning the bristles past sideways (more than 90 degrees from up) with paste
on them caps the score at 0.25 for the rest of the trial: the dab can fall off or smear.

**Screwed fully on** means turned down to within 45 degrees of its seat, square and centered on the neck. In
simulation it is read only from the cap's own pose on the neck: its turn about the neck within 45 degrees of the
seated turn, on the thread's last turn (within half the 1.5 mm pitch of its seat), its axis within 2 degrees and 0.6
mm of the neck's. A cap set on without turning, a cap a turn or more short, a cap part way down (also when pushed down
into the thread's play) and a crooked cap each score at most 0.5. In the real trial a tester turns the cap clockwise
by hand after the trial; it must stop within an eighth of a turn and sit square: an even gap between the cap's mouth
and the shoulder all round.

**Why 0.15 to 0.35 g.** The CDC defines a pea-sized amount as 0.25 g. Children of 40-48 months told to use a pea
dosed 0.256 g on average (range 0.035-0.620 g, n = 28; den Besten,
https://www.aapd.org/globalassets/media/publications/archives/denbesten-18-04.pdf). UK parents' usual dose was about
0.36 g (https://eld.bl.uk/catalog/100046207873_0x000001; figures from the abstract). The 0.15 g bottom is half-way
between the CDC pea and the about-0.1 g rice-grain smear for under-3s
(https://www.smilesforlifeoralhealth.org/topic/how-much-toothpaste/).

**Why 0.05 g off the bristles.** That is 5 counts on the 0.01 g scale and about a fifth of a pea: a stray smear, well
above the scale's resolution.

**Why the arms must be off the tube, the cap and the brush.** A cap held on, or pressed onto a loose one, is not
closed until it is let go, and a brush still in a hand is not set down.

**Why put down, not dropped or thrown.** Whenever the arms let go of the tube or the brush it must already rest on
the counter (its lowest point within 5 mm of it) and move under 0.3 m/s.

**What the scorer reads.**
- Paste can leave the nozzle only while the nozzle is out of the cap: with the cap on, or loosened but still over the
  nozzle, a squeeze raises the pressure and nothing leaves; the pressure comes out once the cap is off.
- Paste lands on the bristles only from a nozzle pointing within 60 degrees of straight down and at most 3 cm above
  the bristles, while they face within 60 degrees of up; otherwise it lands off them.
- Touching is contact on the arms' own collision hulls (every link from the shoulder to the fingertips, both arms): a
  hull point within 1 mm of the tube's, the cap's or the brush's outline. The hulls load at run time from the
  competition environment.
- The dose is stored in the brush's own frame, so the score is the same wherever the brush is held or laid
  (`test_placement_sweep`).

## The cap and the thread

The tube is the Kid's Crest Cavity Protection Bubblegum 4.2 oz tube; its listing's directions read "twist off cap to
remove foil seal" (Target, TCIN 1001365458, UPC 037000951438): a screw cap on a foil-sealed tube.

- **Neck and thread.** The 15 mm tube neck takes a "15 x 1,5" neck finish (Amcor's 35 mm screw-on tube closures,
  neck diameter 15 mm, neck standard M): a 15 mm metric thread of 1.5 mm pitch, single start, right hand. Its section
  is the ISO metric basic profile for that size (ISO 68-1): 60-degree flanks, an external thread 0.81 mm deep, crest
  flats of 0.19 mm on the neck and 0.38 mm in the cap.
- **What is approximated.** Each ridge is a helix of convex hulls, one per 10 degrees of turn, each sharing its end
  faces with its neighbors so a ridge slides along the other without ledges. A hull's straight crest stands at most
  0.03 mm inside the true helix. The ridges run out to nothing over their last 45 degrees, as a molded thread starts.
  The cap's bore is a ring of 32 convex wedges with a lead-in at its mouth; its top is one convex disc. The neck's core
  is one convex cylinder with a separate chamfered tip, the shoulder one convex cone; no hull has more than 64 points.
- **Clearances [M].** The cap stands 0.4 mm off the neck's crest and 0.3 mm off its root, which leaves 0.35 mm of
  axial play between the flanks. The neck carries 2.25 turns of thread and the cap 1.25, so the cap frees after about
  1.1 turns (`test_thread`: the cap's and the neck's hulls never overlap seated or along the helix, and a straight
  pull is blocked until then). The trial cap is checked by counting its turns from seated to free.
- **Nothing holds the cap but contact.** No joint, spring, capture zone or snap acts between the cap and the tube.
  Fully on, the cap's inner top face rests on the neck's tip: the stop is that contact. A hand screwing it on turns
  until its grip slips. The cap body runs 32 position and 8 velocity solver iterations (the tube runs 32 position
  iterations); this adds no force, friction or capture.
- **The cap.** A 35 mm polypropylene screw cap on the 15 mm neck (Amcor "35mm Screw Cap": 35 mm, neck 15 mm, PP, for
  tubes with a removable foil seal). Its height and mass are those of the same maker's 35 mm Softline flip-top
  screw-on closure for this 15 x 1,5 neck finish, 20.8 mm and 4.4 g; its inertia is that of a thin-walled cup of that
  size and mass. Both are checked on the trial cap with the caliper and the scale before the first trial [M].
- **Friction.** The cap is polypropylene and the tube's laminate and neck polyethylene: polyethylene on polyethylene,
  clean and dry, 0.2 (Engineering ToolBox, "Friction - Coefficients for Common Materials and Surfaces"), on every
  surface of the tube and the cap. The YAM's finger pads are rubber-faced: 1.0 static and 0.8 sliding (rubber on
  rubber 1.16 and on cardboard 0.5-0.8, Engineering ToolBox). PhysX averages the two touching surfaces.
- **Start.** The cap starts screwed fully on. If the arena shifts the tube at reset, the cap is put back screwed on
  with it before anything moves.

## The paste

`squeezekit` with a toothpaste tube's numbers.

**Walls.** The tube's round body is a rigid core with four wall panels on sliding joints, each with a spring for the
laminate wall. A hand dispenses only by pushing panels in; their push is read from the joints every control step. The
panels' push displaces paste, and the air trapped inside is compressed and pushes back. The panels are held at rest by
their own joint stops until the first control step, so nothing sags while the scene settles.

**Flow.**
- The paste flows through the open nozzle in proportion to how far the pressure inside passes its yield pressure.
- The yield pressure is the pressure needed to push a yield-stress plug through the nozzle: 4 x yield stress x nozzle
  length / bore. The yield stress is the middle of the measured range: about 100 Pa on a model toothpaste (arXiv
  1903.10888) and about 500 Pa on a commercial-style formula (US 5202112). The 7.5 mm bore is a conventional tube
  opening of 0.28-0.32 in (US 9567138).
- As paste leaves, the air re-expands and the flow dies away, so paste keeps coming briefly after the squeeze stops.
- The demo's holding grip stays under the yield and dispenses nothing.
- The panel area, trapped air and flow rate are fitted on the trial tube (below); the real trial scores grams, so they
  matter only in simulation.

**Where it lands.** The paste drops plumb from the nozzle. The plumb line is cut with the bristle plane and the hit
point is stored in the brush's frame on a 1 mm grid of the 20 x 9 mm bristle field, so the dab moves with the brush.
A cell holds a bead about as tall as a pea dab; more slumps into its neighbors, and slump over the field's edge counts
as off the bristles. The dab is drawn as blue tiles on the bristles.

**Fidelity note: the plumb drop.** A real dab leaves the nozzle as a soft ribbon that touches down, builds up and is
broken off when the tube is lifted. There is no ribbon, stringing or tail here. In the demo the nozzle is 4 mm over
the bristles, about where a real 7.5 mm ribbon touches down, so both land in the same place; squeezed from more than
3 cm up, the paste is counted off the bristles.

**Offline doses** (`test_task.py`): the demo's squeeze (1.75 mm a side for 1.2 s, then 1.5 s at the holding grip)
leaves a pea-sized dose; a firmer, longer squeeze (2.5 mm for 1.5 s) too much; a short light one (1.2 mm for 0.6 s) a
smear.

**Fitting the squeeze on the trial tube** (simulation only). Hold the prepared, uncapped tube nozzle-down in the YAM's
gripper at the holding grip over a cup on the 0.01 g scale. Close in 0.1 mm steps a side, 1.2 s each, until the scale
first gains 0.01 g (sets the yield stress). Then command closings of 1.0, 1.5 and 2.0 mm a side for 1.2 s, five
times each, weigh each dose, and fit the panel area, trapped air and flow in `test_task.py`'s offline squeeze until
each mean matches within 0.02 g.

## The demo

Open loop, both arms, as a person does it: the left hand holds the tube and the right turns the cap. The left takes
the lying tube from above, lightly across its body (a holding grip, under the paste's yield), and lifts it level with
the cap toward the robot, over the brush. The right comes in pointing forward and closes on the cap while the left
holds lightly; then the left holds firmly (the cap seals the nozzle, so nothing can flow) and the right unscrews the
cap counterclockwise (seen from the cap's top), half a turn per wrist stroke, following the thread; between strokes it
opens, turns back and closes again, 1.25 turns in all. Before the stroke that frees the cap the left eases back to its
light hold, so no pressure is left to push paste out. It draws the cap straight off and holds it aside, out of the
left arm's way. The left turns the tube nozzle-down over the brush, the nozzle 4 mm above the bristles, squeezes 1.75
mm a side for 1.2 s, eases to the holding grip for 1.5 s, lifts off and brings the tube back level. The right puts the
cap back on the neck, pressing it on lightly, and screws it on clockwise in the same strokes, a little past the seat,
lets go and backs away. The left lays the capped tube down, lets go and lifts clear.

## Real-world build

The real trial scores only the outcome: grams on and off the bristles, the cap's seat, the bristle angle, and the
tube and brush let go on the counter with the arms off them. No force gauge and no squeeze readout are used.

**Products** (all off the shelf):

| Item | Product | Notes |
|---|---|---|
| Toothpaste | Kid's Crest Cavity Protection fluoride toothpaste, Bubblegum, 4.2 oz tube with screw cap (Target TCIN 1001365458, UPC 037000951438) | its directions: "twist off cap to remove foil seal"; remove the foil seal before the first trial |
| Toothbrush | Colgate Kids Extra Soft toothbrush, Ocean Explorer 4-pack (Target A-75563150, UPC 035000458476) | one new brush per trial, or a set of washed brushes in rotation, each used only once fully dry (its weight steady on the scale) |
| Scale | American Weigh Scales AWS-100-CAL pocket scale: 100 g x 0.01 g, with its 100 g calibration weight | calibrate at the start of each session |
| Rule | any 150 mm steel rule with 0.5 mm marks | for the top photo of the dab and checking the marks |
| Tape measure | any metric tape measure | for laying out the marks from the arm bases |
| Caliper | any 150 mm digital caliper | for the tube, cap and brush dimensions marked [M] |
| Thermometer | any room thermometer reading to 1 C | the room must be 20-24 C |
| Also | a phone camera on a stand, masking tape, dry tissues, a small cup, nitrile exam gloves | |

A low-cost setup: the robot and a phone aside, every item is a common household or hobby item.

**Setup marks** (once). From the midpoint between the arm bases, x forward, y to the robot's left:
- Brush: a tape outline of the brush lying on its back, the bristle field's middle at x 300 mm, y +20 mm (within
  5 mm), its long axis along x, head away from the robot.
- Tube: a tape outline of the capped tube lying on its side along x, cap toward the robot, the front end of its round
  body (where the shoulder starts) at x 250 mm, y +110 mm (within 5 mm), the rolled end flat on the counter.

**Starting state** (each trial). Keep the room, the paste and the scale at 20-24 C (a paste's yield stress changes
with temperature).
1. The tube holds 50 to 70 g of paste (gross weight minus the weighed empty tube and cap). Roll it flat from the crimp
   up to the paste, hold it nozzle up and press gently until paste fills the nozzle to its mouth (no air at the tip),
   wipe the nozzle flush, screw the cap on with the fingertips until it just stops (no extra twist), and lay the tube
   at its mark. Let it rest 1 min so the laminate springs back.
2. Lay the clean, dry brush on its back at its mark.
3. Both arms at their ready pose.

**Gripper protection.** Each gripper pad gets a disposable nitrile finger cover cut from an exam glove, replaced
whenever paste touches it and at the end of the session. A 10 s hold check on the tube at the start of the session
confirms the grip.

**Measure.**
1. Before: weigh the brush (B0) and the tube with its cap (T0).
2. Run the trial, recording video from above and from the robot's side.
3. Cap check: turn the cap clockwise by hand; it must stop within an eighth of a turn and sit square: an even gap
   between the cap's mouth and the shoulder all round (mark the cap and the tube's shoulder with a pen line first to
   read the turn). A cap left off or loose fails this check.
4. Photograph the brush head from straight above beside the rule.
5. Wipe any paste off the brush outside the bristle field and weigh the brush (B1): on the bristles = B1 - B0.
6. Weigh the tube with its cap, unwiped (T1): dispensed = T0 - T1; off the bristles = dispensed - (B1 - B0).
7. From the video: the bristle angle at the end, whether the arms are off the tube, cap and brush at the end, whether
   the tube and brush were let go resting on the counter (not tossed), and the 1 s of stillness.

**Safety.** Only a toothbrush and toothpaste are used: nothing sharp, hot or heavy. The brush is never used in a
mouth; discard or wash each dosed brush. The operator keeps a hand on the robot's stop and stops the trial if an arm
drives into the counter, the other arm or the tube.

**Tolerances**

| Quantity | Value | How it is measured | Source |
|---|---|---|---|
| Pea dose (full credit band) | 0.15 to 0.35 g on the bristles | scale, B1 - B0 | CDC pea 0.25 g (MMWR 2019;68(4)); study mean 0.256 g (den Besten); parents' usual dose about 0.36 g |
| Smear band | 0.05 to 0.15 g | scale | rice-grain smear about 0.1 g (Smiles for Life) |
| Too much | 0.35 to 0.70 g (0.25); over 0.70 g scores 0 | scale | half and full loads in MMWR 2019;68(4); 0.70 g is about 3 peas (design choice) |
| Off the bristles | at most 0.05 g | scale | 5 counts of the 0.01 g scale |
| Cap screwed fully on | stops within an eighth of a turn (45 degrees) when turned clockwise by hand, and sits square (an even gap between the cap's mouth and the shoulder all round) | pen mark on cap and shoulder | design choice: a cap that still turns is not closed |
| Arms off the tube, cap and brush | no contact at the end (in simulation every link at least 5 mm clear) | video | design choice: a held cap is not a closed one |
| Bristle angle at the end | within 45 degrees of up | video frame | design choice: dab on top |
| Stillness | under 1 cm/s for 1 s | video | the score is read once nothing is moving |
| Paste density | 1.30 g/mL (sim only) | not needed in the real trial | pastes 1.3-1.6 g/mL (https://library.scconline.org/v021n07/32) |
| Yield stress | middle of 100-500 Pa (sim only) | not needed in the real trial | arXiv 1903.10888; US 5202112 |
| Neck thread | 15 mm, 1.5 mm pitch, ISO 60-degree section; clearances and turns [M] | caliper; count the turns from seated to free | Amcor 35 mm tube closures (neck finish 15 x 1,5); ISO 68-1 |
| Cap | 35 mm across [M]; 20.8 mm tall and 4.4 g [M] | caliper, scale | Amcor 35mm Screw Cap (35 mm, PP); height and mass from the same maker's 35mm Softline Flip Top Tube Closure Screw On (15 x 1,5 neck finish) |
| Tube | 4.2 oz (119 g) of paste new, 60 g left in simulation; body 36 mm across, rolled end 50 mm wide [M] | caliper, scale | Target listing (net 4.2 oz; package 1.45 x 2 x 6.65 in) |
| Nozzle bore | 7.5 mm [M] | caliper | US 9567138 (0.28-0.32 in) |
| Brush | 151 mm long; head 26 x 12 mm; bristle field 20 x 9 mm, 9 mm tall; 14 g [M] | caliper, scale | [M] on the trial brush |
| Fill | 50 to 70 g of paste | scale | reset rule above |
| Setup marks | brush at (300, +20) mm, tube's shoulder at (250, +110) mm, within 5 mm | tape measure, rule | the simulator's start |
| Room temperature | 20-24 C | room thermometer | keeps the paste's yield stress steady between trials |

[M] = checked with the caliper or the scale on the trial object before the first trial; the simulation uses the value
given. No [M] item is part of the real score.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes).
- No randomness in the scripted demo: the layout is fixed (`-E object_jitter=0`).

## Sources and licenses

- `task.py`, `capkit.py`, `squeezekit.py`, `yamkit.py`, `gripkit.py`, `yamgeom.py` and `test_task.py` were written by
  this task's team.
- The robot model and its collision hulls load at run time from the competition environment (arena revision
  0059d02a) and are not redistributed: `yamgeom.py` reads them through `yam_arena.analysis.link_hulls()` and caches
  surface points in the system temp folder, outside the task folder.
- No meshes, images or data files from third parties are in this folder.

## Files

- `task.py`: the scene, the scorer and the scripted demo.
- `capkit.py`, `squeezekit.py`, `yamkit.py`, `gripkit.py`, `yamgeom.py`: the task's helper modules.
- `test_task.py`: offline checks.
- `task.json`, `README.md`.
