# Floss a dental training head's lower front teeth with a floss pick: ease it through, wipe both sides, bring it back out

A dental training head stands upright on a plywood board on a shipping scale, facing the robot with its mouth open: a
silicone face mask with soft lips over a 28-tooth typodont with soft gums. The two lower central incisors' facing
sides are coated green. A disposable F-shaped floss pick with a thick ribbed grip lies on the board's front right
corner. The instruction is:
**"Floss between the two lower front teeth of the dental training head with the floss pick: ease the floss through
the contact with a gentle back-and-forth without snapping it into the gum, wipe each tooth's side up and down, then
bring it back out."**

To do it, the robot has to:
- pick up the pick by its grip and turn it so its arms point down, the handle out of the mouth;
- bring it in through the 52 mm mouth opening and land the floss on the tight contact between two 5.4 mm-wide
  incisors, within about 0.6 mm of its centre (the notch above the contact is 1.7 mm wide);
- slide the front arm down between the soft lower lip and the teeth, pushing the lip forward;
- work the floss through the contact with a back-and-forth along the floss, keeping the push light: pushed straight
  down, the contact holds and then lets go all at once and the floss drops onto the gum 2.5 mm below;
- stop short of the gum, press the floss against one incisor's hidden side so it wraps it, and wipe up and down
  between the contact and the gum line; then the other incisor;
- pull the floss back up through the contact (it holds from below until pulled 3 N) and out.

## Changes in this version

- Snap: a mark on a 0.4 mm pressure film strip on the papilla under the contact scores 0.
- Pressed on the head or board: the scale rule applies the whole trial and counts everything on the board.
- Passed: any bare cell in rows 2-4 of either band.
- Coverage: before-and-after microscope photos in fixed jigs, read as 10 x 4 cells a band.
- Run limit 3,600 steps.

## Why it matters

About a third of US adults floss daily and about two-thirds have flossed in the past week (CDC NHANES 2011-14). Floss
picks are an ADA-listed way to do it ("pre-threaded flossers"), and a trial found one "as effective as finger
flossing". Parents and carers floss for others. The guidance is about exactly this motion:
- ADA: "Guide the floss between your teeth using a gentle, rubbing motion. Never snap the floss into the gums." "When
  the floss reaches the gum line, curve it into a C shape against one tooth." "If you floss too hard, you might damage
  the tissue between your teeth."
- NHS: "Regular flossing may also reduce gum disease, tooth decay and bad breath by removing plaque that forms along
  the gum line."
- A home-care aide textbook (CC BY 4.0): "Do not quickly snap the floss between the teeth or you can cause pain or
  injury."
- Floss injury is documented as "flossing clefts" (J Periodontol 1986).

The training head stands in for a person's mouth, as a care manikin stands in for a person. The task tests
force-limited insertion of a taut thread through a tight contact that gives way suddenly, keeping the push low with a
back-and-forth and stopping before the gum; then pressing the thread round a hidden convex surface in a sub-millimetre
gap behind soft tissue, and withdrawing it through the same contact.

## Files

| File | What it is |
|---|---|
| `task.py` | the yam_arena task file: objects, the floss hook (every physics step), the rules and `score()` |
| `floss.py` | the floss as a string in task code, the contact rule, and the floss-side rules (snap, wipe, load); NumPy |
| `geometry.py` | every size and the layout (the incisor model, the band, the pick, the lip) and the scoring constants |
| `build_assets.py` | writes `assets/*.usd*` from `geometry.py` (needs `pxr`, not the simulator) |
| `scenekit.py` | USD bodies and articulations for task files, and the physics-rate hook |
| `demokit.py`, `demo.py` | the scripted demo and the scorer checks |

All assets are built from scratch in `build_assets.py` (no third-party meshes, scans or textures). The robot is
yam_arena's.

## Run a trial

```
source <ARENA_DIR>/yam.env
python build_assets.py                                    # once
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    --scorer task_score --max-steps 3600 --policy ...
```

One trial is one layout (no seeds, no randomisation).

## Dream-1 run

Dream-1 scored 0 with the task's exact instruction over the full 3,600 steps (one run, not retried). Its right arm went
to the mouth empty-handed and never picked up the floss pick. `model_run.mp4` shows the run (three
cameras).

## Scripted demo

```
python demo.py task.py demo_out [--no-record] [--variant snap|press|one_side|straight|leave|push]
```

The demo uses the right arm and reads object poses (the pick's pose every
control step; the floss's position comes from the pick's pose and its arm tips).

1. **Pick up.** The hand comes down over the handle's rear end, which overhangs the board's edge, and pinches the
   thick grip across its 10 mm depth (pads on its two 6 mm-thick edges over the rear 26 mm, the fingers commanded
   6 mm inside it: about 15 N a pad). It lifts it, then rolls it 90 degrees in joint space at a point clear of the
   head so the arms point down (fingers above and below the handle, the hand coming from the robot's right). The pick
   settles about 12 degrees about its handle in the pinch; the hand keeps that roll and the floss's position is
   closed-loop.
2. **In through the lips.** The pick goes in front of the mouth, then in at the incisal edges' height plus 6 mm, the
   handle along x out of the mouth, the floss across the contact with its front arm 2.4 mm in front of the contact.
   Its correction is settled there in free space; from then on the push is bounded about it.
3. **Onto the notch and through.** Down at 0.6 mm/s: the front arm's tapered tip meets the back of the lip's rounded
   top edge and wedges the lip about 2 mm forward (the push allowed 15 mm of correction while the floss is still above
   the edges). Just above the contact it eases off, settles the floss on the contact, then saws +-1 mm along the floss
   at 2 Hz (about +-0.5 mm at the floss) while the floss's height is closed-loop on a target descending 0.5 mm/s, 0.15
   mm/s over the contact's last 0.3 mm (the push is whatever makes the floss creep that fast). Once the floss is 0.6
   mm below the contact zone it stops lowering and holds 1 mm below the contact.
4. **Hug and wipe.** Below the contact the push is bounded mostly downward (about 0.8 N up at most, under the
   contact's hold from below). The pick shifts 0.5 mm toward the left incisor (41) so the floss wraps its facing
   side, then makes one stroke at 2 mm/s down to 2.05 mm below the contact and back up to the contact zone's bottom,
   adding no upward push on the way down; then the right incisor (31). The sideways correction is bounded (about 1 N
   while hugging, 0.6 N otherwise).
5. **Out.** Centred, it pulls the floss straight up through the contact (the target rising 0.5 mm/s until the pull
   reaches the contact's 3 N hold from below), lifts the floss 1 cm above the incisal edges, draws the pick 5 cm out
   of the mouth and holds it.

Checks (`--variant`): `snap` saws through like the demo, brings the floss up under the contact, then drops the hand
12 mm at once; `press` presses the floss into the gum after passing; `one_side` wipes only 41; `straight` strokes in
the middle of the gap without hugging; `leave` does everything but bring the floss back out; `push` pushes straight
down without sawing (the pinch turns the pick long before the contact's 8 N, so it never passes).

Results. The film strip's peak is the floss's largest force on it in any physics step; the press
on the head and board is its largest 50 ms average over the whole trial and the largest it held for 0.5 s.

| Run | Score | Steps | Floss on the film strip, peak (N; marks at 2) | Press on the head and board, peak / held 0.5 s (N; limit 3 held 0.5 s) | Band cells wiped 41 / 31 (needs 80%) | Zero rule |
|---|---|---|---|---|---|---|
| demo (recorded) | 1.0 | 1856 | 1.10 (0.55x) | 3.06 / 0.86 (0.29x) | 98% / 98% | none |
| check: snap onto the gum (`snap`) | 0.0 | 808 | 3.19 (1.59x) | 3.39 / 1.24 (0.41x) | 0% / 0% | snapped onto the gum: the pressure film marked (3.19 N on the strip) |
| check: press into the gum (`press`) | 0.0 | 848 | 1.10 (0.55x) | 21.10 / 12.56 (4.19x) | 48% / 0% | pressed on the head or board: 3 N or more for 0.5 s |
| check: one side only (`one_side`) | 0.5 | 1695 | 1.10 (0.55x) | 2.99 / 0.86 (0.29x) | 100% / 45% | none |
| check: straight strokes, no hug (`straight`) | 0.25 | 1937 | 1.10 (0.55x) | 3.21 / 0.86 (0.29x) | 18% / 35% | none |
| check: left in the gap (`leave`) | 0.75 | 1095 | 1.10 (0.55x) | 1.38 / 0.86 (0.29x) | 98% / 98% | none |

The demo's margins: the film strip's peak, and the press held for 0.5 s, against their limits as above; band cells
against 80%.

Media (in the media folder): `demo.mp4` (three views: the top camera large, the left and right arm cameras beside
it; from the notch on, the top camera is moved to a close mouth-level view, shown as a 2x crop and labelled "Close
view"); `setup.png` (the first top-camera frame); `closeup.png` (the close view as the floss settles on the
contact); `model_run.mp4` (the Dream-1 run, three cameras).

## Scoring

Read every control step (20 Hz); the floss and its rules every physics step (240 Hz). In a real trial the same
quantities come from the pressure film strip, the scale's display in a 240 fps phone video, and microscope photos of
the two incisors taken out before and after the run.

**0 for the rest of the run if:**
1. **Snapped onto the gum:** the floss marks the pressure film strip on the papilla. Real: a strip of Fujifilm Prescale
   MS (mono-sheet, 10-50 MPa, about 0.1 mm thick), cut 0.4 mm wide and 6 mm long, lies flat along the papilla's crest
   under the contact (not onto the tooth sides), held by a tab of 0.05 mm polyimide tape at each end on the gum,
   labial and lingual of the papilla, clear of the bands. A fresh strip each trial, checked unmarked under the
   microscope before it goes in; after the run any red mark on it is a snap (marks only along the cut edges, narrower
   than 0.1 mm, from cutting, are photographed before the trial and ignored).
   Sim: the floss's force on the same strip reaching 2 N (estimate) in any physics step.
2. **Pressed on the head or board:** at any time in the trial, the downward force on the head and the board it
   stands on (which also carries the pick at the start and the scale's display) is 3 N or more for 0.5 s. Sim: the
   floss's push on the teeth, gum and contact plus the pick's and the grippers' contact forces on the head, the lip,
   the board and the display, averaged over each 50 ms step, for 10 steps in a row. Real: the scale's tethered
   display, in the 240 fps video, reads 300 g or more above tare.
3. **Floss failure:** the floss pulls out of the pick or breaks. Sim: the floss's load (the net force it carries)
   passes 22 N (estimate).
4. **Head moved** 5 mm (real trial; the head is fixed in the sim).

**Passed below the contact:** any cell in rows 2-4 of either band is bare (the bands lie under the contact; the top
row is left out, as mounting or removing an incisor can scrape it). Real and sim alike.

**Wiped side:** 80% or more of the band's cells are bare. The band, framed by the four scribed marks, is 2 mm wide,
centred under the contact point, from the contact's lower edge (0.5 mm below the contact point) to 1 mm above the gum
line; it is read as a grid of 10 columns (0.2 mm each, between the two vertical ticks) by 4 rows (each a quarter of the
band's height at that column, between the two grooves), 40 cells. Real: each incisor is seated, facing side up, in its
own putty jig (polyvinyl siloxane moulded round a spare tooth of the same type, so every tooth sits the same way) under
a Dino-Lite AM4113T USB microscope on its stand at 50x (field 7.8 x 6.3 mm, 1280 x 1024 pixels), lit by its 8 built-in
LEDs at a fixed setting with the room lights off; a 0.1 mm calibration slide is photographed at the same setting for the
scale bar. The same jig and setting are used before and after the run. The 10 x 4 grid is laid over the after-photo on
the scribed marks, and a cell counts as bare when bare resin covers at least three quarters of it (compared with the
before-photo); a side is wiped at 32 of 40 cells. Sim: 4 points a cell (2 x 2 at its quarter points); a point is wiped
when the floss's surface passes within 0.1 mm of it while sliding over it at 1 mm/s or more and pressing that tooth with
0.5 N or more, and a cell when 3 of its 4 points are. The band can only be reached through the contact: it lies under
it, on the tooth's facing side.

**Otherwise, from the final 1 s** (the lowest tier any of its steps reached). Every nonzero tier needs the floss to
have passed below the contact:
- **1.0:** passed, both facing sides wiped, the floss back above the incisal edges;
- **0.75:** passed, both sides wiped, the floss still between the teeth;
- **0.5:** passed, exactly one side wiped;
- **0.25:** passed, no side wiped;
- **0:** never passed (including pressing on an incisal edge, or doing nothing).

Which tooth comes first, the number of strokes, the sawing amplitude and where the pick ends up don't matter. The
trial ends once 1.0 has held for 1 s.

## Real-world build

Positions in mm from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table.

| Item | Details |
|---|---|
| Training head | A fix-on-table dental training head (e.g. ANNWAH mouth simulator, about US$140): silicone face mask with soft lips over a metal head form, no tongue, a 28-tooth Kilgore/Nissin 200-type resin typodont with screw-in teeth in soft urethane gums (as listed for Candent D85SDP-200-GUB) in hinged jaws. Upright, facing the robot, centred on y 0; the jaw at its open stop (48 between the incisal edges, the hinge screw set so 5 N or more under the chin closes it); mouth opening 52 wide. |
| Target | The contact between the two lower central incisors (31, 41) at x 300, y 0, 140 up, 1.5 below their edges; the papilla 2.5 below it. |
| Lower lip | The mask's lower lip rests on the lower front teeth from their gum line up to 3 below their edges. |
| Board and scale | 300 x 300 x 18 plywood board (x 235 to 535, y -150 to 150), the head's clamp on its back edge, standing in a taped outline on a digital shipping scale (300 x 300 platform, 2 g steps) with a tethered display, which sits on the board's left front corner (x 245 to 290, y 90 to 140). |
| Floss pick | GUM Professional Clean Flossers, F-shaped (floss parallel to the handle between two arms) with a ribbed control-grip handle: 75 long, floss 15 between the tips of two 10 arms, handle 10 deep and 6 thick at the grip. Flat on the board's front right corner, handle along x, head away from the robot, arms to the robot's right, the handle's centre line at y -115, its rear end 30 over the board's front edge. A new pick every trial. |
| Coated incisors | Spare 31/41 pairs: on each facing side, four marks scribed before coating (a groove at the contact's lower edge, one 1 above the gum line, two vertical ticks 2 apart centred under the contact) frame the band; the facing sides are sprayed green with Pascal Occlude occlusal marking spray (shake 10 s, 10 cm away, two 0.5 s passes, dry 60 s) and photographed in their jigs under the microscope (see Scoring). |
| Pressure film | Fujifilm Prescale MS (mono-sheet, medium pressure, 10-50 MPa, about 0.1 thick), cut into strips 0.4 wide and 6 long with a scalpel against a steel rule; 0.05 mm polyimide (Kapton) tape cut into 2 x 3 tabs. One strip per trial, laid flat along the papilla's crest between 31 and 41 under the contact and held by a tab at each end on the gum, clear of the bands. |
| Microscope and jigs | A Dino-Lite AM4113T USB microscope on its stand at 50x, a 0.1 mm calibration slide, and one polyvinyl siloxane putty jig per incisor type (31 and 41), moulded round a spare tooth so each seats facing side up at the same pose under the lens. |
| Phone | On a clamp tripod at x 300, y 350, 140 up, filming the mouth and the scale's display at 240 fps. |
| Tools | Feeler blades 0.05 and 0.10 mm (contact check), digital caliper, 0-5 N and 0-10 N spring scales, scriber, 50 g weight. |

**Per trial:** swap in a pre-coated, photographed, contact-checked pair; check a fresh film strip is unmarked
under the microscope and tape it on the papilla;
re-tare the scale; put a new pick on its mark; start the phone. After the run, take the strip off and look for any red
mark (under the microscope at 50x if in doubt), record the final state from the video, take the pair out and
photograph each facing side in its jig under the microscope; count the band's bare cells against the pre-trial photo.

## Simulation fidelity

Rigid PhysX at 240 Hz (yam_arena's default), 20 Hz control, GPU dynamics.
- **Head:** one kinematic body: the 28 teeth, the gums with their papillae, the face mask with its 52 mm mouth opening,
  the mouth's dark interior and the post, all meshes built in `build_assets.py`. The two lower central incisors are a
  parametric crown at typodont size (5.4 mm wide at the contact, crown 9 mm, 2.5 mm thick labio-lingually at the
  contact, flat facing sides), each moved 0.3 mm away from the contact: a 0.6 mm gap. The papilla's crest is 2.5 mm
  below the contact. Triangle-mesh colliders on the front teeth, the lower gum and the mask. The gums are rigid; the
  head is fixed.
- **Floss:** a string in task code (`floss.py`): 31 nodes between the pick's two arm tips, 0.5 mm, frictionless,
  tension 5 N plus 4.7 N per mm of stretch, solved every physics step. It is straight where free and wraps whatever it
  touches: the two incisors, the papilla, and the contact. Its pull on the two arm tips is applied to the pick every
  physics step, so the robot feels it through the grasp.
- **The contact:** task code. A plug only the floss touches fills the gap over the contact area (1 mm tall). From
  above it holds while the floss's push is below 8 N and the floss does not slide along itself; past that, or while
  the floss slides along itself (sawing, its travel along itself over 0.1 s), it gives way at the speed where the push
  equals 8 N times the share of the floss's slip that is downward (friction opposes the total slip), so a
  back-and-forth lets the floss creep in under a light push (3 N sawing +-2 mm at 2 Hz). It lets go at the zone's
  bottom. From below it holds one way: the floss goes back up through it only when pulled up 3 N or more, sawing or
  not, so pressing on a tooth below the contact never lets it creep back in.
- **Lower lip:** an articulation: the lip on a slide along x, 0-10 mm forward, a 0.3 N/mm spring with 0.3 N preload
  against the teeth, damped, friction 0.6, 30 g; 40 mm long following the arch, 10 mm thick, its top edge rounded (6
  mm). It touches only the pick and the robot.
- **Pick:** one rigid body (a 10 x 6 mm grip and two arms tapering to 0.6 mm at their tips), 5 g. The gripper
  squeezes the grip with about 15 N a pad.
- **Pressure film:** the floss's force on a 0.4 x 6 mm strip along the papilla under the contact, every physics step,
  marked at the force given under Scoring. The strip's thickness is not modelled.
- **Board and display:** kinematic bodies on the scale; the pick's and the hands' contacts on them count in the press.
- **Wipe:** the band's 40 cells a side (4 points each) are wiped where the floss slides over them (within 0.1 mm, at
  1 mm/s or more) while pressing that tooth with 0.5 N or more; a cell when 3 of its 4 points are.
- **Display only:** the lower lip and the mouth's dark interior are drawn cut away (the lip still acts in physics), a
  small lamp lights the inside of the mouth, the floss is drawn as a 1 mm bright blue line, the band's green coating
  loses its dots where wiped, and the four scribed marks and the film strip (red once marked) are drawn. In recorded
  demo runs the top camera moves to a close mouth-level view once the floss is over the notch.
- **Not simulated:** the coating itself (the rule stands in for it), the gums' and lip's softness, the jaw's hinge.
- **Estimates:** every force and stiffness above, and the head's and pick's sizes.

## Reset between trials

Throw the pick away. Take off the face mask, lift the lower typodont jaw off the head's frame, unscrew the two lower
central incisors from below, and screw in the next spare pair, each screw to its stop with the same screwdriver. Refit
the jaw at its open stop, tape a fresh film strip (checked unmarked) on the papilla, refit the mask, re-tare the scale,
and put a new pick on its mark: about 8 minutes. Spare pairs are prepared ahead: marks scribed, contact checked (a
0.05 mm feeler blade enters with resistance, a 0.10 mm one does not), sprayed and photographed in their jigs. Re-spray
any pair showing bare resin in the band.

## Sources

- Fleming EB et al. (CDC/NCHS), J Periodontol 2018;89(8):933-939: daily flossing 31.6% of US adults; 32% reported
  no flossing in the past week.
- American Dental Association, flossing guidance (MouthHealthy): quotes above; floss picks listed as
  "pre-threaded flossers".
- NHS, "How to keep your teeth clean".
- Foundations for Assisting in Home Care (SUNY, CC BY 4.0), oral care: "Do not quickly snap the floss between the
  teeth or you can cause pain or injury."
- Carter-Hanson C et al., J Clin Periodontol 1996;23:873-8 ("as effective as finger flossing").
- Hallmon WW et al., "Flossing clefts", J Periodontol 1986;57(8):501-4.
- Nimmanon V et al., M Dent J 2019;39(3):223-230: seven disposable floss holders, floss 13.2-16.1 mm between the arms,
  failure under a mid-span load from 22.69 N.
- Sunstar GUM Professional Clean Flossers: F-shaped flossers with an "ergonomically shaped control grip handle" (a
  widened, ribbed finger rest).
- Fujifilm Prescale: nine grades from 0.006 to 300 MPa; MS (medium pressure, mono-sheet) 10-50 MPa (100-500 kg/cm2).
- Dino-Lite AM4113T specifications: 10-50x and 220x, 1280 x 1024 pixels, 8 white LEDs; field 7.8 x 6.3 mm at 50x.
- Candent, 28 Tooth Adult Typodont with GUB Urethane Gums (D85SDP-200-GUB): "Soft urethane gingivae with 28
  screw-retained teeth".
- Holley TJ et al., J Tenn Dent Assoc 2014;94(1):34-6: typodont teeth sprayed with Occlude indicator spray as a test
  coating for flossing.
