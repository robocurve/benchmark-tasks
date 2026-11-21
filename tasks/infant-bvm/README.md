# Give an infant CPR manikin two breaths with a bag-valve mask: mask sealed over nose and mouth, chest rising each time

A Laerdal Little Baby QCPR infant CPR manikin lies face up on a rubber mat, head toward the arms, its head in the
neutral (sniffing) position; a white card flag with a black dot stands on its chest. To its left an Ambu SPUR II Infant
bag-valve mask (BVM) stands mask rim down on the mat, its bag pointing toward the robot. The instruction is:
**"Give the infant manikin two rescue breaths with the bag-valve mask: keep its head in the sniffing position, hold
the mask sealed over its nose and mouth, squeeze the bag just until the chest rises each time, then lift the mask off
its face and keep holding it."**

To do it, the robot has to:
- pick up the 110 g BVM and land its 55 x 47 mm anatomical mask on the face, the narrow end on the bridge of the nose
  below the eyes and the wide end on the chin above its point, centred within 6 mm, every part of its air cushion
  pressed on;
- keep it pressed on evenly (about 10 N) without flexing or over-extending the head out of the sniffing position;
- with the other hand, squeeze the bag, which sticks out 12 cm from the mask's centre, about 25 mm over about 1 s
  without pushing it up or down (a push there tips the mask, a side of the cushion lifts and the breath leaks), so the
  chest's flag dot rises 5-12 mm, then let go and let the chest fall;
- do it exactly twice, touching the manikin with nothing but the mask and the hand holding it;
- lift the mask 30 mm or more off the face and hold the BVM still.

## Why it matters

- Infant CPR is compressions and breaths: ILCOR's 2025 paediatric consensus notes that "most cardiac arrests in infants
  and children have a respiratory etiology" and that "both ventilation and chest compressions are important components
  of CPR in children". The same bag-mask breaths serve newborns: Wall et al. 2009 count "the estimated 6 million
  newborns each year who require basic neonatal resuscitation", which is "basic neonatal resuscitation, including
  bag-and-mask ventilation", and "The key equipment for neonatal resuscitation is a self-inflating bag-and-mask".
- The AHA BLS infant skills test checks exactly this: "Gives 2 breaths with a bag-mask device: Each breath given over
  1 second; Visible chest rise with each breath".
- It is hard to do well. ILCOR 2025 (neonatal): "Positive-pressure ventilation administered via a face mask is commonly
  ineffective because of mask leak or failure to achieve airway patency." On manikins the mean leak was 55% (Wood et
  al. 2008). NRP: the mask "should rest on the chin and cover the mouth and nose, but not the eyes"; "The airway will be
  obstructed if the neck is excessively flexed or extended"; "excessively high lung volumes and airway pressures can
  cause lung injury"; too much pressure on the mask can "inadvertently flex the baby's neck".
- A robot giving these breaths frees a lone rescuer for compressions.
- The manikin enforces the airway itself: Laerdal's Little Baby QCPR "Airway closes when neck is hyperextended or
  underextended", shows "Visible chest rise on ventilations" and counts them ("QCPR sensors ... ventilations (rate and
  count)").

The task tests sealing a soft-rimmed tool on curved face landmarks (narrow end on the bridge of the nose clear of the
eyes, wide end on the chin, every part of the cushion pressed on) and keeping it sealed while the other hand squeezes a
bag coupled to it within a volume and timing window, with nothing else touching the infant.

## Files

| File | What it is |
|---|---|
| `task.py` | the yam_arena task file: objects, contact reading, the neck's and the swivel's friction, the monitor and scoring |
| `bvm_rules.py` | the air rule (bag squeeze to volume, lungs, chest rise), the breath detector and the score tiers |
| `geometry.py` | every size and the layout, the rule constants, the BVM's resting start pose |
| `assets/` | `manikin.usda`, `bvm.usda` and `landmarks.npz`: everything the task loads |
| `build_manikin.py` | build tool: the manikin's body from the CC0 MakeHuman base mesh and its CC0 baby targets (download commands inside; it downloads into `build/` and writes `build/manikin.npz` and `assets/landmarks.npz`) |
| `build_assets.py` | build tool: writes `assets/manikin.usda` and `assets/bvm.usda` (needs `pxr`, not the simulator) |
| `scenekit.py` | USD bodies and articulations for task files, hooks for per-step task code |
| `demokit.py`, `demo.py` | the scripted demo and its variants |
| `variants.py` | runs several demo variants one after another in one process (`demo.py --variants`) |

## Run a trial

```
source <ARENA_DIR>/yam.env
# assets/ is shipped; to rebuild it: python build_manikin.py && python build_assets.py
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    --scorer task_score --max-steps 2400 --policy ...
```

- Scene `station`, 20 Hz control, physics 240 Hz (yam_arena's default). The run limit is 2,400 steps (120 s).
- No object jitter and no seeded randomness: the scene is the same in every trial, so the seed changes nothing.
- `--scorer task_score` gives 1.0 on success (the 1.0 state held for 1 s ends the trial); otherwise the score at the
  end of the run (`SCORE_OVER = "end"`).
- Offline checks of the air rule and the breath detector: `python bvm_rules.py` (no simulator).

## Dream-1 run

Dream-1 (dreamlabs/dream-1-beta), with the exact instruction, `object_jitter=0`, 20 Hz and the 2,400-step limit, ran for
1,040 steps. The Dream API then refused its next inference request with "PolicyError: DreamAPI refused the inference
request: DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: The instruction acts on a
baby-like figure." inspect-robots marks such a run as errored and records no score, so its 1,040 recorded actions were
replayed through `task.py`: score 0 throughout. In those 52 s its left arm reached toward the bag but never lifted it,
and its right arm moved over the manikin's head; no breath was given. `model_run.mp4` shows the 1,040 steps (three
cameras). One run with the task's exact instruction, not retried after the refusal.

## Scripted demo

```
python demo.py task.py demo_out [--no-record] [--keep-top-cam] [--variant offcentre|chest|big|hyper|drop|one]
python demo.py task.py out_dir --variants full,one,big,...     # several, one after another, no video
```

The demo reads object poses (and the cushion's and the bag walls' joint positions). In a recorded run the top camera
moves to a close view of the face and the chest flag when the mask goes onto the face (`--keep-top-cam` keeps it
where it is); only the video changes.

1. **Take the bag (left arm).** From above, tilted 20 degrees away from the robot, pads on the bag's two side walls
   10 mm from its middle toward the valve (8 mm squeeze commanded, about 2 mL: well under any breath); lift 70 mm.
2. **Carry the mask over the face.** In joint space to 35 mm over the face's mask point, the mask turned to the face
   (narrow end toward the crown), then a few corrections on the mask's measured pose.
3. **Pinch the stem (right arm).** From the manikin's right, fingers pitched 20 degrees down, pads across the 22 mm
   stem and the swivel's patient port just above it (6 mm squeeze commanded). Holding both stops the swivel, so the bag
   hand's grip keeps the mask turned as the face. Any hold that keeps the mask sealed counts.
4. **Seat and press.** Both hands lower the mask onto the face. The right presses it on (a loop on the cushion's mean
   compression, 2.2 mm), trims its tilt (a plane fit to the 12 segments' compressions) and keeps its centre on the
   mask point; once all 12 segments are in 1.1 mm or more and the centre is within 3 mm for 5 steps, the trims stop.
5. **Two breaths (left arm).** Squeeze 25 mm over 1.3 s (a loop on the walls' travel), hold 0.3 s, let go over 0.6 s;
   wait until the chest is back within 0.4 mm, then 0.4 s; again.
6. **Lift off.** The right lets go of the stem and backs out; the left lifts the BVM off the face and holds it still.

Result (the recorded run in `demo.mp4`): **1.0**, success at step 833 (41.7 s of the 2,400-step limit), no
zero condition.

| Margin | Breath 1 | Breath 2 | Rule |
|---|---|---|---|
| bag squeeze, volume | 25.0 mm, 30.0 mL | 25.0 mm, 30.0 mL | breath 20.2-31.9 mm (5-12 mm of rise) |
| chest (flag dot) peak | 6.70 mm | 6.76 mm | 5 to 12 mm |
| onset to peak | 1.00 s | 1.00 s | 0.5 to 2 s |
| least-pressed cushion segment | 1.12 mm | 1.21 mm | 1.0 mm or more, all 12 |
| mask centre off the mask point | 1.08 mm | 1.11 mm | 6 mm or less |
| head angle (0 = eyes up) | -0.63 to -0.57 deg | -0.68 to -0.63 deg | 5 deg flexed to 15 deg back |
| clean contact | yes | yes | nothing but the mask and its hand on the manikin |

Held off: the mask 130 mm off the face, the BVM in the left hand and still, from step 795; the trial ended at 833.
The manikin moved 1.0 mm (limit 20) and its back lifted 0.0 mm (limit 5).

Variants (`--variants`, one Isaac process, no video; each from a fresh reset, 1,500 steps, `hyper` 1,100):

| Variant | What it does | Score (expected) | Evidence |
|---|---|---|---|
| `one` | one 25 mm squeeze, then lifts off | 0.5 (0.5) | one breath, 6.70 mm, 1.00 s; held off |
| `big` | second squeeze 40 mm | 0.5 (0.5) | breath 1 6.85 mm; squeeze 2 is 76.7 mL, the lungs stop at 60 mL: 15.0 mm, too big |
| `drop` | after two breaths carries the BVM 250 mm aside, sets it down beside the manikin and lets go | 0.75 (0.75) | breaths 6.83 and 6.84 mm; never held still 1 s; no zero |
| `chest` | after 3 s seated the mask hand lets go of the stem and pinches the chest beside the flag during both squeezes | 0.25 (0.25) | fingertips on the torso (not clean); the mask, held only by the bag, sits with 0 of 12 segments in 1 mm, so both 25 mm squeezes (28.6 mL) put no air in |
| `hyper` | the head set 25 deg back on its neck at the start (the neck's friction holds it), then the demo up to the squeezes, the mask kept on | 0 (0.25) | the airway rule holds: seated at 24.4 deg back, 12 of 12 in, both 25 mm squeezes (29.6 mL) give 0.00 mm of rise; but during the first squeeze the manikin is pushed 24 mm along the mat and its back lifts 5.6 mm (zero): the demo is not adapted to a head this far back |
| `offcentre` | the mask aimed 10 mm toward the chin | 0 (0.25 or 0) | the cushion's chin end slides past the chin: the mask tips 26 deg, 4 of 12 segments on the face, centre 19 mm off; the bag slips out of the bag hand, whose squeezes close on nothing; when the mask hand lets go at the end the BVM falls on the manikin (zero), which has also moved 20 mm |

Media: `demo.mp4` (the top camera large, the left and right arm cameras stacked beside it; from step 438, when the
mask goes onto the face, the top camera is moved to a close view of the face and the chest flag, labelled "Close view",
so the seated mask and the flag's rise show); `setup.png` is the first top-camera frame; `closeup.png` is the close view
at the first breath's peak (step 585: the mask pressed on, the flag's dot 6.7 mm up).

## Scoring

Read every 50 ms control step from the poses, joints and contacts of the manikin (torso, head, chest), the BVM (mask
dome, 12 cushion segments, valve and bag) and both arms. Real-trial tools: the phone's side video of the chest flag,
the station videos, the tape marks and the Laerdal QCPR Learner app on the phone (a new session is started for each
trial and its ventilation count is read off the phone when the trial ends).

| Rule | Value |
|---|---|
| Seated | The mask covers the nose and mouth, its narrow end on the bridge of the nose below the eyes and its wide end on the chin above its point. Sim: the mask's centre (the middle of its cushion's contact line) within 6 mm of the face's mask point (midway between the nasion and the chin point, on the face); all 12 cushion segments pressed on the face (each pushed in 1 mm or more and touching the head); the cushion's outer edge off both eye zones (9 mm round each eye's centre). Real trial: the station videos. |
| Clean | Nothing touches the manikin (torso, head, chest) except the mask (dome and cushion) and the fingers of the hand that holds the mask, on the head. The bag, the valve, the other hand and every arm link count. Sim: PhysX contact forces, 0.02 N or more. Real trial: the station and phone videos. |
| Air (sim) | `bvm_rules.py`: the bag's squeeze becomes volume, pi/4 (d_lr^2 + d_tb^2) x 60 mm from the walls' travel (29.5 mL at 25 mm, at most 150 mL); a step's increase reaches the lungs only while the mask is seated and the head is between 5 degrees flexed and 15 degrees back (the airway window; 0 = eyes straight up); the lungs hold at most 60 mL, hold their volume while a squeeze is held and empty with a 0.5 s time constant once it is let off or the mask is not seated; the chest (flag) rises 0.25 mm per mL. Nothing else moves the chest. |
| Breath | While seated and clean, the flag's dot rises 5 mm or more above rest, peaks no more than 12 mm up, 0.5 s to 2 s after it started to rise (left 0.5 mm), and falls back within 2 mm of rest. Seated and clean must hold from the start of the rise to the fall. Real trial: the dot's rise in the phone video, scaled by its own 10 mm diameter, from its height in the first second; the app counts a ventilation for it. |
| Too big | Any seated, clean rise that peaks more than 12 mm up, whatever its timing (about 8 mL/kg for the 6 kg infant the manikin stands for). |
| Held off | The mask 30 mm or more from the face, the BVM in a gripper (both pads on it) and still (under 2 mm per step), for 1 s. |
| Zero for the run | The manikin moves 20 mm from its outline or its back lifts 5 mm off the mat; the mask's rim presses on an eye for 1 s or more; the BVM falls 30 mm or more, ungripped, onto the manikin. |
| Tiers (end of run) | 1.0: exactly two breaths, none too big, then held off. 0.75: exactly two breaths, none too big, but the mask left on the face, not held still, or dropped or put down. 0.5: one breath; or three or more; or any breath with a too-big one. 0.25: no breath, but the mask seated for 2 s or more (continuously). 0: anything else. |
| Success | 1.0 held for 1 s (`SUCCESS_HOLD_S`) ends the trial, in a real trial too. |

Which hand holds the mask doesn't matter.

How the shortcuts fail: lifting or pinching the chest moves nothing (the chest moves only with the air rule) and voids any
breath in progress; resting the bag on the manikin voids the breath; squeezing with the mask in the air or off-centre
puts no air in; flexing or over-extending the head closes the airway; one big fast squeeze is too big or peaks too
early; extra breaths cap the score at 0.5.

## Real-world build

Millimetres from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table.

| Item | Details |
|---|---|
| Mat | a 700 x 500 x 3 rubber non-slip mat, taped down, centred at x 500, y 0 |
| Manikin | Laerdal Little Baby QCPR, light skin (Laerdal 133-01050; Bound Tree 3611-13301, US$323.99): 650 x 260 mm, 1.8 kg, "designed to represent a three-month-old infant"; "Visible chest rise on ventilations"; "Airway closes when neck is hyperextended or underextended"; "QCPR sensors ... ventilations (rate and count)" over Bluetooth to the Laerdal QCPR Learner app on a phone. Its airway (lungs) fitted as the maker shows. Face up on the mat, centred on y 0, head toward the robot, crown at x 180, heels at x 830; head in its neutral position, eyes looking straight up. Its outline is marked on the mat with tape. |
| Chest flag | a stiff white card, 30 tall and 20 wide, taped upright with removable tape on the middle of the chest at the nipple line (x 400, y 0), its face toward +y, a 10 mm black dot centred 20 above the skin |
| Phone | on a tripod beside the table at x 400, y 700, level with the dot, filming the flag face-on and the manikin's left side |
| Bag-valve mask | Ambu SPUR II Infant (Bound Tree 2442-21201, US$24.99): thin-walled bag 190 x 71 mm, frosted and slightly translucent (silicone, estimate), 70 g, 40 cmH2O pressure-limiting valve, patient valve on a swivel; its oxygen reservoir taken off; its Ambu infant anatomical face mask (clear PVC dome, air-filled PVC cushion, 14 g). Mask rim down on the mat at the manikin's left, mask centre at x 300, y 190, narrow (nose) end toward the robot, tipped toward the bag: the bag points toward the robot along y 190 and its far end rests on the table at x 80. The mask's spot is marked with tape. |

Cost: about US$380 (manikin, BVM, mat, card, tripod, wipes); a 24-pack of replacement airways is US$34.99.

**Real trial:**
- *Before each trial:* stand the BVM on its tape mark; head neutral, manikin on its outline; flag upright; start a
  new session in the QCPR Learner app; film the flag for 1 s before the start (its resting height).
- *Seated, clean, held off, drops, eyes:* from the station videos and the phone's side video.
- *Breaths:* the dot's height in the phone video, in units of its own 10 mm diameter, against its resting height; the
  time from the start of a rise to its peak from the video's frame count. The app's ventilation count, read off the
  phone at the end, is a cross-check: if it and the video disagree, the video decides.
- *End:* the trial is stopped once 1.0 has held for 1 s (two breaths, then the BVM held still off the face).
- *Manikin moved or back lifted:* its tape outline and the side video.

## Simulation fidelity

Rigid PhysX at 240 Hz (yam_arena's default), 20 Hz control, GPU dynamics. Every unpublished size, mass, stiffness and
friction below is an estimate.

- **Manikin body:** the CC0 MakeHuman base mesh with MakeHuman's own CC0 baby targets (the six ethnicity and sex
  targets mixed equally), the arms lowered to the sides and the feet pointed, scaled 1.08 to the Little Baby's 650 mm
  length (`build_manikin.py`). It is a floating articulation: the torso (1.55 kg, 19 convex pieces) carries the head
  (0.25 kg) on a hinge about y. The hinge is the centre of the occiput's curve (66 mm over the mat); a sphere about it
  is the occiput's collider and rests on the mat, so a press on the face goes into the mat. The neck's friction (0.5 N
  m) is a stick-slip held by `task.py` (a stiff spring to a rest angle that is dragged along past the breakaway); the
  hinge stops at 20 degrees flexed and 30 back. The face is the head's mesh as a signed-distance-field collider, rigid:
  the cushion conforms, the face does not.
- **Face positions** (the sim's, estimates): nasion x 260, 136 up; chin point x 317, 131 up; eyes x 260, y +-20;
  the mask point midway, x 289, 134 up.
- **Chest:** a patch of the chest's skin with the flag on it rides a slide that only `bvm_rules.py` moves (the air
  rule); it has no collider, so touching the chest touches the torso.
- **Mask:** the dome (2 g) is the BVM's root body; its collider is the cap above the nose and the 22 mm stem. The air
  cushion is 12 capsules (1 g each, 3.5 mm radius) along its contact line, 5 mm inside the 55 x 47 mm outline, each on a
  slide along the mask's axis: 4 mm of travel, 0.4 N/mm, 0.1 N preload. Their rest heights follow the face under the
  rim (the anatomical shape): centred on the face's mask point and pressed in 2 mm each, the cushion carries about
  11 N.
- **Valve and bag:** the swivel (a revolute about the mask's axis, 0.02 N m of friction held by `task.py`) carries
  the 22 mm patient port, the valve housing with its expiratory port and the bag's rigid core (56 g together), the
  valve's axis 72 mm over the mask's plane. The bag's middle 100 mm is four wall shells (10 g each) on radial slides:
  30 mm of travel, 0.8 N/mm, 0.5 N preload; squeezed 25 mm across, each pad pushes about 10.5 N. Each wall's collider
  is a 24 x 15 mm block under its crown (a pad flattens a soft bag's wall; round shells let a pad roll off); the bag's
  underside is flattened 3 mm where it rests. The bag is drawn as smooth, slightly translucent surfaces over its
  colliders. Total 110 g. The BVM's parts never collide with each other.
- **No air:** `bvm_rules.py` (see Scoring, "Air"). The seal is all-or-nothing: the breath's air enters only with every
  cushion segment pressed on the face and the mask seated; a partly sealed mask passes none.
- **Friction:** mat 0.5, manikin skin 0.8, BVM 0.9 (averaged per contact).

Gaps: the face is rigid; the leak is all-or-nothing; the squeeze's resistance does not grow with lung pressure; the
airway window's angles stand in for the manikin's own airway.

## Reset between trials

Lift the bag-valve mask off, let the bag refill, and stand it back on its tape mark, mask rim down, bag toward the
robot. Return the manikin's head to neutral, eyes straight up, and the manikin to its tape outline; check the flag is
upright on its tape and start a new session in the QCPR Learner app. Wipe the mask's cushion and the face with an
alcohol wipe and let them dry. About 1 minute. The airway is replaced after each day's trials, as Laerdal advises after
every class.

## Sources

- Laerdal Little Baby QCPR item page (133-01050) and Important Product Information (size, weight, three-month-old,
  airway, chest rise, QCPR ventilation feedback, airway replacement); Bound Tree listing 3611-13301 (price, replacement
  airways 3616-10150).
- Ambu SPUR II datasheet (LBL-006982) and instructions for use (infant: 190 x 71 mm, 70 g, 40 cmH2O, swivel); Ambu
  Disposable Face Mask datasheet (anatomical shape, air-filled cuff, infant 14 g); Bound Tree 2442-21201.
- ILCOR 2025 Pediatric Life Support and Neonatal Life Support CoSTRs; Wall et al. 2009, Int J Gynaecol Obstet
  (PMC2875104); AHA BLS Infant CPR Skills Testing Checklist (2025); NRP 8th edition, Lesson 4; Wood et al. 2008 (mask
  leak on manikins); Schilleman et al. 2010 (two-point top hold); O'Shea et al. 2016 (mask fit).
- MakeHuman base mesh hm08 and macro baby targets, released CC0 (makehumancommunity/makehuman on GitHub).
