# Load a slide into a microscope's stage by its edges: coverslip up, seated under the objective, spring arm lowered gently

An AmScope M620 compound microscope (or the B120C, the same stand with a binocular head) stands on the table, its head
toward the arms, powered off. Its stage is fully lowered with the 4x objective in the path, its X and Y scales at the
readings that centre a seated slide under the objective, and its spring caliper closed. To the robot's left, an angled
bench slide holder holds one prepared slide in its middle slot, label end up. The instruction is:
**"Load the prepared slide into the microscope's slide holder, coverslip up and seated against the holder's back edge
and jaw, so the specimen is under the objective. Handle the slide only by its edges and label end."**

To do it, the robot has to:
- take a 75 x 25 x 1 mm glass slide out of the holder touching only its 1 mm edges (or its frosted label end), never the
  coverslip or the faces of the specimen area, and turn it from standing to flat, coverslip up, without losing it;
- lay it flat on the stage in front of the seat (the objective and nosepiece hang over the seat itself);
- open the stage's spring caliper by its lever with one hand and keep it open;
- push the slide back by its 1 mm front edge, under the objective, until its back edge is flush with the template's back
  edge (within 0.3 mm), without shoving the stage itself;
- let the caliper's arm back down under control, the hand still on the lever when its tip meets the slide, so that it
  presses the slide's left end onto the fixed jaw (within 0.3 mm); then let go of everything.

## Changes in this version

- A cheaper microscope is named: the AmScope M620 (40X-1000X; monocular, about US$219, new from AmScope). It is built on the same
  stand as the AmScope B120C (binocular, about US$300) named before: the same two-layer mechanical stage, metal
  template, spring caliper and lever, nosepiece and objectives, so the stage height and the space under the 4x
  objective are the same. Only the head differs; its single eyepiece tube sits between the B120C's two, further from
  the arms' paths. Either microscope can be used; the scene models the B120C.
- The scene, the scoring, the demo and the scores are unchanged.

## Why it matters

Every look at a slide in research, teaching and clinical labs starts with loading it into the stage, and the step is
written into every maker's manual:
- AmScope 120-R series manual: "A prepared slide can be placed on the stage by pushing the caliper lever outward to open
  the caliper, then sliding the slide in place so the long edge is flush against the back edge of the template. Gently
  releasing the caliper will secure the slide in place." and "The stage should be set to a low position when mounting
  the specimen."
- Olympus CX23 manual: "Press the holding lever knob b backward (arrow direction) to open the specimen holding lever c,
  and place the specimen slide d by sliding it on the stage from front toward back." and "If you return the holding
  lever c with excess force or release your hand from the holding lever knob b in the middle, the lever may collide
  with the specimen slide to damage it."
- University of Ottawa teaching lab (Olympus CX41): "Position the microscope slide against the left and back sides of
  the specimen holder and gently release the right arm so that it is against the right front corner of the microscope
  slide."
- Nikon E100 manual: "Place specimen slide on the stage with the coverglass facing upward."
- Carolina Biological: "Slides should be held by the edges, avoiding the cover glass area."

The task tests handling a thin, fragile, rigid plate by its edges alone (a pinch across its 25 mm width on two 1 mm
edges, kept through a turn from standing to flat), edge-pushing it along a surface under an overhanging obstruction into
a two-face corner it cannot be seen entering, while the other hand holds a sprung arm open, and a controlled return of
that arm onto glass. Unlike laying a part into an opened clamp from above, the slide cannot be lowered into its seat:
the objective is over it.

## Files

| File | What it is |
|---|---|
| `task.py` | the yam_arena task file: objects, physics rate, the stage's stick-slip friction, contact reading and scoring |
| `geometry.py` | every size and the layout (estimates marked "est.") and the scoring constants |
| `build_assets.py` | writes `assets/*.usda` from `geometry.py` (needs `pxr`, not the simulator) |
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

## Dream-1 run

Dream-1 (dreamlabs/dream-1-beta) scored **0** with the exact instruction, `object_jitter=0`, 20 Hz, 320 x 176 cameras and
the 3,600-step limit (one run, not retried). Its left arm reached over the slide holder but never took a slide out of
its slot; its right arm moved about beside the microscope's stage. No slide reached the stage, and no zero condition
fired. 1,920 of its 3,600 action rows were beyond the arm's limits and were clipped. 75 inference requests, mean
latency 0.78 s (median 0.53 s), 6 API sessions (5 requests needed a second attempt); 3,600 steps took 10 min 45 s.
Video: `model_run.mp4`.

## Scripted demo

```
python demo.py task.py demo_out [--no-record] [--variant face|snap|short]
```

1. **Pick (left arm).** The hand comes down along the standing slide, pads across its 25 mm width on the two long
   1 mm edges near the label end (fingers commanded 3.2 mm inside the slide's width: about 7 N per finger, under the
   task's 10 N finger limit), and lifts it out along the slot.
2. **Turn and set down.** Carrying it round to the left of the stage, the wrist turns the slide flat, coverslip up,
   label end left; the hand comes in level from the left, fingers pointing right and tipped 21 degrees down so the
   pads' lower edges (the fingertips taper) lie flat on the stage, and lays the slide down 35 mm in front of the seat,
   3 mm right of the jaw line. It opens and slides the fingers out sideways.
3. **Open the caliper (right arm).** The hand takes the lever knob loosely between its pads (12 mm apart round the
   8 mm knob) and swings the arm 30 degrees open, the trailing pad pushing the knob, and holds it.
4. **Push (left arm).** Fingers closed and pointing back, hand pitched 18 degrees down so the fingertips are the
   fingers' lowest point, 0.08 mm over the stage (held there in closed loop on the hand's measured pose); their end is
   flat across the closed fingers' height, so they meet the slide's 1 mm front edge, not its top face. A part pushed
   at one point turns away, so the push goes in short strokes: at the middle of the front edge while the slide is
   square (within 0.6 degrees), at 14 mm toward its lagging end when it is not; between strokes the tip backs off and
   lifts before moving sideways (the pads' grip would drag the slide). 8 mm/s, 2 mm/s over the last 1.5 mm; once one
   end of the back edge meets the template, strokes at the other end turn the slide flush. It backs off.
5. **Lower the arm (right arm).** The arm returns at 7 mm/s at its tip, the knob riding on the retreating pad; the pad
   is still on the knob when the tip meets the slide's right end. The hand goes on the same way at the same speed: the
   spring carries the slide the last few mm onto the jaw and the arm comes to rest, then the pad leaves the knob as the
   load comes off it; the hand lifts straight up off the knob.
6. Both arms move clear of the microscope and go home.

Results (final code):

| Run | What the script does | Score |
|---|---|---|
| demo (recorded) | as above | **1.0**, success at step 1,204 (60 s of the 3,600-step limit), no zero condition: final landing 5.5 mm/s with the pad on the knob when the tip met the slide, back edge 0.03 / 0.02 mm from the template, left end 0.0 mm from the jaw, stage moved 0.0 mm, grippers 67 mm clear |
| check: face pinch | takes the slide across its faces 27-43 mm below the label end (pads on the specimen area, closed) | 0 (rule 1 at step 130: contacts with normals along the face normal) |
| check: lever let go | after the push, the right hand rises off the open lever at once | 0.75 (landing 118 mm/s, no gripper on the lever: hard-landing flag; seated 0.0 mm) |
| check: push 3 mm short | stops the push 3 mm before the template | 0.5 (held by the arm, landing 4.8 mm/s with the pad on the knob; back edge 5.5 / 1.1 mm off, jaw 1.5 / 0.0 mm: not seated) |
| check: brush, then let go | brushes the open arm with the slide during the push, then lets the lever go | 0.75 (the brush is not taken as the landing; final landing 142 mm/s, no gripper on the caliper: hard-landing flag) |

The four checks run with the final code (no recording).

Media: `demo.mp4` (the top camera large, the left and right arm cameras stacked beside it); `setup.png` is the first top-camera frame.

## Scoring

Read every control step (20 Hz). **The run scores 0 (latched)** if:
1. **Face contact:** a gripper (either hand's body or fingers) touches the coverslip, or touches either face of the slide
   outside its frosted label end: a contact carrying force whose normal is within 45 degrees of the slide's face normal
   and whose point lies on the face more than 0.3 mm inside the slide's outline, or a contact point on the coverslip.
   Contacts on the 1 mm edges have in-plane normals; a contact on an edge's arris (within 0.3 mm of the outline, where a
   rounded fingertip pushing the edge meets it) is an edge contact, while a pinch across the faces presses on their
   interior and is caught. (The slide is one body of three boxes, label end, specimen part and coverslip; contacts are
   read from PhysX with their points and normals.)
2. A gripper or the slide touches an objective or the nosepiece.
3. **Drop:** the slide falls freely more than 10 mm (its height more than 10 mm below its highest point since a gripper
   last touched it), or comes to rest (0.5 s, no gripper on it) anywhere but the bench holder or the stage.
4. **Stage moved:** the stage's X or Y carriage moves more than 1 mm from its start.

**Hard-landing flag** (latched, caps the score at 0.75): the landing is the arm coming to rest on the slide with no
gripper on it (the final landing, after the hand has let go); a touch while a hand holds the arm, open or not, is not a
landing. Each landing is measured over its final approach, the tip's last continuous motion before it stopped: the
flag is set if the tip's mean speed over the last 5 mm of that motion (or all of it if shorter; path over time, from
the per-step tip positions) exceeds V_max (20 mm/s), or if no gripper was touching the caliper (arm or lever) when the
tip met the slide on that approach.

Otherwise, from the final 1 s (every step of it must reach the rung), each level with both grippers 3 cm or more from
the slide, the caliper's arm and its lever:
- **flat:** all four bottom corners within 0.3 mm of the stage plate, coverslip up, slide still;
- **held:** the caliper's tip touches the slide and no gripper touches the caliper;
- **seated:** both ends of the slide's back long edge within 0.3 mm of the template's back edge and both corners of its
  left end within 0.3 mm of the fixed jaw; **under the objective:** the coverslip's centre within 8 mm of the optical
  axis;
- **0.25:** flat, not held; **0.5:** flat and held, not seated (or not under the objective); **0.75:** flat, held,
  seated and under the objective, with the hard-landing flag; **1.0:** the same with a gentle landing. The trial ends
  once 1.0 has held for 1 s;
- **0:** every other end state: coverslip down, riding on the template or under the arm's tip (not flat), still held,
  nothing done.

## Real-world build

Positions in mm from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table.

| Item | Details |
|---|---|
| Table | the station's wooden bench top |
| Microscope | AmScope M620 (40X-1000X) monocular compound microscope (about US$219), or the AmScope B120C binocular (about US$300), the one modelled. Both are built on the same stand: two-layer mechanical stage 116 x 125 mm with 70 x 25 mm X-Y travel as modelled (AmScope's current pages list 127 x 118 mm and 70 x 21 mm), metal template with a spring caliper and lever, forward-facing quadruple nosepiece (4x/10x/40x/100x, 45 mm parfocal), 25 mm stage stroke; only the head differs (the M620's single eyepiece tube sits between the B120C's two). Powered off, head toward the robot, optical axis at x 360, y 0 (base front edge at about x 285). Fixed to the table by two 25 x 100 mm strips of double-sided foam mounting tape under its base (3M Scotch-Mount or similar), or a padded F-clamp on the base's rear lip clear of the stage and the arms' paths. Stage fully lowered, 4x in the path, X and Y scales at the recorded readings that put a seated slide's centre on the optical axis, caliper closed. |
| Caliper | a curved arm swinging in the stage plane about a vertical pivot at the template's right back; its tip presses the slide's right end toward the fixed jaw; its lever knob sits at the stage's right edge. |
| Slide | a standard prepared slide made to this recipe: a ground-edge colour-frosted glass slide (Fisherbrand ground-edge frosted, 75 x 25 x 1 mm, about 5 g, frosted end about 20 mm) with a 22 mm square cover glass (Carolina 633095, 0.13-0.17 mm) mounted with one drop of permanent mountant over a dried 5 uL drop of 1% methylene blue at the slide's centre, the cover glass centred within 1 mm using a printed paper template under the slide; cured overnight. Make ten at once and use any of them. |
| Bench slide holder | Heathrow Scientific HS15981C (white polyethylene, 15 angled slots, 152 x 102 x 20 mm), centred at x 300, y 220, its long side along y so the slots run along x (front to back). The slide stands alone in the middle slot, label end up, its coverslip facing the microscope (-y); the neighbouring slots are empty. |

**Measured on the unit before the trials** (estimates in the sim, `geometry.py`; the measured values go into this
table): stage-top height with the stage fully lowered (sim 190 mm); the clearance under the 4x objective (sim 44 mm);
the template's back edge and fixed jaw; the caliper's pivot, lever and tip; the depth of stage in front of the seat (sim
50 mm); the slot's angle and depth in the bench holder (sim 20 degrees from vertical, 15 mm);
- **the caliper's tip force** (sim 1.5 N on a seated slide): the tip pushed against a kitchen scale;
- **the stage's breakaway force** (sim 5 N on each axis): a kitchen scale pressed against the slide holder, read at the
  moment the stage moves;
- **the caliper's free-closing speed** (sim about 400 mm/s at the tip): drop the lever from fully open onto a seated
  slide and film it at 240 fps (phone slow motion) beside a 1 mm tick strip; the mean tip speed over the last 5 mm
  before contact is 5 mm divided by the elapsed time; repeat 5 times and take the median. V_max is a fifth of that
  median and no more than 20 mm/s (20 mm/s in the sim). If a dropped lever closes slower than 50 mm/s, the gentle-landing
  rule is dropped. The sim's spring damping is set to match the measured speed.

**Real trial:**
- *Before each trial:* lower the stage fully and turn the 4x objective into the path; set the X and Y scales to their
  recorded readings; close the caliper; stand the slide in the holder's middle slot, label end up, coverslip toward the
  microscope.
- *During the trial:* an overhead phone films the stage at 60 fps; a second phone at stage height films from the right
  side; a strip of masking tape on the stage along the caliper tip's path carries 1 mm ticks; check that the
  microscope's base is still on its tape marks.
- *Face contact:* while the slide is held, the robot's logged gripper opening classifies the grasp: 20 to 30 mm is a
  pinch on the long edges (allowed); under 5 mm is a pinch across the faces, allowed only if the top video shows both
  pads wholly on the frosted label end. During the push and the set-down, a pad or fingertip visibly resting on the top
  face outside the label end, or on the coverslip, in the side video scores 0.
- *From either video (each is a 0):* contact with the coverslip or a face outside the label end, a touch on an
  objective, a drop.
- *After the trial:* inspect the slide and coverslip under the bench light; any crack or chip scores 0.
- *Hard landing:* in the overhead 60 fps video, take the arm's final landing (it comes to rest on the slide and the
  hand has let go; a touch while the hand holds the arm open does not count) and count the frames its tip takes over
  its last 5 mm on the tick strip before it stops; the mean speed is 5 mm x 60 / frames and must not exceed V_max (at
  20 mm/s, 15 frames or more). The gripper must still be on the caliper (lever or arm) when the tip meets the slide on that approach.
- *Stage moved:* the X or Y scale reading differs from the recorded one by more than 1 mm.
- *Seated and flat:* a 0.3 mm feeler blade does not enter between the slide and the template's back edge at either end,
  nor between the slide and the jaw, nor under any corner. *Held:* the arm's tip rests on the slide (photo). Under the
  objective is not checked separately: the coverslip is centred and the stage starts at its centred readings.

## Simulation fidelity

Rigid PhysX, 400 Hz physics (20 substeps per 20 Hz control step; yam_arena's default is 240 Hz), GPU dynamics.
- **Slide:** one rigid body of 4.7 g made of three boxes: label end 20 x 25 x 1 mm, specimen part 55 x 25 x 1 mm and
  the coverslip 22 x 22 x 0.2 mm fused on top; contact offset 0.2 mm, rest offset 0 (also on the stage plate,
  template and caliper). The scorer reads each gripper contact's point and normal (PhysX contact data) to tell an edge
  touch from a face touch. GPU PhysX does not support CCD (Isaac Lab turns it off with GPU dynamics); the 400 Hz rate
  stands in for it (at the demo's speeds the slide moves well under its 1 mm thickness per substep).
- **Friction (estimates):** glass on painted metal 0.3 (min-combined, so it rules the slide's contacts with the stage,
  template, caliper and holder), glass on the finger pads 0.9. The fingers' force is limited to 10 N in this task.
- **Stage:** the frame, nosepiece, objectives and head are static. The mechanical stage is an articulation: a fixed
  lower plate, a Y carriage (prismatic along x) and an X carriage (prismatic along y) carrying the upper plate and the
  template. Each carriage axis has stick-slip friction: a stiff spring (100 N/mm) holds it to a rest point that the task
  drags along once the spring's force passes the breakaway force (5 N, est.), so a shove harder than that moves the
  stage and it stays moved.
- **Caliper:** a revolute arm on a vertical axis on the X carriage, swinging in the stage plane, with a torsion-spring
  drive preloaded to 1.5 N at the tip on a seated slide (est.) and low damping, so a dropped lever snaps shut (about
  400 mm/s at the tip, est.; the scorer's 20 Hz reading gave 118-142 mm/s over the last 5 mm in let-go checks, the
  whole close taking under one step). The damping will be set from the 240 fps measurement on the real stage (the sim value is an estimate).
- **Not simulated:** glass cracking and chipping (the real trial inspects the slide; in the sim, face contact, objective
  contact, drops and the hard-landing flag are the rules standing in for it), and the nosepiece's and stage's focus
  travel (both fixed: stage fully lowered, 4x in the path).
- **Sizes:** the microscope's frame, nosepiece, stage and caliper are estimates from AmScope's specifications and
  comparable instruments (`geometry.py`, marked est.); the README's measured values replace them before the real trials.

## Reset between trials

Push the caliper lever out and slide the slide forward by its front edge until it overhangs the stage; take it by its
long edges and stand it back in the middle slot, label up, coverslip toward the microscope; release the caliper gently;
reset the X and Y scales, lower the stage, turn the 4x into the path. Wipe the slide with lens tissue if marked; replace
it from the spares if cracked or chipped. If glass breaks: put on cut-resistant gloves, pick up the pieces with tweezers
or a dustpan and brush (never bare fingers or the grippers), put them in a broken-glass box, and wipe the stage and pads
with a damp tissue to pick up slivers. Check the microscope is still on its tape. Nothing is used up.

## Sources

- AmScope 120-R series manual (B120-R), section 2.1 "Mounting a Specimen" and specifications.
- AmScope product pages for the M620 and B120C (amscope.com, prices and specifications: two-layer stage with caliper,
  forward quadruple nosepiece, 45 mm parfocal objectives; side and front photos of both on the same stand).
- Olympus CX23 instruction manual, section 3 "Placing the specimen slide"; CX23 brochure (dimension drawing, stage
  height about 190 mm).
- Nikon Eclipse E100 instruction manual, "Specimen Mounting".
- University of Ottawa biology teaching lab, Olympus CX41 page (specimen holder).
- Chabot College BIOS15 lab manual (LibreTexts), microscope handling and care.
- Carolina Biological, "How to Handle, Store, and Repair Microscope Slides"; Microscope International, "All about
  microscope slides".
- Heathrow Scientific slide holders (HS15981C); Carolina cover glass 633095; Fisherbrand ground-edge colour-frosted
  slides.
