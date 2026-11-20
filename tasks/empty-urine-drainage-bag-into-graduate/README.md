# Empty a urinary drainage bag into a graduate without the tube touching it, then close the valve and sleeve the spout

A Bard 154002 2000 mL urinary drainage bag hangs from a catheter-bag floor holder beside the bed. It holds 600 mL of
clear tap water standing in for urine. The robot sets a 1000 mL graduate under the drain, takes the drain tube's spout
out of its sleeve, aims the spout into the graduate's mouth without letting the tube touch the graduate, opens the
twist valve and lets the bag empty, following the stream as its arc shortens, then closes the valve and pushes the
spout back up into its sleeve. The whole scene stands in a plastic drip pan that catches the water of a failed trial,
and both grippers wear waterproof covers.

## Why it matters

**Who does it, and how many.** Emptying the drainage bag into a graduate is how urine output is measured for a patient
with an indwelling (Foley) catheter, and intake and output are on the task lists of two large occupations:

| Occupation (O*NET) | US jobs (O*NET OnLine, 2025) | O*NET task line (verbatim) |
|---|---|---|
| Nursing Assistants (31-1131.00) | 1,505,900 | "Measure and record food and liquid intake or urinary and fecal output, reporting changes to medical or nursing staff." |
| Licensed Practical and Licensed Vocational Nurses (29-2061.00) | 666,900 | "Record food and fluid intake and output." |

Together that is about 2.2 million US workers. Registered nurses do it too, but it is not on their O*NET list, so they
are not counted.

**How often.**
- Between 15 % and 25 % of hospitalized patients may receive a short-term indwelling urinary catheter (CDC, CAUTI
  guideline background); about 5 % of US long-term-care residents have one at any time.
- Each bag is emptied at least once a shift and whenever it is half full. One US academic hospital's nursing
  guideline (2025; done by "Registered Nurse (RN), or Licensed Practical Nurse (LPN), or Nursing Assistant (NA), or
  Patient Care Technician (PCT)"): "Empty drainage bag, so it remains less than half full. Empty bag at a minimum each
  shift."
- So every catheterized patient's bag is emptied at least 2-3 times a day (8 or 12 h shifts), every day, by the aide or nurse on shift.

**Why it is core, not ancillary.** It is the measurement that the O*NET lines above describe (urine output), and the
step most tied to infection control. It is a graded skill in nurse-aide competency testing (the Arizona skills
checklist "Catheter Care - Emptying a Urinary Drainage Bag"; its steps are in the table below). The CDC's
catheter-associated UTI (CAUTI) guideline (2009), recommendation III.B.3 (Category IB, kept in the 2022 SHEA/IDSA/APIC update), sets the method: "Empty the collecting bag
regularly using a separate, clean collecting container for each patient; avoid splashing, and prevent contact of the
drainage spigot with the nonsterile collecting container." The same guideline keeps the bag below the bladder and off
the floor and the system closed; CDC states that up to 380,000 CAUTIs and 9,000 related deaths a year could be
prevented.

**Why it is hard for a robot.** The spout hangs on soft tubing. One hand has to hold it a few centimeters above the
graduate's mouth without letting the tube touch the rim, the inside or the floor, while the other twists the valve
collar on that same tube, and at the end the spout has to go straight up into a sleeve whose bore is 1 mm wider than the collar.
The full bag throws the stream forward in an arc, and as the bag empties the stream slows and its arc shortens, so the aiming hand has to follow it. A picture of the end state does not show any of this.

## The scene and how this matches the real job

The real items (each is in the sim at the sizes under Fidelity):
- **The bag:** Bard (BD) 154002 urinary drainage bag. BD lists it as 2000 mL; McKesson lists it as vinyl, with an
  anti-reflux valve, a hook and string hanger, 48 in tubing and a twist-valve drain tube. Its clear drain tube leaves
  the middle of the bag's bottom, runs out 8.5 cm toward the robot and folds up 11 cm in front of the bag; the
  twist-valve collar at its end, with the spout past it, sits closed with the spout pushed fully home in the bag's
  drain-spout sleeve.
- **The sleeve and the drain's sizes:** the drain spout parks in a sleeve on the bag ("Put the drain spout back into
  its sleeve at the bottom of the bag", patient guide; Bard's patent US4417892: the outlet's free end "is received in
  a protective housing heat sealed to one face of the urine drainage bag"). It holds the spout by its grip alone:
  pushed fully home, the spout stays; pulled down, it comes out. The drain tube is 9 mm across. The twist-valve collar
  is 22 mm long and 12 mm across, a quarter turn from closed to open, and the spout past it is 15 mm long and 8 mm
  across. The sleeve is 30 mm deep from its mouth to its closed top, its bore 13 mm across (1 mm wider than the
  collar), flaring over its lowest 4 mm to an 18 mm mouth. Fully home, the spout's tip reaches the closed top, the
  spout fills the upper 15 mm and the collar the lower 15 mm, and 7 mm of the collar shows below the mouth.
- **The hanging height:** the bag hangs from a 50 cm catheter-bag floor holder, hook 48 cm up, bag bottom 21 cm up:
  below the bladder and off the floor, as the CDC asks. The drip pan's floor stands in for the floor beside the bed.
- **The fill:** 600 mL, 30 % of the bag's 2000 mL: a bag emptied at the end of a shift, well before it is half full.
- **The liquid:** clear tap water, in the sim and in the real trial: hygienic and the same every trial, and urine is
  mostly water. The sim draws it light and slightly blue-tinted.
- **The graduate:** an Eisco CH0357D 1000 mL short-form measuring jug, transparent polypropylene, graduated, with a
  handle and a pour spout. It does the graduate's job (a clean graduated container for this patient) and has a handle,
  so the robot carries it by the handle.
- **The barrier:** a blue Medline disposable underpad on the floor under the bag, folded in half both ways (29 x 46 cm).
- **The floor and the spill catch:** an Oatey 34169 plastic washing machine pan (28 x 30 in inside, 2.5 in deep; 29.75
  x 31.75 in outside) under the whole scene. Its floor is the floor beside the bed; its walls are far below the drain
  and the valve, and the arms keep 20 mm or more clear of them all through the demo (`test_task.py`, on the
  robot's collision hulls). A dropped tube, a stream that misses the mouth or a knocked-over graduate drains into it,
  not onto the table.

| Step of the real procedure (Arizona checklist, CDC III.B.3) | What the robot does | What the scorer checks |
|---|---|---|
| Place a barrier on the floor under the bag | the underpad is already down (setup) | spill landing on it counts as spilled |
| Place the graduate on the barrier, under the drain | carries the graduate by its handle onto the underpad (it is not pre-placed) | graduate upright (within 10 degrees) and standing on the underpad at the end (1.0) |
| Take the drain out of its holder | one hand pinches the tube and pulls the spout straight down out of its sleeve | spout back up in its sleeve at the end (1.0 only) |
| Hold the spout over the graduate, not touching it ("any part of the tubing") | aims the spout into the mouth, a few centimeters above the rim | any part of the drain tube within 0.5 mm of the graduate's rim, inside wall or inside floor at any time caps the trial at 0.25 |
| Open the drain | the other hand twists the valve collar a quarter turn open | flow follows the valve's opening |
| Empty the bag completely, no splashing | aiming hand follows the stream as it slows and its arc shortens | at least 570 mL (95 %) in the graduate; at most 5 mL spilled for 1.0, over 20 mL scores 0 |
| Close the drain | twists the collar back to its closed stop | valve within 10 degrees of closed (0.75 and up) |
| Replace the drain in its holder | swings the tube up and pushes the spout straight up into its sleeve until it is fully home | spout fully home in its sleeve, both hands clear, everything still for 1 s (1.0) |

Not in the task: gloves, hand hygiene, wiping the drain with alcohol after emptying, reading the level at eye level
and recording it (the scale gives the exact volume instead), and pouring the urine away.

Real failures stay possible: the spout or the tube touches the rim, the inside wall or the floor of the graduate (the
CDC and checklist violation), the stream lands beyond the mouth (or, as it shortens, short of it), a hand gets in the
stream's way (it splashes off), the tube is dropped and hangs down with the valve open (draining onto the floor), the
graduate is knocked over, shaken so it slops, or left off the barrier, the valve is left open, the spout is left out of
its sleeve or only partly in (less than 11 mm of the collar inside).

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/empty-urine-drainage-bag-into-graduate/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Empty the drainage bag into the graduate without letting the drain tube or its spout touch the graduate, then close the drain and push the spout back into its sleeve." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/drainbag-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.
Video: `inspect-robots video logs/drainbag-demo/*.json`. The scorer (`task_score`) reports the score each step; the
trial's score is the one at its end. Offline checks: `python3 test_task.py` in this folder.

## Scoring

"In the graduate" is the water that reached it. "Spilled" is what landed anywhere else (underpad, pan floor), including
a stream that runs into a hand or arm on its way (it splashes off).

A touch is real contact with the surface the real trial coats with lipstick: any point of the drain tube's surface
(its run out of the bag, the folded-up tube and the valve collar with its mouth, the spout) within 0.5 mm of the
graduate's rim (its top edge, the pour spout's lip included), inside wall or inside floor, at any time from the start
of the trial to its end. The checklist says "Avoids touching the graduate with any part of the tubing", so the whole
tube counts. It is checked every control step and along the motion between steps, so a quick swipe across the rim
counts too. As lipstick rides on a hand, a robot link that has touched that coated surface and then touches the tube
also makes a touch. The graduate's outside wall and handle, where the hands carry it, are left bare in the real trial
and do not count.

"In its sleeve" is where the spout is: fully home, its tip within 4 mm of the sleeve's closed top, with at least 11 mm
of the collar inside the sleeve (at most 11 mm showing below the mouth) and the collar straight in the bore. Fully home
the tip is at the top with 15 mm of the collar inside and 7 mm showing, so both thresholds come from the sleeve's
30 mm depth and the 15 mm spout, with the same 4 mm margin. What keeps it there is the sleeve's contact and friction: for 1.0, no part of
either arm (on the robot's collision hulls: fingers, palm, wrist, forearm) may be within 1 cm of the tube, so nothing
else holds it up.

| Score | Condition |
|---|---|
| 0 | more than 20 mL spilled (a puddle on the floor), or nothing done |
| 0 | a stream running into a hand or arm for more than 1 s in all |
| 1.0 | at least 570 mL in the graduate (95 % of the 600 mL), at most 5 mL spilled, the valve closed (within 10 degrees of its closed stop), the spout fully home in its sleeve (tip within 4 mm of its top, at least 11 mm of collar inside), the graduate upright (within 10 degrees of vertical) and standing on the underpad (the middle of its base over it), both hands clear (every tool point and pad tip 1 cm from the spout, the tube and the graduate, and in the sim no part of either arm within 1 cm of the tube) and everything still, held 1 s |
| 0.75 | at least 570 mL in the graduate, the valve closed and the graduate upright |
| 0.5 | at least 570 mL otherwise (valve left open, graduate tipped after), or at least 300 mL with the valve closed and the graduate upright |
| 0.25 | at least 100 mL in the graduate |
| cap 0.25 | a touch at any time caps the trial at 0.25 |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Run limit: 2,400 steps (2 minutes at 20 Hz). Every quantity is read in the
graduate's own frame, so the score is the same wherever on the underpad, and turned whichever way, the graduate is
set down (`test_task.py` checks four turns and five positions).

## Real-world build

A low-cost setup of named, widely available products; nothing has to be made, and one set serves the whole session.

- Use a Bard (BD) 154002 urinary drainage bag, 2000 mL, with its anti-reflux chamber, twist-valve drain tube and
  drain-spout sleeve (a stock hospital-supply item; one bag lasts the session, as it only ever holds tap water).
- Use a catheter-bag floor holder for drainage bags, 50 cm tall (e.g. the Essential Aids urine catheter bag floor
  holder, plastic-coated metal), its base held to the pan's floor with two strips of 3M duct tape. It puts the hook at
  the sim's 48 cm.
- Use two Eisco CH0357D 1000 mL short-form polypropylene measuring jugs as graduates, one in the trial while the other
  is washed and re-coated.
- Use a Medline disposable underpad (23 x 36 in), folded in half both ways to 29 x 46 cm.
- Use clear tap water, 600 mL per trial (no coloring: it stays hygienic and identical trial to trial), and a small
  plastic kitchen funnel (e.g. the smallest of an OXO Good Grips 3-piece funnel set) to fill the bag.
- Use a basic digital kitchen scale reading 1 g to 5 kg (e.g. Etekcity EK6015 or OXO Good Grips 11 lb food scale); it
  also weighs out the fill.
- **Spill containment:** an Oatey 34169 plastic washing machine pan (28 x 30 in inside, 29.75 x 31.75 in outside with
  its lip, 2.5 in deep), its 1 in PVC drain adapter left off and the drain hole taped shut with 3M duct tape. It stands
  on the table under the whole scene (holder, bag, underpad and graduate) and holds many times the bag's 600 mL, so a
  dropped tube draining onto the floor, a stream that misses the mouth or a knocked-over graduate stays in the pan. Its
  walls are far below the drain and the valve, so both stay as reachable as without it.
- **Gripper and arm protection:** a powder-free nitrile finger cot (First Aid Only 90922, large, 4.25 in long) rolled
  over each fingertip; if it does not fit the flat fingertip snugly, a finger cut from a nitrile exam glove (the
  gloves the setup already uses) instead, its open end taped down with Scotch Super 33+ vinyl electrical tape. A clear
  quart-size Ziploc freezer bag goes over each gripper's body: its two bottom corners are cut so the fingers come out
  through them, the cut edges taped flat to the outside of each finger's base with the same tape (so no loose film
  reaches the fingers' slide), and its mouth taped round the wrist. Over the wrist camera's lens the bag is cut away
  and the edge taped down round the camera housing, so the lens looks out clear. A clear gallon Ziploc bag, slit along
  one side, wraps each forearm and is taped closed. A stray stream or a splash runs off the covers into the pan.
- **Covers check (once per session, before the first trial):** with the covers on, the fingers open and close fully
  five times with no film caught in the slide; the cots stay in place, with no rolling or slip, while the hand pinches
  the tube and pulls the spout out of its sleeve, carries an empty graduate by its handle, and twists the valve collar
  from stop to stop; each wrist camera's image is clear and unfogged. If a cot slips or rolls, that finger is
  re-covered with a cut glove finger and the check repeated.
- **Touch check:** a red cream lipstick (Revlon Super Lustrous Lipstick in Certainly Red, a cream finish; not a matte,
  long-wear or transfer-proof formula), white paper towels, a white LED flashlight, and Dawn dish soap to wash it off.
- Use ScotchBlue 2090 painter's tape and a 30 cm steel ruler (e.g. Westcott) for the setup marks, and a phone on a
  tripod mount for the side video.
- Safety: tap water only; nothing sharp, hot or heavy (the full bag is 0.7 kg: 600 g of water and the bag); the
  pan keeps the water off the table and the robot's bases, and the operator stops a failed trial (below).

**Operator stop.** If the stream runs onto a hand or an arm for more than 1 s in all (a 0 already), or the tube is
dropped with the valve open, the operator stops the robot, twists the valve closed and ends the trial; it is scored as
it stands.

**How the touch check works.** Lipstick is wax, oil and pigment: it does not dissolve in water, and it leaves a mark
on what touches it (lipstick traces on drinking cups and glasses are an everyday example). The graduate's rim (its top
edge, the pour spout's lip included), inside walls and inside floor carry a thin coat; its outside wall and handle stay
bare, as the hands carry it there. The drain tube, the valve collar and the spout stay bare, so the hands hold and
twist exactly the clean parts the sim models. After the trial the whole drain tube, the collar and the spout's end face
are wiped with a white paper towel: any red on the towel or the tube means some part of the tube touched the coated
graduate at some moment of the trial. The stream landing on the coated inside, or the spout dipping into the collected water,
does not carry red to the tube, as the coat does not dissolve. The sim scores the same events: the tube within 0.5 mm
of the rim, inside wall or floor, or a robot link that touched them and then the tube (a finger reddened on the rim
reddens the tube it then holds).

Times: first setup about 25 minutes (pan, marks, holder, covers and their check, the coat check, weighing the
underpad); each reset about 8 minutes (drain and refill the bag 2 minutes; rehang, close and sleeve the spout 30 s;
wash the used graduate and the tube with dish soap, rinse and dry them, and coat the spare graduate 4 minutes; new
underpad and finger cots, and a look over the bag covers 1 minute; wipe the pan 30 s); the measurement 4 minutes.

## Real-world evaluation (reproducible protocol)

The products and tools are those listed under Real-world build. Positions are in cm from the midpoint between the arm
bases (x forward, y to the robot's left).

**Setup (marks and tolerances; about 25 minutes the first time)**
1. Set the pan on the table with its near right outside corner at x 6.5, y -42 cm (within 5 mm), its near edge along
   the y axis, and tape its corners to the table with painter's tape. Heights below are from the pan's floor.
2. Weigh each empty graduate (the model's 136 g within 15 g).
3. Tape the holder's base to the pan's floor so its hook sits at x 42, y -4 cm (within 5 mm), 48 cm above the pan's
   floor (within 5 mm; check with the ruler).
4. Pre-weigh the dry underpad (1 g) and tape its corners flat centered at x 28.5, y 4.5 cm, its long side along y
   (within 1 cm).
5. Tare a dry graduate on the scale and weigh 600 g of tap water into it (within 2 g). Pour it through the funnel into
   the bag's inlet tube held above the bag, then dry it.
6. Hang the bag on the hook, front toward the robot, twist the valve to its closed stop and push the spout fully home
   in its sleeve (7 mm of collar showing below the mouth). Let go.
7. Coat the graduate: rub the lipstick round the rim's top edge (the pour spout's lip included), over the inside walls
   from the rim down and over the inside floor; leave the outside wall and the handle bare. Blot it once with a paper
   towel so the coat is thin (a thick coat smears). Tare it and write down its empty mass. Check the coat once per
   session on the spare graduate: a light tap of a clean dry valve collar (a spare bag's, or the trial bag's before
   step 5) on the dry coated rim, and another of the collar dipped in a cup of water on the coated inside wall wetted
   with water, each leave red on the collar; wipe the collar clean afterwards.
8. Fit the gripper and forearm covers and do the covers check (Real-world build).
9. Stand the coated graduate on its tape circle at x 23, y -25.8 cm (within 5 mm), handle toward the robot's right.
10. Stand the phone on its tripod 60 cm to the robot's right of the graduate's spot, its lens at the rim's height, and
    film at 60 fps; check once, with the tube held at the draining pose, that the graduate, the spout, the stream's
    path and both hands are in frame.

**Measurement (4 minutes, six readings)**
1. Graduate: weigh it; net mass in g = mL in the graduate.
2. Spilled: weigh the underpad; its gain in g, plus what is on the pan's floor and the covers wiped up with pre-weighed
   paper towels, = mL spilled.
3. Touch: wipe the whole drain tube from the bag's outlet down, the valve collar and the spout with a white
   paper towel and look at the towel and the tube under the flashlight. Any red is a touch. Photos of the towel and the
   tube.
4. Stream into a hand: from the side video, the total time the stream runs into a hand or an arm; more than 1 s scores
   0. Water that splashes off is in the spill reading.
5. Valve and sleeve: photo of the collar at its closed stop, and a side photo of the sleeve's mouth with the ruler
   held against the collar: fully home shows 7 mm of collar below the mouth; more than 11 mm showing (the tip more
   than 4 mm short of the top) is not in its sleeve. No drip from the spout for 30 s (video).
6. Graduate upright, standing on the underpad, and both hands clear at the end: the side video and a photo from above.

**Reset (about 8 minutes):** unhook the bag and drain what is left into the sink; refill it as in setup step 5 (with
the spare graduate, dry); rehang, close it and push the spout fully home in its sleeve (7 mm of collar showing); wash the used graduate and the drain
tube with dish soap and warm water, rinse and dry them; coat the spare graduate as in step 7, tare it and put it on its
circle; lay a fresh pre-weighed underpad (or the same one once it is dry and back to its start mass); wipe the pan's
floor and the covers dry, look the bag covers over under the flashlight and replace any with red on it, and fit fresh
finger cots.

**Tolerances**

| Item | Value | Tolerance |
|---|---|---|
| Pan | near right outside corner at x 6.5, y -42 cm; near edge along y | 5 mm |
| Hook position and height | x 42, y -4 cm; 48 cm above the pan's floor | 5 mm |
| Bag fill | 600 g (600 mL) | 2 g |
| Graduate start | x 23, y -25.8 cm, handle right | 5 mm |
| Underpad | 29 x 46 cm at x 28.5, y 4.5 | 1 cm |
| Scale | 1 g reading | 1 g |
| Full drain | 570 mL in the graduate | scale |
| Spill | 5 mL full credit, 20 mL limit | scale |
| Touch | any part of the drain tube within 0.5 mm of the rim, inside wall or floor in the sim; any red on the tube in the real trial | towel, flashlight, photos |
| Spout in its sleeve | fully home: tip within 4 mm of the sleeve's top, at least 11 mm of collar inside (at most 11 mm showing below the mouth) | photo with a ruler |
| Stream into a hand | more than 1 s in all scores 0 | side video |
| Valve closed | within 10 degrees of its closed stop; no drip in 30 s | photo, video |
| Graduate upright, on the underpad | within 10 degrees of vertical; base middle over the pad | video, photo |

## Design notes

- **The holder and the bag.** One articulation fixed to the pan's floor at the holder's base, its parts colliding with
  each other (only the tube's first run, which starts inside the bag's outlet, is excused against the bag). The bag
  hangs on a hinge at the hook: a pull on the tube swings it, as on a real holder; the holder's frame stops it swinging
  back.
- **The sleeve.** A sleeve on a tab in front of the bag, open at the bottom and closed at the top, 30 mm from its mouth
  to its top, its bore 13 mm across (1 mm wider than the collar), its lowest 4 mm flared out to an 18 mm mouth: convex
  panels on the bag's own body, so the collar and spout touch them. Fully home, the spout's tip is at the closed top,
  the spout fills the upper 15 mm and the collar the lower 15 mm, with 7 mm of it showing below the mouth. A springy pad in its bag-side wall stands in for the real sleeve wall's springiness: a slide across the bore
  (it can only push the collar sideways onto the far wall, never along the sleeve), held against a stop by a light
  spring and chamfered top and bottom so a collar end coming in or out meets a slope. The collar's friction on the pad
  and the far wall holds the tube up. The pad's push and the plastic's friction are model values (the sleeve's
  colliders carry a plastic surface; the tube and everything else of the bag carry the environment's grip material);
  over half to twice that push and friction from 0.25 to 0.5 the grip still exceeds what the tube's weight asks
  (`test_task.py`). Nothing else holds it: with the
  sleeve's colliders taken out (`DRAINBAG_NO_SLEEVE=1`, a check run only) the tube drops out of it. The pad's spring is
  on from the first physics step after every reset, so the spout stays in through the scene's settle before the trial
  starts. To score, the spout must be pushed fully home: its tip within 4 mm of the top, at least 11 mm of the collar
  inside, the collar straight in the bore (`in_holder`, from the sizes in one block at the top of `task.py`).
- **The drain.** A lumped flow model: no particles. The bag is a pillow: welded flat along its bottom seam, its section
  grows as sqrt(height) over the bottom 8 cm, then stays 18 x 4.5 cm, so 600 mL stands 10 cm deep. It drains as an
  orifice, Q = Cd A sqrt(2 g h) with Cd 0.61 and the 6 mm bore, h from the bag's level down to the spout, scaled by how
  far the valve is open: the flow slows as the bag empties. Up in its sleeve, the spout is above the full bag's level,
  so nothing flows even if the valve is opened there.
- **The stream.** It leaves the spout along its axis at sqrt(2 g h) and falls ballistically. It is caught if it crosses
  the plane of the graduate's mouth inside the rim; otherwise it lands on the floor and is spilled. As h falls the
  stream slows and its arc shortens. Each step the arc is checked against both arms' own collision hulls (the robot
  model's, at their live poses): a stream that runs into a finger, a hand or an arm splashes off it onto the floor
  and is spilled, and more than 1 s of it in all scores 0. If either arm's hulls cannot be read in the sim at any
  step, the log says so and the trial cannot score 1.0.
- **The graduate's water.** A liquidkit vessel of the graduate's inside shape: its surface tilts with the graduate's
  motion through the first sloshing mode (natural frequency and viscous damping of water in a tank of its size, after
  NASA SP-106 as updated by Dodge), so a hard shake or jolt slops water over the rim; tipped, it pours over the rim as
  a weir. A gentle move spills nothing.
- **The touch.** The whole drain tube counts, as on the checklist. The coated surface is the rim's top (the jug's
  pour spout, which the model leaves out, counted as rim there), the inside wall and the inside floor, on the
  graduate's own colliders. Points over the drain tube's surface, no point of it more than 0.4 mm from one, are
  checked against the coated surface every control step and along the straight motion between steps; a gap of 0.5 mm
  or less is a touch. A robot link that comes within 0.5 mm of the coated surface carries lipstick; if it then comes
  within 0.5 mm of the tube, that is a touch too. The tests
  place the graduate at 0.3 mm (a touch) and 0.7 mm (none) from the tube on the rim, the inside wall low and high and
  the floor, check that the bare outside wall and handle are not touches, and check a swipe across the rim within one
  step, a graduate tipped and raised to meet the spout, the tube laid over the rim (and over the pour spout's lip) with
  the spout inside the mouth, the rim in the tube's knee at bends from 0 to 120 degrees, a fingertip carrying red from
  the rim to the tube, and a spout hovering inside the mouth (no touch).
- **The demo** (open loop, 106 s). The right hand carries the graduate by its handle onto the underpad under the
  drain. The left hand pinches the tube, pulls the spout straight down out of its sleeve, raises the run to 39 degrees
  above horizontal and aims the spout 4.4 cm above the rim; the right hand twists the valve a quarter turn open and
  stands off. The left hand follows the shortening stream through six waypoints planned from the scorer's own drain
  model. 49 s after the flow starts, the right hand twists the valve closed, and the left hand swings the tube up and
  pushes the spout fully home in its sleeve (aiming 25 mm past the sleeve's top, which stops the spout); both hands
  let go and back away. Offline, on the robot's collision hulls at the planned joint angles every 0.25 s, no arm comes
  within 5 mm of the coated surface, once the graduate is down none comes within 8 mm of it, and the arms stay clear
  of the sleeve and the bag throughout.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes).
- Scripted demo: the plan lasts 106 s and is open loop; its score on this version is in task.json (scores).
- No randomness: `-E object_jitter=0` keeps the graduate on its mark; the bag starts with 600 mL every trial.

## Files

- `task.py`: the scene, the drain model, the scorer and the scripted demo.
- `yamkit.py`: spawns the holder, bag, tube, valve and sleeve as one articulation whose parts collide (the sleeve's pad
  spring is its only law) and the graduate as a rigid prop, and gives the scorer the grippers and joint states.
- `liquidkit.py`: the graduate's liquid (volume, surface, slosh, spill over the rim when tipped or shaken) and the check
  of the stream against the arms' collision hulls.
- `guardkit.py`: checks that both hands are clear of the spout, the tube and the graduate.
- `test_task.py`: the offline checks.

## Fidelity

- The water is a lumped flow model (orifice flow, a ballistic stream, the graduate's sloshing volume; see Design
  notes). Volumes are conserved to the microliter; what it leaves out is splashing, dripping and surface tension.
- The vinyl bag is a rigid pillow on its hook hinge, and its mass stays at full (the 70 g bag and 600 g of water) as it
  drains. The graduate's body keeps its empty mass as it fills, and the stream pushes on nothing.
- The flexible PVC tube is two rigid links on hinges that carry only damping. Folded, it stands 6.25 cm off the bag's
  front, its spout in the sleeve on the tab in front of the bag (room for the gripper's 10 cm palm beside the bag); a real tube
  folds closer. The sleeve holds the spout by contact and friction, with a light spring pad for the springiness of
  the real sleeve's wall; its friction, 0.30 static and 0.25 sliding, is a model value set at the low end of dry PVC's
  published sliding friction on other materials (0.23-0.30), and the pad's push is a model value too. The valve
  collar turns between its two stops, which are the joint's limits.
- The graduate is round, as the jug is; its small pour spout is left out, its lip counted as rim. The tube parts'
  masses are raised slightly for the solver.
- In the sim the drip pan has its inside size and a slightly larger outside, 30 x 32 in (so the arms are checked against
  more than the real pan); the gripper and forearm covers (thin nitrile and clear bags) are not modeled; the
  once-per-session covers check confirms the covered fingers open, close, pinch, carry and twist.
- Sizes in the sim (the task's sizes): the bag 2000 mL (BD's capacity), its body a rigid pillow 18 x 4.5 x 25 cm with
  an 8 cm bottom bulge; the drain tube 9 mm across, 8.5 + 11 cm; the collar 12 mm across, 22 mm long; the spout 15 mm long, 8 mm across, 6 mm bore; the
  valve a quarter turn, clockwise seen from the spout; the sleeve 30 mm from mouth to closed top, its bore 13 mm
  across, its lowest 4 mm flared to an 18 mm mouth (fully home: 15 mm of collar inside, 7 mm showing); the graduate
  11.2 cm across the bottom and 12.7 cm across the top outside, 14.6 cm tall (the maker's 4.4 in, 5 in and 5.75 in),
  with a 2 mm wall, a 3 mm floor and 136 g; its handle's top arm reaching 4 cm out from the rim, 1.2 cm wide, and its
  grip 9 cm tall.

## Sources

- O*NET 29-2061.00 Licensed Practical and Licensed Vocational Nurses, task list and 2025 employment:
  https://www.onetonline.org/link/summary/29-2061.00
- O*NET 31-1131.00 Nursing Assistants, task list and 2025 employment: https://www.onetonline.org/link/summary/31-1131.00 ;
  BLS OEWS 31-1131: https://www.bls.gov/oes/current/oes311131.htm
- CDC, CAUTI guideline background (15-25 % of hospitalized patients; about 5 % of long-term-care residents; up to
  380,000 infections and 9,000 deaths a year preventable): https://www.cdc.gov/infection-control/hcp/cauti/background.html
- CDC, Guideline for Prevention of Catheter-Associated Urinary Tract Infections (2009), recommendation III.B.3:
  https://www.cdc.gov/infectioncontrol/guidelines/cauti/recommendations.html
- A US academic medical center's Nursing Services Guideline, Indwelling Foley Catheter Care (2025):
  https://www.utoledo.edu/policies/utmc/nursing/guidelines/general/pdfs/Guideline%20General%20-%20Indwelling%20Foley%20Catheter%20Care%20with%20Nurse%20Driven%20Protocol.pdf
- Arizona nurse-aide skills checklist, Catheter Care - Emptying a Urinary Drainage Bag:
  https://src.bryanuniversity.edu/wp-content/uploads/Catheter-Care_-Emptying-a-Urinary-Drainage-Bag_AZ-Skills-Checklist-.pdf
- Bard (BD) 154002 urinary drainage bag, 2000 mL (BD's product page): https://www.bd.com/en-us/products-and-solutions/products/product-page.154002 ;
  McKesson's listing (drain tube type "Twist Valve", "48 Inch Tubing", vinyl, anti-reflux valve, hook and string
  hanger): https://mms.mckesson.com/product/146120/Bard-Medical-154002
- The drain spout's sleeve: "Remove the drain spout from its sleeve at the bottom of the drainage bag" and "Put the
  drain spout back into its sleeve at the bottom of the bag":
  https://healthlibrary.methodisthealthsystem.org/surgery/recovery/how-empty-and-clean-your-urinary-catheter ;
  C.R. Bard, US patent 4417892 ("The free end of the outlet is received in a protective housing heat sealed to one face
  of the urine drainage bag"): https://patents.google.com/patent/US4417892
- Friction of dry PVC on other materials (kinetic: plastic tape 0.229, aluminum 0.281, wood 0.296):
  https://arxiv.org/pdf/1902.03573
- Eisco CH0357D 1000 mL short-form polypropylene measuring jug, with handle and spout (5.75 in tall, 5 in top and
  4.4 in bottom diameter, 6.75 in from spout to handle edge): https://www.eiscolabs.com/products/ch0357d
- Medline triangular graduated intake/output container, 1000 mL, for comparison (translucent, no handle listed):
  https://www.blowoutmedical.com/triangular-intake-output-containers.html
- Catheter bag floor holder, 50 cm tall, 21 cm deep, 300 g, plastic-coated metal:
  https://www.essentialaids.com/bedroom-aids/bedroom-aids-bed-pans-portable-urinals/urine-catheter-bag-floor-holder.html
- Oatey 34169 plastic washing machine pan, 28 x 30 in, 2.5 in deep, 1 in PVC adapter:
  https://www.oatey.com/products/oatey-washing-machine-pans--1320635356 ; Oatey's spec sheet for its 28 x 30 in pans
  (34169 plain, no ribs; 0.09 in minimum thickness):
  https://res.cloudinary.com/consolidated-supply/resource/raw/oatey_34067_specification_sheet.pdf ; overall
  31.75 x 29.75 in, interior 30 x 28 in (the same pan with ribs, 34067):
  https://www.webstaurantstore.com/oatey-34067-28-x-30-washing-machine-pan-with-ribs-case/83534067.html
- First Aid Only 90922 nitrile finger cots, large, 4.25 in, powder-free: https://mms.mckesson.com/product/1203101/Acme-United-90922
- Revlon Super Lustrous Lipstick, Certainly Red (a cream finish):
  https://www.temptalia.com/product/revlon-super-lustrous-lipstick/certainly-red
- Lipstick (wax, oils and pigments; traces left on drinking cups and glasses): https://en.wikipedia.org/wiki/Lipstick
- Sloshing frequency and damping of the graduate's water: Dodge, The New Dynamic Behavior of Liquids in Moving
  Containers (update of NASA SP-106), eq. (1.40)-(1.41) and ch. 2 eq. (2.6a):
  https://www2.swri.org/www2/fluid-slosh/dodge-sp-106-update.pdf

## Sources and licenses

All files in this folder are written for this task (code and text). The robot model and its collision hulls (used for
the stream-on-a-hand and lipstick-on-a-hand checks) load at run time from the competition environment (arena revision
0059d02a) and are not redistributed here.
