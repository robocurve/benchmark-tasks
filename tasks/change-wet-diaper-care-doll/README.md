# Change a newborn care doll's wet diaper

A 53 cm vinyl newborn care doll lies face up on a changing mat. Its used (wet) diaper has its tabs undone and is
folded in half under its bottom; a clean newborn diaper lies open at its feet, back half toward the doll, the legs
resting on it; a small open bin stands at the robot's front left. The robot must change the diaper:

1. lift the legs by the ankles until the bottom comes up off the mat;
2. pull the used diaper out from under the raised bottom and put it in the bin;
3. slide the clean diaper's back half (with the hook tabs) under the bottom, up to the lower back;
4. lower the legs, bring the front up between them onto the belly;
5. pull each tab round its hip and press it onto the front's landing zone, keeping the front centred, then let go.

**Instruction:** "Change the baby doll's wet diaper for the clean one, used diaper in the bin, then let go."

## Why it matters, and what is new

Diapering is one of the most common acts of infant care: newborns typically go through about eight to twelve diapers a day
([Pampers](https://www.pampers.com/baby/diapering/article/how-to-change-a-diaper); the
[NHS](https://www.nhs.uk/baby/caring-for-a-newborn/how-to-change-your-babys-nappy/) says up to 10 or 12 changes), and "change the diapers of
infants and toddlers" is a listed duty of US childcare workers, 1,010,300 jobs in 2025
([BLS OOH](https://www.bls.gov/ooh/personal-care-and-service/childcare-workers.htm)). The method follows Nemours
KidsHealth ([diapering](https://kidshealth.org/en/parents/diapering.html)): lift the legs gently by the ankles and slide
the clean diaper under, pull the front panel up between the legs onto the belly, fasten the tabs.

The tested ability is a coordinated two-hand swap under a body: one hand must keep the bottom raised (the doll's
weight pins anything under its pelvis), while the other works sheets out from and into the narrow gap, then wraps a
flexible garment round the hips and fastens it. The difference from the closest tasks is listed under Closest tasks below.

## Closest tasks

- Newborn doll held with legs flat while a measuring footboard closes: scores straight legs and a board reading. This task lifts the legs to free the space under the pelvis and scores the diaper's end state.
- Canopy fitted onto a baby carrier: fits a fabric part onto a rigid frame with no body involved. Here soft sheets are swapped under a body that one hand keeps raised, then wrapped round its hips and fastened.
- Blood-pressure cuff wrapped on a mannequin arm: one strap round a fixed limb, closed. This task adds the swap under the raised body, and scores the order and placement under it.
- Shoe fitted on a manikin's foot: a rigid item on a held limb. This task scores soft sheets moved under and round a body.

## Real trials

- Doll: Nasco 21 in newborn care doll (soft vinyl, jointed arms and legs; listed e.g. as SB26943), face up on a vinyl changing mat.
- Diapers: newborn-size disposable diapers (size N). The "wet" diaper is a dry diaper marked beforehand with a blue stripe
  (marker); no liquid is on the table. It is laid with its tabs undone, folded in half under the bottom. Diaper sizes
  vary by brand: the sim models a 35.5 cm clean diaper. Use one brand for every trial (e.g. Pampers Swaddlers size N);
  before the first trial measure an opened one and check it is 34-37 cm long, and lay it out the same way. Use a fresh
  clean diaper for each trial.
- Press check, before the first trial: with the tape on the landing zone, a fingertip press of about 5 N (read on a kitchen
  scale under the diaper) should fasten it and a light touch should not, as in the sim's press rule. In a real trial a tab
  counts as fastened if it stays flat on the zone after the hand lets go; one that lifts off is loose. The sim sheets do
  not collide with each other; the real-trial score is judged on the end state (used diaper in the bin, clean one
  fastened), which does not depend on that.
- Landing zone: if the diaper has no printed one, outline a 6.5 x 13 cm rectangle in green marker on the front's outer
  face, centred across, top edge 1 cm below the waistband (the tapes stick to the outer cover; the outline is only for
  scoring).
- Bin: any open plastic bin about 18 cm across and 7 cm tall, on the table past the doll's head.
- Wet-only change: no wiping is needed, so no wipes are on the table.
- Scoring from an end photo and a ruler (see task.json "scoring"; mark the hip line 3 cm and the navel sticker 11.5 cm above the crotch line): used diaper inside the bin; clean diaper's back
  under the bottom with its edge above the hip crease; front up on the belly with its edge within 4 cm of the navel
  sticker; each hook tab on the green landing zone; tab heights level within 3 cm; hands 5 cm clear.
  After the end photo, lift the legs by the ankles and measure the back edge from the hip-line mark (5-14 cm above
  it), then lower them; judge the used diaper in the bin by eye (at least 80% of it below the rim; use a fresh clean diaper each trial so the tapes grip the same).

## Simulation

- `task.py`: scene, doll articulation, scoring. `diaper.py`: the diaper sheets.
- The doll is a fixed torso with a hinged pelvis (pivots on the lower back), legs (both on one hip axis; the stiff vinyl hip
  flexes 9 deg before the pelvis tilts) and feet (free ankle hinge). Mass 2.3 kg in all, 0.8 kg in pelvis, legs and feet.
  The real doll has separately jointed hips. In a diaper change the torso stays flat on the mat and lifting both ankles
  together rolls the pelvis up about the lower back, so a fixed torso, a hinged pelvis and one shared hip axis keep the
  motion that matters here (the gap opening under the bottom); independent leg spreading is not modelled.
- Each diaper is a reduced position-based sheet: a node grid (about 2 cm along, 1-2.4 cm across), stretch links that resist
  extension only, bilateral links that stiffen the core. Rules, all triggered by the measured gripper state: a closed
  jaw pins up to 3 nodes in its pad zone (edge and tab nodes of the clean diaper, any node of the folded used one; a
  jaw already holding one diaper takes only that upper layer); a pinched hook tab held pressed on the landing zone
  (touching it: within 4 mm of its mid-surface, where the two 2 mm sheet half-thicknesses meet) and pushed at least
  1.5 mm further into it toward the body for 0.25 s fastens there; merely touching does not fasten. The jaw is
  position-controlled and the sheets push no force back, so press depth stands in for the press force: a firm
  fingertip press of a few newtons (standard hook-and-loop peel tests close samples under an 11 lb, ~49 N, roller,
  [ASTM D5170](https://www.astm.org/d5170-98r21.html)); it is not calibrated and stays fastened when the jaw opens, and peels if a pinch
  pulls it 3 cm off; nodes on the mat under the resting pelvis or torso cannot slide;
  a touching node that would move under about 10 cm/s stays put (static friction). Sheets collide
  with the doll's shapes, the mat, the bin and the finger pads; they do not push on the rigid bodies and have no
  self-collision. The sheets are drawn as meshes, so the cameras see them.

## Run a trial

Requirements: yam-isaac-lab-arena and Isaac Lab-Arena 0.3.1 installed; `diaper.py` sits next to `task.py`.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3600 \
  --instruction "Change the baby doll's wet diaper for the clean one, used diaper in the bin, then let go."
```

Run limit 3,600 steps (180 s at 20 Hz). The layout is fixed (object_jitter=0); nothing is randomised.

## Scoring (score() at the end of the run)

Score 0 if a diaper sheet went non-finite or was torn at any time (in sim, any link stretched past 3x its rest length, where the sheet tears; in a real trial, a visible rip or a tab pulled off), a gripper still holds a diaper or is within 5 cm of the
doll or a diaper, any used-diaper node is within 1.5 cm of the doll's surface, or less than 80% of the used diaper is inside the
bin. Otherwise 0.25; 0.5 if the clean diaper's back panel (centre, 3-8 cm from its back edge) lies under the pelvis
and its back edge is 5-14 cm above the hip axis with the pelvis down (lumbar hinge within 0.1 rad of flat); 0.75 if also the front edge is up on the belly
(over 6 cm above the mat) within 4 cm of the navel both along and across the body, the crotch centre within 3 cm of the
body midline, and at least one tab attached on the landing zone (a front-up diaper with no tab attached stays at 0.5); 1.0 if both tabs are attached on the landing zone at heights
along the body within 3 cm of each other. A run that reaches 1.0 ends early once that end state has held for 3 s.

## Demo video

The video shows the arms lifting the legs by the ankles, pulling the used diaper out into the bin, sliding the clean
diaper under, lowering the legs, bringing the front up and fastening both tabs, then opening the hands and backing away.
It reaches 1.0 in 1,931 of the 3,600 steps, including the 3 s hold with both hands clear.

## Shortcut checks

Scripted policies scored by the scorer, all at 0 except where noted: doing nothing; putting the clean diaper on over the
old one; leaving the used diaper on the mat instead of the bin; one arm only; one tab only (0.75, as the tiers say);
never letting go with the last hand. A tab held touching the landing zone (3.3 mm off it) without the press stays loose.
