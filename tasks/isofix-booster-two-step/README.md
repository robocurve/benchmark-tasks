# Fit an ISOFIX booster seat in the manual's two steps

A rigid stand-in for a car's rear seat is fixed to the table. In the gap between its cushion and backrest (the seat
bight) are the car's two ISOFIX low anchorages: 6 mm round steel bars 280 mm apart, each inside a plastic insertion
guide. A high-back ISOFIX booster seat (KIDFIX i-SIZE type, 6.5 kg) stands on the cushion 190 mm from the backrest,
turned 2 degrees, its locking-arm bracket pulled out 100 mm. As in the booster's manual, the robot must (1) push the
booster back until both connectors click onto their bars, then (2) hold the red ISOFIX adjustment button in, which
lets the arms slide into the seat, and push the booster back until its back sits flat against the car's backrest.
Released, the button locks the arms again. The armrests' front ends are the push points because they are the
booster's only front faces at hand height; without the button the arms do not move, so the booster cannot be pushed
flat by force alone. A window on each armrest shows red until that side is latched, then green.

Instruction: "Fit the booster seat to the car seat's ISOFIX anchors."

## Run a trial

Files: `task.py` (scene, scorer, scripted demo), `mechkit.py` and `closedloop.py` (unchanged team kit, beside it).

    inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --instruction "Fit the booster seat to the car seat's ISOFIX anchors." \
      --scorer task_score --max-steps 1600 --store-frames

The scripted demo is the same command with `--policy yam_scripted`. `ISOFIX_DEBUG=1` prints, every 10 steps, the
booster pose, the bracket's travel, the button, the back gap, both catches and where each bar sits in its connector.
Seeds: none (fixed layout, `object_jitter=0`). Runtime: about 10 minutes (fitted by step ~600) for the scripted demo and 30 minutes for a full
1,600-step Dream-1 trial on one RTX PRO 6000 with cameras every step (simulator start-up included).

## Scoring (task.py `score()`, final step; hazards latch from any step)

- Latched, per side: the bar's centre 17 to 25.5 mm behind the connector's reference point (it sits at 20 mm when
  seated against the throat), within the 8 mm slot and within 15 mm of the arm's line, and the catch plunger down
  behind it (within 2 mm of its stop). The armrest window shows this every step.
- Flush: the booster's back face within 5 mm of the car's backrest, measured at both sides (20 mm in from its edges,
  70 mm up).
- 1.0: both sides latched and flush; every finger pad of both arms at least 3 cm from the booster (its whole box,
  button included); booster at rest (under 5 mm/s and 0.05 rad/s).
- 0.75: both latched and flush, still held or moving.
- 0.5: both latched, but not pushed flat against the backrest (step 2 not done).
- 0.25: one side latched only (looks fitted, is not).
- 0: neither side latched. Also 0 for the rest of the trial if the booster tips more than 30 degrees or drops more
  than 20 mm below the cushion top (off the seat).

Real trial: the same tiers, read from the green areas on both arms, the back gap with a ruler at both sides, and a
50 N pull on the seat (luggage scale, 2 s) that moves it less than 5 mm.

## Sizes and where they come from

- Bars 6 mm diameter: UN Regulation No. 145 definition of an ISOFIX low anchorage. 280 mm centre to centre: the usual
  ISOFIX/LATCH spacing (secondary source; ISO 13216-1 not read).
- Two installation steps, the ISOFIX adjustment button, the green area on both locking arms as the fitted check, and
  the backrest flat against the vehicle seat: Britax Römer KIDFIX i-SIZE operating manual (parts list and checks).
- ESTIMATES (no data sheet read): booster mass 6.5 kg and its sizes; arm travel 100 mm; connector 30 x 28 mm with an
  8 mm slot; catch plunger spring 40 N/m with a 43 degree ramp; button 60 x 25 mm, 8 mm stroke, 1,500 N/m; bracket slide
  3 N with the button held; guide sleeve 36 x 34 mm with a 6 mm lead-in; friction 0.4 between booster and seat.
- Real build of the stand-in: plywood box with upholstery fabric, two 6 mm steel rods across the bight, off-the-shelf
  plastic ISOFIX insertion guides.

## Retention check

`ISOFIX_PULLTEST=1` (never set in a scored run) makes the demo, once fitted, take both armrests and pull the booster
30 mm back toward the robot, then report how far it came. Run iso-pull3: the booster came 1.0 mm off and stayed latched and flush (score 1.0); both catches and the bracket
lock held.

## Fidelity

Rigid bodies, PhysX 240 Hz. The car seat is rigid: no foam to press into; the bars sit 20 mm into an open 45 mm bight.
Both locking arms are one bracket that slides as a whole; with the adjustment button out the bracket's slide limits
are closed to 0.3 mm around where it is (its ratchet), and while the button is held in (past 4 mm) it slides at 3 N. The connector release buttons are not modelled. Each catch is a sprung
plunger that the bar lifts on its ramp and that drops straight down behind it. The green/red window on the armrest
stands in for the green area on the arm, where the top camera sees it.
