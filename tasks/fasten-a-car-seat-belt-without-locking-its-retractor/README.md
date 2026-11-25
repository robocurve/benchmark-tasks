# Fasten the seat belt around a seated adult surrogate

An adult-sized surrogate sits in a complete car front seat beside a three-point belt. The robot draws the tongue
past the body and inserts it into the buckle on the other side of the seat. The belt crosses the surrogate's chest
and runs low over its hips and upper thighs. A rapid pull can lock the emergency-locking retractor. [NHTSA belt fit guidance][fit]

The gripper has to lift the tongue over the thighs while keeping it in front of the torso, then align its 3 mm blade
with the buckle's 4.5 mm slot in the narrow space beside the inboard thigh. The 48 mm webbing pushes against the
occupant's clothes along this path. Tension draws the tongue back toward the pillar and up from the buckle. Once the
pawl catches, the gripper lets go.

## Hardware and start positions

Positions are in centimetres from the midpoint between the YAM bases. Positive x goes from the bases toward the
seat back, positive y goes toward the outboard pillar, and z is measured above the tabletop.

- The seat has a full 49 × 45 cm cloth-covered foam cushion on a steel frame, with its top at z 4.5 cm. The cushion spans x 7 to 56 cm and y −53 to
  −8 cm. Its two 2 cm side bolsters run the full length. The 9 cm-thick backrest starts at x 56 cm and rises above
  the cushion, with a head restraint on two posts. The seat and its frame are fixed.
- The surrogate is seated against the backrest, centred at y −30.5 cm. The rounded chest is 21.6 cm deep,
  37.6 cm wide and 46.8 cm tall. Its centre is (45.5, −30.5, 41.9). The pelvis is 25 × 35.4 × 15.6 cm
  and centred at (43.1, −30.5, 12.3). Each thigh is 38 × 15.8 × 14.8 cm, centred at x 26.2 and z 11.9, with
  the two thigh centres separated by 16.8 cm along y. Their knees are above the cushion front. Rounded shoulders,
  each 14.6 × 10.2 × 22.4 cm, are centred at x 46, z 49.7 and 14.5 cm to either side of the centreline.
  The neck is 12 × 11.2 × 16.2 cm and the head is 18.8 × 15 × 22.4 cm. Their centres are (47.7, −30.5, 63.9)
  and (48.5, −30.5, 74.8). The body forms are firm foam with cloth over the chest, shoulders, hips and thighs.
- The outboard pillar (steel with 14 × 6 × 68 cm plastic trim) carries a shoulder guide at (42.5, −7, 62) and
  a lap anchor at (32, −7, 4.5). A 30 × 5 × 4 cm sill supports the lower anchor. Bolt the retractor upright next
  to the pillar. The webbing is 48 mm polyester seat-belt webbing.
- The 40 g latch plate has a 50 × 13 × 28 mm plastic head and a 22 × 3 × 24 mm steel blade with a 12 × 11 mm
  window. A sewn stop holds the tongue on the webbing with its slot about 15 cm above the table and the blade
  down. A visible ink mark near the blade edge is 10 mm from its tip.
- The buckle on the inboard flank is centred at (31.1, −55). Its metal stalk rises from z 2.0. The plastic
  shell is 53 × 32 × 50 mm, and its steel latch is reached through a 26 × 4.5 mm slot with a 6 mm-deep funnel.
  The mouth is 9.5 cm above the table, just beyond the inboard cushion edge.
- Both arms start at their ready pose with grippers open. The left arm stays clear in the demonstration.

For a real fixture use a matching used car seat, retractor, tongue, buckle and stalk. Bolt them to a baseboard in the
positions above. Fix an adult-sized clothed foam surrogate in the seat using the rounded chest, pelvis and thigh
dimensions above. A rigid adult training torso padded with upholstered foam can serve as the torso. Use firm
cloth-covered foam pieces for the pelvis and thighs and secure them to the torso. Keep the inboard thigh and bolster
in their specified positions because they constrain buckle access. The surrogate represents a seated body for
handling trials on a bench. Park the tongue on the outboard stop before every trial, and let the retractor return it
slowly when resetting.

## Simulation

Run with [yam-isaac-lab-arena][yam] at commit `0059d02`, Isaac Lab-Arena 0.3.1 and Isaac Sim 6.1.
`bodykit.py` builds the seat, fixed surrogate, guide, loose tongue and the sprung physical buckle.
`routekit.py` builds the rounded dummy surfaces and solves taut shoulder and lap ribbon paths in the planes of
their respective anchor-to-tongue chords. The shoulder section wraps in front of the torso and the lap section
goes over the upper thighs. At 240 Hz the lengths of the routed paths determine payout. The resulting tension
pulls on the tongue slot along both end tangents, so routing around the occupant changes the force on the gripper.
The visible ribbons follow those same paths. Only the tongue and buckle are manipulated. The model represents taut
webbing and fixed upholstered body geometry.

Each strand carries a 2 N preload and an additional 2.2 N/m of total payout. The webbing's force and angle change
as it wraps across clothes. An outward payout acceleration over 0.7 g locks the modeled reel, increasing tension
with a stiffness of 1,250 N/m until 5 mm of webbing has returned. This is the bench model's calibrated threshold.
FMVSS 209 specifies 1–7 N retraction force at zero acceleration for this belt type and separate retractor tests
at 0.3 g and 0.7 g. [FMVSS 209][fmvss]

The inserted blade physically pushes aside the buckle's sprung pawl, which returns into the window. The composed
forces continue to pull on the tongue after the gripper opens. The release button is depicted for identification.

## Score a trial

The scorer is `task_score`. Scoring is terminal, with partial credit for insertion and latching.

- **0.25** if the blade reached 10 mm into the slot and was within 6 mm fore-aft and 4 mm laterally of its centre.
  Read the depth mark and alignment from the close view.
- **0.4** for a latched tongue when the trial ends. The pawl returns within 0.8 mm of its rest position while the
  head sits within 3.5 mm of the mouth. On the real fixture, verify the tongue remains caught with a gentle tug.
- **0.6** if the tongue is latched, the fingers are 20 mm or more clear, the tongue stays within 1 mm of its final
  position during the last second, and the belt is correctly routed, when the reel locked during the attempt.
- **1.0** for the same released, correctly routed latch held for one second without locking the reel.
- **0** if the blade never reached the insertion milestone.

Correct routing puts the shoulder ribbon across the front of the chest and the lower ribbon across both upper
thighs. The gap to the cloth must be at most 12 mm along at least 5 cm of the chest run and 2.5 cm over each thigh.
Use the front view and a side view of the released belt and compare the gaps against a ruler next to the surrogate.
A retractor lock is a click and a visible interruption of outward payout while the gripper is pulling. An
incomplete or incorrectly routed latch can receive 0.4.

## Run the simulation

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fasten_a_car_seat_belt_without_locking_its_retractor.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the seat belt around the seated surrogate." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/belt-dream
```

Dream-1 scored 0.0 at the 2,400-step limit. It took about 320 s of wall time on an RTX PRO 6000 Blackwell.

The scripted demo picks up the tongue from the pillar and lifts it in front of the chest and above both thighs.
It brings the tongue around the inboard leg and reorients the blade before insertion. Its final move releases and
backs away from the buckle. `BELT_WIDE_CAMERA=1` selects a wide external view that shows the full seat and
surrogate. The demonstrated latch held with 21.1 mm insertion depth, about 90 mm of gripper clearance, and 0.01 mm
of movement over the final second. It scored 1.0 in 459 control steps (22.95 s of simulated time). The debug run
took around 103 s of wall time on an RTX PRO 6000 Blackwell.

```bash
BELT_WIDE_CAMERA=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fasten_a_car_seat_belt_without_locking_its_retractor.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the seat belt around the seated surrogate." \
  --scorer task_score --max-steps 560 --store-frames --log-dir ~/yam-eval/belt-demo
```

`BELT_DEBUG=1` reports the payout, rate, acceleration, retractor locks, tongue depth, latch and routing checks.
`BELT_JERK=1` tests an abrupt lateral draw after lifting the tongue clear of the thighs. The jerk reaches
1.05 g outward acceleration, locks the reel, and pulls the tongue out of the hand. It scores 0. The fixture has a fixed layout with
`object_jitter=0`. A seed is optional. All geometry is generated in the task and its helpers.

[fit]: https://www.nhtsa.gov/vehicle-safety/seat-belts
[fmvss]: https://www.ecfr.gov/current/title-49/subtitle-B/chapter-V/part-571/subpart-B/section-571.209
[yam]: https://github.com/kairosiann/yam-isaac-lab-arena
