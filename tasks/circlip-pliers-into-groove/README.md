# circlip-pliers-into-groove

Fit an external retaining ring (circlip) into its groove on a loose shaft assembly with circlip pliers: steady the
shaft, spread the ring just enough to clear the shaft, slide it down to the groove, let it close, turn it in the
groove to check it is seated, and put the pliers down. A ring spread past its elastic limit is scrap; one let go short
of the groove or tilted is not seated.

Category: Impact on jobs. Industrial machinery mechanics, machinery maintenance workers and millwrights held about
547,300 US jobs in 2025 (BLS Occupational Outlook Handbook). Repairing or replacing machine parts is a core duty, and
refitting the retaining ring that keeps a gear or bearing on its shaft is part of that work.

## Run

From the repository root, on a yam_arena install (Isaac Lab-Arena, inspect-robots):

    # scripted demo (scores 1.0)
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/circlip-pliers-into-groove/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fit the retaining ring into the groove on the shaft with the circlip pliers, then put the pliers down." \
      --scorer task_score --max-steps 1500 --log-dir ~/yam-eval
    # Dream-1: the same with --policy dream --max-steps 2400

Offline scorer tests (no simulator): `python test_task.py` prints "all pass".

- Runtime: the demo finishes in 583 control steps (29 s at 20 Hz); a 2,400-step run (2 minutes at 20 Hz) takes about
  10 minutes of wall time on one L40S. No seeds: the layout is fixed (`object_jitter=0`); the demo is deterministic.
- Files: `task.py` (scene, scorer, demo), `ringkit.py` (the ring-on-a-shaft model and its drag on the shaft),
  `yamkit.py` (modeled props, mechanisms and gripper states), `guardkit.py` (hand contact and a debounced latch). The
  kits hook yam_arena's scene builder in memory; nothing on disk changes.

## Spec

Frame: cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0.

| Object | Description | Position |
|---|---|---|
| Shaft assembly | loose, standing on end on the bench, not clamped; about 1.6 kg | axis (36, -6) |
| Shaft | ground steel, 40 mm, 14.9 cm long; 1 mm chamfer at the lower end, 1.5 mm at the top end face | z 0 to 14.9 |
| Shoulder | 46 mm, on the shaft; plain 40 mm shaft below it, where a hand holds it | z 8.8 to 9.4 |
| Gear | steel spur gear, 56 mm, 30 teeth, 1.6 cm thick, sitting on the shoulder | z 9.4 to 11.0 |
| Groove | 1.85 mm wide, 37.5 mm bottom diameter (DIN 471 for 40 mm); lower wall flush with the gear face | z 11.0 to 11.19 |
| Ring | DIN 471 A40 external retaining ring, spring steel, yellow zinc-chromate plated, 1.75 mm thick, free bore 36.5 mm, lug holes 12 mm apart | flat on the shaft's end face, lugs toward the robot |
| Pliers | external circlip pliers with 90-degree bent tips, 13 cm long, red handle grips 8 x 1.0 x 1.2 cm, handles 4.0 cm apart at rest, tips 1.7 cm long, 12 mm apart at rest; squeezing the handles 1 mm spreads the tips 1 mm (handle stop at 18 mm); 130 g | lying on the rest, grip point (26.1, -22, 3.6), tips toward +x |
| Rest | black rubber block, 8 x 3 x 3 cm | center (25.6, -22, 1.5) |

The ring's bore grows by (lug spacing - 12 mm) / pi: it clears the shaft from 40.2 mm (lug spacing 23.6 mm) and
reaches its elastic limit at 41.0 mm (26.1 mm).

Why the shaft is loose: a 40 mm shaft with a gear on it weighs about 1.6 kg and is fitted by hand on the bench, not
in a vise. The fitter stands it on end and steadies it with one hand while the other works the pliers. In the scene
nothing fixes it to the bench: it can slide, turn and tip over like the real part, and the arms and pliers are real
colliding bodies that can push it. The demo's left hand holds it low on its plain part from the side while the right
hand fits the ring, then lets go. Any other safe way of steadying it scores the same.

What the ring pushes on the shaft (estimates, with their basis):
- Closed in its groove, the ring grips the groove bottom with about 270 N all round. Basis: the ring maker's
  detaching speed for this ring (Seeger A40, 14,000 rpm) is the speed at which its 6.03 g, spinning at about 21 mm
  radius, just lifts off the groove bottom, so its grip equals m * w^2 * r = 0.00603 kg * (1466 rad/s)^2 * 0.021 m
  = 272 N. On the 40 mm land it would be 3.5 times that, in proportion to its stretch.
- Turning it in the groove takes mu * 270 N * 18.75 mm = 0.61 N·m with mu 0.12 (lightly oiled steel on steel; dry
  it would be higher): 25 N at the lug holes. The ring passes that torque to the shaft until it slips.
- A loose shaft standing on its 38 mm end face resists turning with about (2/3) * mu * 1.6 kg * 9.81 m/s^2 * 19 mm
  = 0.04 to 0.10 N·m (mu 0.2 to 0.5 on a wood or steel bench). The ring's 0.61 N·m is 6 to 15 times that, so a
  shaft nobody holds turns with the pliers and the ring does not turn on it. In the sim the shaft's footing gave way
  at about 0.27 N·m (11 N at the lug holes).
- Spread clear of the shaft to slide it along (0.2 mm clearance), the ring touches nothing and pushes nothing. Its
  spread force acts between the ring and the pliers' tips only.

In the sim: the demo with the left hand on the shaft scores 1.0; the shaft turns less than 1 degree while the ring
slips 25 degrees net in its groove. The same demo with the left hand left at home scores 0.5: the shaft turns with
the pliers (up to 17 degrees each way), the ring turns 0.5 degrees on it, so it is seated but not checked.

## Scoring

`score()` in task.py, evaluated at the end of the run. Two damage checks are kept over the whole run; the rest is the
end state. Doing nothing scores 0.

| Outcome | Score | Why it is worth that |
|---|---|---|
| Ring ever spread past 41.0 mm bore, or its ends twisted more than 10 degrees out of plane for 2 control steps in a row | 0 | the ring has taken a set or is bent: it is scrap, whatever happened next |
| Ring still on the end face, or less than 5 mm onto the shaft | 0 | no progress (this is where it starts) |
| Ring on the shaft (mid-plane 5 mm or more past the end face) but not seated: short of the groove, cocked, or held open by the pliers | 0.25 | spread over the shaft end within its limit, the controlled-spread part of the job; it can be spread again and moved |
| Seated: in the groove, closed onto its 37.5 mm bottom, not cocked (the pliers' tips may still be in the lug holes) | 0.5 | the ring does its job |
| Seated and turned at least 15 degrees net in its groove | 0.75 | also checked free all round, as a fitter does |
| All that, the pliers put down, and the shaft standing on its end on the bench | 1.0 | the job is finished and left safe |

Thresholds and reasons:
- 41.0 mm bore: the usual maximum installation spread for a 40 mm DIN 471 ring (1 mm, 2.5%, over the shaft), past
  which spring steel takes a set and sits loose in the groove, where it can walk out under load. The sim computes the
  set from the spread. A real trial measures it directly (see below).
- 10 degrees of twist (2.1 mm across the 12 mm lug spacing), for 2 control steps in a row so a one-step physics jolt of
  the pliers does not count. The sim treats this twist as a bent ring, a modeled stand-in; a real trial checks the
  removed ring for flatness.
- 5 mm onto the shaft: the ring's mid-plane past the 1.5 mm chamfer with room to spare, so it is around the 40 mm land
  and not perched on the end.
- 0.2 mm diametral clearance to slide over the shaft.
- 4 degrees of tilt at release: a 49 mm-wide ring's far side then sits 3.4 mm off the groove plane, nearly twice the
  1.85 mm groove width, so it closes onto the shaft beside the groove (cocked), however slowly it is let close.
- 15 degrees of net turn in the groove (5 mm at the ring's rim): the check that it is free all round. Net is the span
  between the smallest and largest angle the ring reached while closed in the groove, so a back-and-forth wiggle or
  jitter counts only its width. The turn is the ring's turn on the shaft: a shaft that turns with the pliers gives
  no check, so the shaft has to be kept from turning.
- Put down: the pliers out of the ring, no gripper touching them (contact with the hand's own palm and finger shape,
  within 2 mm), lying still with the grip point from 1 cm below to 5 cm above the bench top and over its footprint.
  That is on the bench or the 3 cm rest, not on the floor and not left lying across the shaft. The job is not finished
  while the tool is in hand or hanging off the work.
- Shaft standing: its axis within 10 degrees of vertical and its lower end within 1 cm of the bench top, over the
  bench. Standing on its flat end it is within a degree of vertical; knocked over it lies at 90 degrees, and a loose
  1.6 kg part lying on its side can roll off the bench.
- Not scored: how the shaft is steadied (one hand, the other hand between steps, or not at all) and where the arms end.

## Real-world build

All parts are off the shelf: a 40 mm ground steel shaft about 15 cm long with a shoulder, turned with a DIN 471 groove
1.85 mm wide to 37.5 mm (or a 40 mm shaft stub with a 40 mm-bore collar standing in for the shoulder), a 40 mm-bore
spur gear, a DIN 471 A40 ring, a pair of 90-degree external circlip pliers for 19 to 60 mm rings, and a rubber block.
The shaft is not clamped: stand it on its end on the bench at a marked spot. Check that the gripper can squeeze the
pliers' handles through their working range (handle gap 40 mm to 27 mm) and fit stiffer grips if needed.

A real trial is checked from the top video and by hand:
- the largest lug-hole spacing during the trial, against a mm scale on the bench (26.1 mm = 41.0 mm bore);
- the ring's free bore after it is taken off, with calipers (over 36.6 mm means it took a set: 0);
- the removed ring laid on a flat plate (a 0.1 mm feeler gauge sliding under any part means it is bent: 0);
- for a ring left short of the groove, its distance from the end face with a ruler (5 mm or more is on the shaft);
- the ring fully in the groove all round, with a straightedge on the gear face at four points (no part of it above
  the groove's upper edge), and closed on the groove bottom (it turns but does not rock);
- a paint mark on the ring turned at least 15 degrees (5 mm at the rim) against a mark on the shaft;
- the pliers lying on the bench or rest with no hand on them, and the shaft standing on its end on the bench.

What the sim leaves out: the ring has no contact model. Its grip and turning drag on the shaft are computed (above)
and applied to the shaft as a torque about its axis; the pliers do not feel that reaction (the hand holding them is
taken as firm), and a sideways push of the ring on the shaft is not modeled. A real trial shows the same thing
directly: a paint mark on the shaft against one on the bench shows whether the shaft turned instead of the ring.

Reset: take the ring off with the pliers (spread no more than 41 mm), check its free bore with calipers and that it
lies flat on a plate, and replace it if the bore measures over 36.6 mm or a 0.1 mm feeler gauge slides under it. Lay
it on the shaft's end face with the lugs toward the robot, stand the shaft assembly on end at its marked spot, and put
the pliers back on the rest.
