# Set up a mini food processor so it will run

Instruction: `Put the food processor together so it is ready to run.`

A mini food processor (3-cup chopper) has been washed and left in pieces on the counter: the bowl sits loose on the
motor base, the S-blade stands beside it and the lid lies next to it. Like every food processor, it will not start
until all of it is locked: the bowl turned on the base until it snaps locked, the blade down on the bowl's centre post,
and the lid turned on the bowl until its tab pushes the safety-switch plunger in the bowl's handle home.

What makes it hard physically:
- **Brace and turn.** The base is light (0.95 kg) and the bowl's lock is stiff (up to 1.1 N m before it snaps), so
  turning the bowl by its handle drags the base round with it unless the other hand holds the base down. Cuisinart's own
  instructions for its Mini-Prep Plus say to turn the bowl counterclockwise to lock it and to support the machine with
  one hand.
- **Blade by the hub only.** The blade is picked up by its plastic hub and lowered over the centre post; its square
  bore clears the post by about 5 mm a side, so the hanging blade has to be lined up over the post before it drops on.
- **Lid on and home.** The lid goes on with its two bayonet lugs through the gaps between the rim's two ledges, then is
  turned 30 deg counterclockwise until the lugs are under the ledges and its tab pushes the plunger in.

## Scene

Metres; origin on the counter below the midpoint of the arm bases; x away from the robot, y to its left. Heights are
above the counter.

| object | type | size | material | position |
|---|---|---|---|---|
| motor base | free, 0.95 kg | 18 x 14 x 7.5 cm | white ABS housing on rubber feet | centred x 0.31, y 0 (front deck 4.4 cm in front of the bowl); tape marks on the counter at three corners (marks only, the base is not taped down) |
| work bowl | on the base, turns 0-30 deg, 0.20 kg | 12.4 cm inside, 13.2 cm outside, 9 cm deep, 4 mm wall; rim 16.9 cm up | clear plastic | axis x 0.33, y 0; handle (3 x 4 cm, 7.9 cm tall) points 110 deg clockwise from +x (right-front) when unlocked, 80 deg when locked |
| centre post | part of the bowl | 1.1 cm octagon, 4.5 cm tall | white plastic | on the bowl axis |
| rim ledges | part of the bowl | 2 ledges, 30 deg of arc each, 4.5 mm deep, 3.5 mm thick, 3 mm below the rim | clear plastic | inside the rim, 90 deg either side of the tab's home |
| switch plunger | slides 6 mm along the rim, 10 g | 20 x 8 x 6 mm | red plastic | on top of the handle; 15 N/m return spring |
| S-blade | free, 0.06 kg | hub 3.4 cm square, 6.3 cm tall, capped, 2.2 cm square bore; two steel blades 4 x 1.4 cm x 2 mm at its foot | black nylon hub, steel blades | standing at x 0.22, y -0.22 |
| lid | free, 0.11 kg | 13.6 cm disc, 6 mm; skirt 10.6 cm x 12 mm; two lugs 7 x 10 x 3 mm; tab 3 x 1.5 cm sticking out over the handle; grip bar 5 x 1.4 x 2.5 cm on top | clear plastic | lying at x 0.13, y -0.10, tab pointing 117 deg clockwise from +x |

Real product the rig follows: the Cuisinart Mini-Prep Plus DLC-2A (3-cup, 24 oz bowl; about 18 x 13 x 23.5 cm, 1.37 kg;
"turn counterclockwise to lock in place", then lock the cover). Our base is lower (7.5 cm) and the lock geometry,
angles and torques are our own estimates, not measured from that product.

## Real rig (low cost)

Use any mini chopper with a twist-lock bowl and lid and a lid-tab safety switch (e.g. the DLC-2A), unplugged. Tape the
base's corners on the counter (the marks only show where it belongs; don't tape the base down) and mark where the lid and
blade lie. For robot trials, cover the blade edges with tape or use a blunted spare blade; a touch on a covered blade
still counts as touching the blade.

Acceptance checks before the first trial (the simulated lock and switch values are estimates, so the real unit is
measured and must fall in these bands; record the measured values with the trial log):
- Lock direction and stops: seen from above, the bowl locks turning counterclockwise and stops 30 +/- 5 deg from
  where it sits loose; the lid locks counterclockwise too, about 30 deg from where its lugs drop into the rim gaps.
- Bowl-lock torque: a luggage scale hooked on the handle, pulled square to it, reads a peak of 1.1 N m +/- 50 %
  (about 13 N at the handle's middle, 8.6 cm from the bowl axis) just before the lock clicks, with the base held.
- Base: under 1.5 kg on rubber feet; turning the bowl to lock without holding the base drags the base round.
- Interlock: with the lid turned home the switch plunger is pushed in (about 4 mm or more, checked with a caliper on
  the first setup), and plugged in the motor runs; with the lid 10 deg short of home it does not run. This run test
  is the real-trial equivalent of the simulated 4 mm plunger threshold.

Cameras: besides the robot's own cameras, a phone on a small tripod at counter height, 40 cm to the side of the
processor, films the whole trial so any touch on the blade steel can be seen while the blade is carried and lowered. A second phone on the opposite side, also at counter height, covers the other blade edge, so both edges stay in view through pickup and insertion even when a gripper blocks one camera. Before the first trial, carry the blade through the demo path by hand and check that each edge is visible in at least one view throughout.

## Run a trial

Requires yam-isaac-lab-arena and Isaac Lab-Arena 0.3.1; `task.py` needs nothing else in this folder.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
  --instruction "Put the food processor together so it is ready to run."
```

Run limit 1,800 steps (90 s at 20 Hz). The layout is fixed (`object_jitter=0`); nothing is randomised.

## Scoring (score() in task.py)

Score 0 if, at any step, any part of either arm or gripper touches either steel blade (holding the plastic hub is
fine; in sim, PhysX contacts between every robot link and the blade body are read each physics substep and a contact
point on a steel blade, classified against the blade's pose at that same substep, counts); if at the end the base's centre is more than 2 cm from its start or it is turned more
than 10 deg; or if the blade or lid ends off the counter.

Otherwise: 0.25 bowl locked (turned within 2 deg of its 30 deg stop); 0.5 also blade seated (hub upright within 5 deg,
over the post within 5 mm, its bottom within 3 mm of the bowl floor); 1.0 also lid seated (centred within 1 cm, down
on the rim within 3 mm, level within 3 deg), turned home (within 4 deg) with the switch plunger pushed in at least
4 mm, and both grippers parked at the end: every gripper point at least 10 cm from the processor and moving slower
than 2 cm/s, held for 0.5 s.

In a real trial: the same touch rule, watched on the video for the whole run (any arm or gripper touching a steel
blade scores 0); afterwards a photo shows the base on its tape marks and the lid home, and a hand check confirms the
hub sits down on the bowl floor; plugged in afterwards, the machine runs only if the bowl and lid are locked.

## Notes

- The touch check was tested with scripted variants: fingertips pressing one steel blade, and fingertips pushing a
  steel blade so the whole blade slides about 7 cm across the counter (a moving blade); both latch the touch and score
  0. The demo, which holds the blade only by its hub, never triggers it.

- One arm can't do it well: turning the bowl without holding the base turns the base with it (in sim the bowl stalls
  at about 23 deg with the base turned 8-9 deg).
- The bowl on the base is a hinge from the start: in the real machine its lugs sit in the base's bayonet, so it can only
  turn; it is not lifted off in this task.
