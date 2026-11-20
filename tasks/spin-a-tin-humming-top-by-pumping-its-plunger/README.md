# Spin a tin humming top: stand it up, pump its plunger, and let go so it spins alone for ten seconds

Stand the top up by its knob, pump the knob down and up until the top spins fast, then let go and keep both grippers clear so it spins on its point by itself for ten seconds.

## Scoring

Judged from the trial video (60 frames per second or more) with a stopwatch. The white dot on the tin shows its turning.

- **Spun by the plunger:** at some moment the tin turns at least once per second, and it got there by the knob being pushed down its rod. Twisting or flicking the tin itself does not count.
- **Free-spin time t:** the longest unbroken stretch in which both grippers are at least 5 cm from every part of the top while the top turns on its point by itself: at least one turn per second, its tin rim off the tray, its point on the tray's flat deck at least 10 mm in from the tray's raised rim. The stretch ends the moment a gripper comes back within 5 cm, the rim touches the tray, the point leaves that area, the turning drops under one turn per second, or the video ends.
- **0:** never spun by the plunger (doing nothing scores 0: the top starts stopped and lying on its side), or the top left the table, or the rod was pulled out of its guide or bent.
- **0.25:** spun by the plunger, t under 2 s (it fell at once, was touched, or was never let go).
- **0.5:** 2 s <= t < 5 s.
- **0.75:** 5 s <= t < 10 s.
- **1.0:** t >= 10 s.

So exactly 2 s scores 0.5, exactly 5 s scores 0.75 and exactly 10 s scores 1.0. The number of strokes is free and is not scored. The run ends as soon as t reaches 10 s.

How `score()` reads each condition (top frame: origin at the lowest point of the plastic point, z up the rod):

- **Spun by the plunger:** spin about the top's own axis of at least 2 pi rad/s while the screw rule has passed more than 0.001 J into the body (`pump_top_kit.CLUTCH.work_out`; one turn per second holds 0.0043 J); spin from any other source leaves that at zero.
- **Grippers clear:** for each arm, the gripper body and both fingertips (their collision hulls at the current opening) are at least 5 cm from the top's bounding box.
- **On its point alone:** the lowest point of the tin rim (a circle of radius 63.5 mm, 63 mm up the axis) is more than 1.5 mm above the deck; the centre of the round point is within 6 mm of its resting height; in the tray's frame it lies within 165 mm by 125 mm of the tray's centre.
- **Left the table:** the top's point more than 5 cm below the table top. **Rod out of its guide:** any part of the rod more than 10 mm from where the guide puts it.
- t is the longest such stretch so far; the score follows the bands above and `success()` is score 1.0.

## Files

- task.json: instruction, setup (products, sizes, positions, estimates), scoring, run limit, reset and fidelity notes
- spin_a_tin_humming_top_by_pumping_its_plunger.py: scene, scorer and the scripted demonstration's waypoints
- pump_top_kit.py: the top (shell, point, rod, knob) and its one-way screw rule
- top_demo.py: the demonstration's pumping controller
- yam_parts.py, yam_joints.py, yam_mesh.py: kit modules, copied unchanged
- test_spin_a_tin_humming_top_by_pumping_its_plunger.py: CPU checks of the scorer (tier boundaries, the failure rules on leaning poses, shortcuts) and of the demonstration's reach and length

## Run it

```
inspect-robots run --embodiment yam_arena -E task=spin_a_tin_humming_top_by_pumping_its_plunger.py --policy yam_scripted   # scripted demonstration
inspect-robots run --embodiment yam_arena -E task=spin_a_tin_humming_top_by_pumping_its_plunger.py --policy dream           # model run
```

Run limit 1,200 control steps (20 Hz, 60 s). The scripted demonstration reaches ten seconds of free spin about 27.5 s in (550 steps). The start is fixed; `-E object_jitter=0.01` moves the top and the tray up to 10 mm.

## What is different

Unlike a pull-back toy car or a board-game spinner flick, the spin is built by repeated plunger strokes through a one-way screw while the same hand keeps a top-heavy body upright by its knob, and then the hand must leave it balanced on its point.

## Scene and real objects

Millimetres from the midpoint between the arm bases: x forward, y to the robot's left, heights above the table.

- **Top:** Schylling Little Tin Tops (MTA), a tin humming top 127 across with a listed height of 139.7. Tin body in red, blue, green, yellow and orange bands, red plastic point, blue collar, red knob on a twisted steel rod. It starts stopped with the knob pushed in, lying on its point and its rim (leaning about 44 degrees toward +y), the point at (120, 50). One white ink dot is added on the body so turns can be counted on video.
- **Tray:** IKEA TILLGANG (104.199.50), 370 x 290, gray, centred at (195, 50) with its long side along x. It gives the point a smooth, level deck and its rim keeps a fallen top on the table.
- **Arms:** both start at ready with open grippers. The demonstration uses the left arm only; the right arm stays at ready. Nothing else steadies the top: no collar, stand or fixture.

Published: the top's 127 mm diameter and 139.7 mm height, the tray's 370 x 290 mm. Everything else about the top is an estimate, none of it tuned to make a run pass: net mass 110 g (shell 99 g, rod and knob 8 g, bearing 3 g); shell inertia 1.3e-4 kg m2 across and 2.2e-4 kg m2 about its axis, centre of mass 70 mm above the point; knob 28 mm across with a 12 mm tall grip band, its top 140 mm above the point when pushed in; rod travel 80 mm; screw lead 25 mm per turn (from the twist count on a product photo of the extended rod); 14 N to drive the screw, screw efficiency 0.7; spin loss 2 rad/s per second; friction 0.25 between point and tray; tray deck 3 mm thick with a 12 mm rim.

**The point's radius matters most.** The point is modelled as a round button of radius 4.8 mm. That is an estimate from the product photo, where the point is about 26 pixels wide and the 127 mm body 342. Free-spin time depends strongly on it: in this simulation a 2.5 mm point let every top lean over within 7 s, whatever its spin, while the 4.8 mm point gives the times below. Check the real top before trials, a one-minute job: lay calipers across the round end of the point (it should read 9 to 10 mm), or hold a radius gauge of 4.5 to 5 mm against it. A top with a sharper point is a different toy for this task. The 14 N screw push is an estimate too: stand the top on a kitchen scale and push the knob down steadily; the reading should rise by roughly 1.4 kg.

## How it is simulated

- **Body:** a rigid tin shell of revolution made of ten convex coloured bands, standing on a sphere for the point. It is a free body: nothing holds it upright, fixes its point or adds spin. Gravity, the point's contact with the tray, the gyroscopic response and the fall are left to PhysX at 240 Hz.
- **Rod and knob:** the rod slides in the shell's axis on a bearing that turns freely in the shell, so a held knob does not brake the spin. The knob is a convex cone with a cylindrical band that the pads hold.
- **One-way screw (pump_top_kit.py):** before every physics substep, if the rod is moving down faster than the shell's spin allows (rod speed times 2 pi / lead above the spin), the shell gets a torque of at most 0.036 N m and the rod is pushed back up with that torque's 14 N; otherwise nothing acts, so lifting the knob freewheels. The forces are equal and opposite pairs inside the top, and the shell never receives more than 0.7 of the work the hand puts in. The screw catches only after the rod has come down 2 mm and above 10 mm/s. Spin loss is a small torque about the shell's own axis.
- **What the simulation gets wrong:** a simulated top let go upright leans past 20 degrees once it has slowed to roughly 11 to 12 turns per second (70 to 76 rad/s), earlier than a real top of this size, so free spin is shorter than on the real toy: about 14 s when let go at 103 rad/s. There is no sound, so "until it hums" is "until it spins fast". The first stroke from rest is weak in simulation (the demonstration's first stroke gives 10 to 25 rad/s where the later ones add up to 45), so about eight strokes are needed where a person needs four or five.

## Demonstration

One arm, as a person does it. The left gripper lines up with the leaning knob, pinches its band (17 mm commanded on the 28 mm knob), and swings the knob up on an arc about the point until the top stands. It raises the knob about 65 mm and pumps: each down stroke starts just faster than the body is already turning and speeds up with it, ends 4 mm above the stop and creeps the rest; each return is unloaded. The arm's joints follow their targets with a lag, so `top_demo.py` sends, for every control step, the target that makes the lagging joints follow a straight vertical line (planned joint angle plus a lead for speed, acceleration and the 14 N push). There is no sideways motion during or between strokes; at the bottom of a stroke the hand turns slowly so the top's axis stays upright. Pumping stops after eight strokes or once the spin read at the bottom passes 115 rad/s. Then the hand stands the top upright within about 0.7 degrees, opens, lifts 64 mm straight up (5 cm clear of the knob) and moves away. The controller reads the top's spin, lean and rod position from the simulation between strokes; a camera policy has to judge them by eye.

From the fixed start the demonstration's eight strokes bring the spin to 11, 50, 71, 83, 91, 99, 103 and 106 rad/s, and the top is let go leaning about 1 degree at 101 rad/s. In three runs with 10 mm jitter the eight strokes ended at 114, 117 and 106 rad/s, the top was let go leaning 0.2 to 0.9 degrees at 101 to 112 rad/s, and all three reached ten seconds alone. A single stroke from rest leaves the top near 11 rad/s, far too slow to stand on its point by itself: that run scores 0.25.
