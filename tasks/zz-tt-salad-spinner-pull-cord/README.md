# Spin washed lettuce dry in a pull-cord salad spinner

Instruction: `Spin the lettuce dry in the salad spinner.`

A pull-cord salad spinner stands on a kitchen counter, its basket holding washed romaine lettuce. The robot dries the
lettuce the way people do. One hand presses the lid, and with it the bowl, down on the counter. The other hand pulls
the cord's handle up, lets the cord rewind under control, and pulls again until the basket whirls past 300 rpm and
spins long enough to shed water. Then it lowers the handle back into its dock and lets go, and both hands clear the
spinner. Spinning greens dry is part of almost every salad made at home and in restaurant kitchens.

## What makes it hard, and how it differs from other tasks

- **Hold a free object down against its own reaction.** The spinner is not fixed. Each fast pull lifts the lid by the
  cord with up to about 20 N, while the whole spinner weighs under 1 kg. The basket's start-up torque also twists the
  lid. One hand has to press the lid down through every pull, without crushing or tipping the spinner, while the other
  hand works 15-25 cm away. If you pull without pressing the lid down, the lid rises off the rim and the spinner skids.
- **Pull fast, then rewind under control.** The one-way clutch only drives the basket while the cord comes out faster
  than the basket is already turning. A slow pull adds nothing, so speed builds over several fast pulls. Between pulls
  the hand must keep hold of the handle and let the spring take the cord back in. Letting go with the cord out makes
  the handle snap back into the lid.
- **Finish properly.** After spinning at speed for several seconds, the stem of the handle has to go back into its
  1.6 cm socket on the lid, and both grippers have to release and clear the spinner.

Nearest related tasks:
- "Spin a tin humming top: pump its plunger" pumps a plunger straight down on a toy that is free to spin.
- "Send a pull-back toy car" drags a car backwards on the floor to wind a spring.
- "Pull a retractable dog leash out against recoil tension" and "Rewind an extended tape measure" each pull out or
  rewind one cord once, to a length.

None of these holds a free container down with one hand while the other pulls a cord up out of it, again and again,
to build speed in a flywheel through a one-way clutch, and then docks the handle. Public-benchmark food-prep tasks
(RoboCasa `WashLettuce`, BEHAVIOR-1K `wash_lettuce` and `make_a_salad`) wash or place produce and never operate a
salad spinner; single-arm benchmark tasks such as Meta-World `pull-handle` pull a fixed handle once.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left). Heights are above the counter.

| object | real item | size / mass | sim | position |
|---|---|---|---|---|
| salad spinner | Zyliss Easy Spin 2, large (pull cord, brake button, BPA-free plastic); listed at 26 x 26 x 14.5 cm and 896-915 g | 26 cm across, 14.5 cm tall with lid | three rigid bodies (bowl, basket, lid) + handle, 0.94 kg in total | centred (30, -8) on the counter |
| bowl | the spinner's clear SAN bowl, non-slip rubber foot ring | 26 cm across, 12.5 cm tall, 0.42 kg | free rigid body, 32 clear wall panels, floor and foot ring | on the counter |
| basket | the spinner's green slotted basket, with a strip of red tape (1.2 x 8 cm) on its outside facing the front as a speed mark | 22.4 cm across, 10 cm tall, 0.19 kg | revolute joint on the bowl's centre pivot | inside the bowl |
| lettuce | six pieces of an artificial romaine leaf (soft PVC food replica), cut to ~8 x 6 cm and trimmed to 9 g each, wetted before each trial | 54 g in total | rigid leaf pieces with pale midribs carried against the basket wall | inside the basket |
| lid | the spinner's white lid with retractable cord housing and brake button | 26 cm across, 2 cm thick, 0.30 kg | free rigid body sitting on the rim; 1.2 cm cord hub (1.6 cm socket) at its centre | on the bowl |
| cord + handle | the spinner's 45 cm pull cord and green pull handle, left unfolded for use (not folded flat onto its storage clip) | modelled as an 8 x 1.6 cm T-bar on a 1.2 cm stem, 30 g | free rigid handle, cord forces modelled (below) | docked: stem in the hub socket, bar resting on the hub top (bar centre 16.5 cm above the counter), running front to back |

The two hands stay at least 14-15 cm apart. The pressing hand goes on the lid's far left part, about 11 cm from the
centre. The pulling hand grips the bar from behind (tilted 60 degrees forward-down so the arm can reach high) and lifts
it straight up to about 40 cm above the counter.

## How the spinner works (and how the sim models it)

The Easy Spin 2 is a pull-cord spinner with a brake button: the cord retracts after each pull while the basket keeps
spinning between pulls. Like a recoil starter, the cord is wound on a spring-loaded spool in the lid that drives the
basket through gears and a one-way clutch (ratchet). While the coil spring rewinds the cord, the clutch free-wheels so
the basket keeps turning and each new fast pull adds to its speed.

The sim models this every physics step (240 Hz). The cord is a straight line from the hub socket floor to the foot of
the handle's stem. Its tension pulls the handle toward the hub and the lid toward the handle:
- Rewind spring: 1.5 N docked, plus 2 N/m of cord drawn out.
- Clutch: while the cord is more than 4 mm out and paying out faster than 66 rad/m times the basket speed, it drives
  the basket (66 basket radians per metre of cord, about 4.7 turns per full 45 cm pull). The torque catches the basket
  up within 0.04 s and is capped at 0.33 N m. The matching force goes on the cord (up to about 22 N) and the reaction
  torque goes on the lid.
- Basket: moment of inertia 0.0016 kg m². Pivot friction is 0.001 N m and viscous drag is 0.0001 N m s, so from 300 rpm
  the basket coasts for about 20 s.

To match a real unit, count the tape-mark turns during one slow full pull and set the drive ratio from that.

## Scoring

Score 0 if, at any time during the trial:
- the bowl slides more than 3 cm, lifts more than 1 cm off the counter, or tilts more than 5 degrees;
- the lid rises more than 5 mm off the rim;
- the cord is more than 5 cm out while neither hand is on the lid. "On the lid" means a fingertip of the hand not
  holding the handle touches the lid top (within 1 cm of its top surface, inside the rim). Hovering higher does not
  count, and because each pull lifts the 0.30 kg lid with 15-22 N, hovering even a few mm above lets the lid rise past
  the 5 mm limit;
- the handle is let go with more than 3 cm of cord out (it snaps back). Holding the handle means a gripper's centre
  point (TCP) is within 4.5 cm of the bar centre with its finger pads closed on the 1.6 cm bar (pad gap 0.6 to 3.5 cm).

Otherwise, award the highest tier met:
- **0.3**: the basket reached at least 150 rpm.
- **0.7**: the basket reached at least 300 rpm, and spent at least 5 s in total at 200 rpm or more (a momentary peak
  that stops right away does not dry the leaves).
- **1.0**: as for 0.7, and at the end:
  - the handle is docked, with its stem seated in the hub socket (cord no more than 8 mm out, handle within 12 degrees
    of upright);
  - both grippers are at least 10 cm from the spinner, taken as a 13 cm-radius cylinder around the bowl's axis from
    the counter up to 20 cm (above the docked handle), measured from each gripper's fingertips and centre point;
  - both grippers have moved less than 2 cm/s for at least 1 s (20 control steps).

How to score a real trial:
- Speed and spin time: film the basket's red tape mark through the clear bowl from the side with a phone at 240 fps
  slow motion. 300 rpm is 5 turns/s, so the basket has reached 300 rpm if any turn takes 48 frames or fewer; 150 rpm
  is 96 frames/turn, and 200 rpm is 72 frames/turn. The 5 s requirement at 200 rpm or more is met when the turns taking
  72 frames or fewer sum to at least 1,200 frames (5.0 s).
- Zero rules: a tape outline around the bowl's foot on the counter checks the 3 cm slide limit; the video checks lift,
  tilt, lid rise, lid contact during pulls, and any handle snap-back.
- End state: docking, 10 cm gripper clearance (checked with a ruler against the bowl), and 1 s of stillness are checked
  at the end of the video.

## Fidelity

- Water is not modelled; drying is judged by reaching 300 rpm and spinning for at least 5 s at 200 rpm or more.
- The six leaf pieces ride fixed inside the slotted basket. The basket has no contacts of its own because it turns on a
  revolute pivot inside the closed bowl.
- The lid's moulded cord pocket and unfolded pull handle are simplified as a central 1.2 cm hub/socket and an
  8 x 1.6 cm T-bar on a 1.2 cm stem resting on the hub top (1.2 cm jaw clearance under the bar ends, matching an
  unfolded pull handle).
- The cord is drawn as a straight line between the socket and stem, with its rewind spring and one-way clutch forces
  applied every 240 Hz physics step.
- The lid is constrained to slide straight up off the rim (0-4 cm) so the scene stays intact after a failure; any rise
  past 5 mm already scores 0.
- Pulls draw about 23 cm of the 45 cm cord, matching the arm's vertical reach above the spinner.

## Run a trial

Requirements: yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1. No external assets are used; all geometry is built from
primitives in `task.py`.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \
  -E cam_height=176 --scorer task_score --max-steps 2400 --instruction "Spin the lettuce dry in the salad spinner."
```

Run limit: 2,400 steps (120 s at 20 Hz). The layout is fixed (`object_jitter=0`).

## Demo video

`demo/` holds a scripted run (score 1.0, about 20 s at 20 Hz) that plans motions from the object poses in the simulator
state. The left arm presses the lid down on its far left side while the right arm grips the bar at a 60-degree tilt,
makes three fast upward pulls of about 23 cm with controlled rewinds between pulls (building basket speed past 340 rpm
and keeping it above 200 rpm for over 7 s), seats the stem back in the hub socket, releases, and both arms park clear
of the spinner looking back at the docked end state.

## Shortcut checks

Scripted variants scored by `score()` verify that unintended strategies and incomplete runs stay below 1.0:

| variant | what it does | score |
|---|---|---|
| idle | neither arm moves | 0.0 |
| one-arm (no hold) | pulls the cord without pressing the lid down | 0.0 (no hand on lid; lid lifts off the rim) |
| weak pull | holds the lid and makes one slow, short pull (~100 rpm) | 0.0 (below 150 rpm) |
| let go | spins up, then opens the gripper at the top of a pull | 0.0 (handle released with >3 cm cord out) |
| single brief pull | holds the lid, makes one fast pull (~304 rpm, <5 s above 200 rpm), docks and parks | 0.3 (spin duration <5 s) |
| undocked end | spins past 300 rpm for >5 s above 200 rpm, but leaves the handle undocked | 0.7 |

## Reset

Press the brake button until the basket stops. Place the six freshly wetted replica leaf pieces against the basket wall
with the red tape mark facing forward, set the lid flat on the bowl rim, let the cord rewind fully, and seat the
unfolded handle's stem in its socket with the bar running front to back. Centre the bowl on its tape outline.

Wetting: dunk the six leaf pieces together in a bowl of cold water for 2 s, lift them out and let them drip for 3 s (about 1 g of water per piece), then load them. The spinner's bowl is closed, so spun-off water stays inside; empty it between trials and keep a dish towel under the bowl to catch drips during loading.
