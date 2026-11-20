# Box a donut from a self-serve bakery case with pastry tongs

At a supermarket or coffee-shop bakery counter, the self-serve case has a front door that lifts up and swings shut
under its own weight. Taking a donut means doing two things at once: one hand holds the door up, the other works a
pair of pastry tongs in through the opening, pinches one donut between its neighbours and draws it out. Then the door
is lowered by hand until it rests shut, rather than let go to bang against the frame, and the tongs go back where the
next person will find them. Nobody touches the food by hand.

Instruction: `Put one donut from the case into the box using the tongs, then lower the case door shut by hand and put the tongs back on their rest.`

## Why it matters

Self-serve bakery cases with drop-shut doors stand in supermarket bakeries, donut shops, coffee shops and convenience
stores; shoppers and counter staff serve themselves from them every day. The door is made to fall shut (makers sell
that as a feature, e.g. Choice's 2-tray case: "the door opens upwards and shuts with gravity", so it is never left
open), so the job
cannot be done one step at a time: the door has to be held for as long as the tongs are inside. The tong pick is a
real tool skill: the robot squeezes the tongs' arms, not the donut, and the flat blades have to go into the 4 cm gaps
either side of the donut without pushing its neighbours.

## Scene (sim and real)

Counter top = reference; x forward from the line through the arm bases, y left of their midpoint (cm).

- **Case:** clear acrylic two-tier countertop bakery case the size of the Cal-Mil 921 (18-1/2 x 14 x 10 in: 47 wide,
  35.6 deep, 25.4 tall), front face at x 40, centred on y 0, taped down. Its front door is a 44 x 21.2 cm, 3 mm
  acrylic panel (0.33 kg) hanging from a hinge along the case's top front edge, 24.2 cm up; its bottom edge is 3 cm up,
  with a small pull (2.5 cm out, 2 cm tall, 1.2 cm wide) at y +15. The door opens upward to a stop at 88 deg, so it
  never stays open on its own: let go, it swings shut.
- **Donuts:** six glazed ring donuts (in real trials, display replicas weighted to a real donut's 55 g; see
  materials), 9.5 cm across, 3.5 cm tall, 55 g, in two rows of three on the lower tray (top
  2.5 cm up), 4 cm apart: rows at x 47.2 and 60.7, columns at y -13.5, 0, +13.5. The upper tier is empty.
- **Tongs:** 9-1/4 in (23.5 cm) flat stainless pastry tongs, 108 g, sprung open (tips 12.5 cm apart), standing open on
  their edges on a tong rest to the right of the case: a 28 x 14 cm board centred at x 30, y -34, turned so the tongs
  point forward-right at 45 deg, with two 5 cm tall wooden bars across it. Pin end at x 21.9, y -25.9.
- **Box:** an open white single-donut box, 15.5 cm square inside with 6.5 cm walls, lid standing open at the back,
  centred at x 50, y -52.

The door hangs from the case's top edge, so the tongs can only go in underneath it while it is held up; nothing
props it. Each donut sits 4 cm from its neighbours, so the tongs' 3 mm paddles have about 2 cm of room on each side.

## Scoring

Judged from the trial video and an end photo (the sim computes the same from object states). Film each trial with
a fixed side camera at tray height, beside the case on the right and looking across the case front, so both tong
paddles and the donut stay in view from the pinch, through the carry, to the box; an overhead camera shows the end
state. (In the sim the same view comes from the wrist camera of the hand holding the tongs, which looks down the
tong arms at both paddles and the donut the whole way.) Contact in the video means visibly touching, with no gap: a hand is on the door when a finger or pad presses
the door or its pull, and a donut is pinched when both paddles press its rim from opposite sides. The sim's allowances
(a gripper point within 1.5 cm of the door or pull; both paddles within 6 mm of the donut) are the same calls made
from tracked points: the sim's hand points sit inside the fingers (fingertips, grasp centre), so 1.5 cm is about a
fingertip's half-width plus a few mm, i.e. a finger on the door; the paddles are checked by their bounding boxes, and
6 mm is about two paddle thicknesses. A gap bigger than these shows plainly in the side camera and fails.

Score 0 if:
- a gripper touches any donut (hands off the food; in the sim, a gripper point within 6 mm of a donut);
- a donut is carried without being pinched: at any moment a donut is off the tray (lifted 1 cm or more) and not yet
  over or in the box, but not held between both tong paddles (scooped on one paddle, flicked or pushed up; in the
  sim, checked every step: both paddles within 6 mm of the donut and on opposite sides of its centre). Let go over the box, it may drop in;
- a donut touches the counter, or ends anywhere but on the tray or in the box;
- the door is let go while it stands open: at any moment the door is more than 10 deg open (its bottom edge about 4 cm
  or more out from the frame) with no hand on it (in the sim, no gripper point within 1.5 cm of the door panel or its
  pull). The door can't stay open by itself, so a door let go that far open falls and bangs; the hand has to stay on it
  from lifting it until it rests shut. In the trial video this is a yes/no: does the hand leave the door before it is
  down?

Otherwise:
- 0.25: a donut was lifted 1 cm clear of the tray, pinched between both tong paddles;
- 0.5: a donut carried that way (lifted pinched in the tongs) is in the box;
- 1.0: exactly one donut in the box, carried there pinched in the tongs, the other five still resting on the tray within 1.5 cm of their spots (centre
  within 1.5 cm, underside on the tray), the door shut (within 2 deg), the tongs lying across the bars of their rest
  (the middle of each arm and of each paddle over the board, each part's underside at bar-top height), and both arms
  parked: the tongs let go and, over the last 0.5 s, every gripper point 10 cm or more from the tongs, the case (door
  and pull included) and the box (walls, open lid), moving under 2 cm/s. In the video: at the end both hands are
  still, clearly apart from the tongs, case and box, and the tongs lie on the rest by themselves.

## Real-world materials

- Case: Cal-Mil 921 Classic two-tier acrylic display case with front door (18-1/2 x 14 x 10 in; its front door lifts
  up on a top hinge, with a pull fixed to the door). The door must fall shut by itself: if it can swing past upright and stay open, glue
  a 1 cm foam bumper on the case top where the open door meets it, so it stops at about 88 deg. Tape the case down at
  its marks (the sim case is fixed).
- Donuts: six resin or foam display-replica glazed donuts, 9.5 cm across, 3.5 cm tall, each weighted to 55 g with a
  steel washer glued underneath. Replicas keep the trials repeatable (no crumbs, no drying out). Mark each tray spot
  with a 1 cm dot under its centre.
- Tongs: 9-1/4 in flat stainless pastry tongs whose tips spring about 12.5 cm apart (for example the Vollrath 47107
  Tender-Touch flat pastry tong; check the opening). Spring check: squeezed 12 cm from the pin (mid-arm), the tongs
  should start to close at about 5 N (4-8 N; press them onto a kitchen scale), rising to about 6.5 N with the tips 9 cm
  apart; the sim's torsion spring is 0.6 N m at the open stop plus 0.02 N m per degree closed. Lying open on their
  edges, they must rest across both bars of the tong rest.
- Tong rest: a 28 x 14 cm board with two 1.5 cm wide, 5 cm tall wooden bars glued across it, 16 cm apart (5 cm and
  15 cm long), so the tongs stand open on their edges across both bars. Tape it down at its marks.
- Box: a white single-donut bakery box about 16 cm square inside (6-1/4 x 6-1/4 in), 6.5 cm deep, lid open.

## Running

Requirements: yam-isaac-lab-arena and Isaac Lab-Arena 0.3.1.

Run a trial:

```
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \
  -E cam_height=176 --scorer task_score --max-steps 1600 \
  --instruction "Put one donut from the case into the box using the tongs, then lower the case door shut by hand and put the tongs back on their rest."
```

The layout is fixed (object_jitter=0). A scripted demo (not part of the task files) reaches 1.0.
