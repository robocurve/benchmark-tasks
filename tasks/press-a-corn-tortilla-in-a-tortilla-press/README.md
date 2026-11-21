# Press a corn tortilla in a cast-iron tortilla press, at least 14 cm across, then open the press

An 8-inch cast-iron tortilla press (two round 20 cm plates; the top plate hinged on one side, a long pressing lever
pivoted on the other, like the Victoria 8-inch tortilla press, model TOR-003) stands open on the table, its plates
lined with two squares of plastic cut from a freezer bag. A 35 g ball of corn masa sits on a small plate beside it.
The instruction asks for the ball to be pressed into a corn tortilla at least 14 cm across, and the press opened
again.

Pressing corn tortillas is an everyday home-cooking job across Mexico and Central America, and in many homes and
taquerias elsewhere; a press is how most people make them by hand. WebstaurantStore's guide "How to Use a Tortilla
Press" ([WebstaurantStore][guide]): "Place a ball of tortilla dough in the center of the press over the paper." "Lower
the top plate of your tortilla press onto the dough." "Press down on the handle of the tortilla press." "Lift the top
plate" and peel the tortilla off (it also turns the tortilla and presses it a second time; the task asks for one press
to full size). Plastic cut from a freezer bag is the usual lining at home. A ball put down off-centre comes out
lopsided and is squeezed out past the plates' edge; a light press leaves a thick, small tortilla.

[guide]: https://www.webstaurantstore.com/blog/4514/how-to-use-a-tortilla-press.html

## The hand skill

- The ball (4 cm, soft, 35 g) is set down in the middle of the bottom plate. Off by a few centimetres, the pressed
  tortilla runs past the plates' edge.
- The top plate (1.2 kg cast iron, standing open past upright on its hinge) is taken by its pressing boss and lowered
  onto the ball.
- The lever (standing open past upright on its stop) is swung over and laid on the top plate's boss, and its end,
  28.8 cm from its pivot, is pushed down firmly through about 4 cm, to a force of about 20 N.
- Then the lever is lifted back over onto its stop, and the top plate is lifted back over onto its hinge stop.

Here one arm handles the ball, the top plate and the press, the other the lever: the lever's pivot is on the right of
the press and its end on the left. A person uses one hand for most of it; the task does not claim that two hands are
needed.

The task is the task file
[`press_a_corn_tortilla_in_a_tortilla_press.py`](press_a_corn_tortilla_in_a_tortilla_press.py) for the `yam_arena`
embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the press, the masa and the plate are built in these files from boxes, cylinders and spheres.

`yam_arena` task files place single boxes, cylinders and spheres and cannot put one on a joint. The press (a fixed base
with the top plate on a revolute joint) and the lever (a fixed bracket with the lever on a revolute joint; a separate
articulation, so that it collides with the top plate's boss) are built with `bodykit.py`, which adds such bodies and
joints: importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives the task
file each joint's position. This copy of `bodykit.py` starts a joint away from its built pose (`init`): the press is
built closed and starts open. Every control step the task file sets the top plate's drive (the masa's resistance, as
described below) through Isaac Lab's articulation API, and places the drawn tortilla.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Tortilla press, fixed (3.9 kg, on rubber feet): bottom plate 20 cm across and 1.2 cm thick, centred at x 22, y 0,
  its top 3.0 cm above the table. Top plate 20 cm across, 1.2 cm thick, 1.2 kg, hinged along x at y 11.5, 3.6 cm
  above the table, with a round pressing boss 2.4 cm across and 4.0 cm long (along x) at its centre. Plastic sheets,
  17 x 17 cm, on both plates. At the start the top plate is swung open 100 deg, resting on its hinge stop.
- Lever, 0.45 kg, iron bar 2.2 cm wide and 1.4 cm deep, pivoted along x at y -11.8, 7.4 cm above the table, 28.8 cm
  from its pivot to its rounded end. Laid over the closed press it bears on the boss 11.8 cm from its pivot (2.4 : 1
  at its end) and its end is at y 17. At the start it stands open, swung up 95 deg onto its stop.
- Corn masa ball, 4.0 cm across, 35 g, on a white plate 11 cm across and 1.2 cm tall at x 12, y -21.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial, as in the screenshot.

What is modelled, and how:

- The top plate and the lever turn on their hinges against light damping, and rest on their stops when open.
- The masa (simulators cannot model dough): once the top plate comes down onto the ball lying between the plates, the
  ball becomes a disc of the same volume (33.5 cm3) under the plate, centred where the ball lay, and is drawn as a thin
  disc on the bottom plate. Its thickness h only goes down. The dough pushes back on the top plate as a stiff spring
  (30 N/mm at the dough's centre) on the plate's hinge, and it yields (gets thinner) while that force exceeds
  F(h) = 60 N x 2.18 mm / h: 60 N presses it to 2.18 mm, which is 14 cm across. The diameter is
  d = sqrt(4 V / (pi h)). The top plate's own weight (with the lever's) takes the ball to about 7.5 cm across; the
  lever pushed down at its end takes it past 14 cm. The force comes from real contact: the arm pushes the lever, the
  lever's underside pushes the boss, the plate's hinge drive carries the dough's spring.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the top plate's and the lever's angles and the ball's position, and keeps the
tortilla's centre and diameter.

- 0.25: the masa has been pressed between the plates (the top plate came down onto the ball lying on the bottom plate).
- 0.5: the tortilla is 10 cm across or more.
- 0.75: 14 cm across or more.
- 1.0 (success, held 1 s): 14 cm across or more, and the press open again: the lever and the top plate both swung back
  90 deg or more (past upright, onto their stops), so the tortilla can be taken off.
- 0 for the whole run once the tortilla runs past the plates' edge: its centre's distance from the plates' centre plus
  its radius over 10 cm (a ball put down off-centre).

Doing nothing scores 0. A real trial is scored from the end of the video and a ruler laid across the tortilla on the
open bottom plate (its smallest width, and whether any of it was squeezed out past the plates' edge).

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=press_a_corn_tortilla_in_a_tortilla_press.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Press the masa ball into a corn tortilla at least 14 cm across, then open the press." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tortilla-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The right gripper, its fingers closing
along y so that its body stays clear of the standing lever, takes the ball off its plate and sets it down in the
middle of the bottom plate, then goes back to its ready pose. The left gripper pinches the top plate's boss by its two
ends and lowers the plate onto the ball over 4 s (the plate's weight flattens the ball to about 7.5 cm). The right
gripper pinches the lever's bar 17 cm from its pivot and swings it over onto the boss. The left gripper, fingers
closed, pushes the lever's end down (aimed 3.5 cm below its lowest point: the arm gives under load) and holds it for
2 s: the dough force reaches about 69 N and the tortilla 15 cm. The right lifts the lever back onto its stop, and the
left lifts the top plate back onto its hinge stop. It reaches success at about step 1285 (64 s); the run stops at
step 1305, once success has held 1 s.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=press_a_corn_tortilla_in_a_tortilla_press.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Press the masa ball into a corn tortilla at least 14 cm across, then open the press." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/tortilla-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tortilla-dream/*.json; do inspect-robots video "$log"; done
```

`TORT_DEBUG=1` prints, every 5 steps (`TORT_DEBUG_EVERY=1` for every step), the top plate's and the lever's angles,
whether the dough has formed, its thickness, diameter and centre, the dough force now and its peak, whether it ran
past the edge, whether the press is open, the ball's position, and where each gripper is and how far it is open.

The wrong ways:

- `TORT_OFFCENTRE=1`: the same demo with the ball set down 5 cm off-centre, toward the arms. Pressed to 15 cm, the
  tortilla runs past the plates' edge: score 0 (it ran past the edge at about 10 cm across; the run went on to 15 cm
  and an open press).
- `TORT_NOLEVER=1`: the left gripper pushes the top plate down by its boss, by hand, instead of using the lever.
  The arm's push on the plate (about 26 N on the dough at its peak, with no leverage) takes the tortilla only to
  9.2 cm: score 0.25.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: debug runs of the scripted demo
without video (about 1,300 steps, 65 s of simulated time) took 10 to 15 minutes from start to finish; the demo
rendered at 1280 x 720 took about 7 minutes, and a Dream-1 trial of 2,400 steps at 320 x 176 about 6 minutes. Isaac Sim's
first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

An 8-inch cast-iron tortilla press (Victoria TOR-003, about 3.9 kg, about 25 US dollars), set on the table with its
plates' centre at x 22, y 0, its hinge toward the robot's left and its lever's pivot toward the right (taped or set on
a non-slip mat). Two 17 cm squares cut from a freezer bag on the plates. A 35 g ball of masa made from instant corn
masa flour (for example Maseca) and warm water, about 1 : 1.2 by weight, kneaded 2 minutes and rested 10 minutes under
a damp cloth, rolled between the palms into a ball; a fresh ball for each trial, from a batch kept covered at room
temperature (it keeps a few hours). A small plate 11 cm across for the ball. Before each trial: open the press fully
(top plate and lever on their stops), smooth fresh plastic sheets onto the plates, and put the ball on its plate. After
the trial measure the tortilla on the open bottom plate with a ruler.
