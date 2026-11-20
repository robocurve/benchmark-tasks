# Measure a foot on a Brannock device: heel seated in the heel cup, width bar and arch pointer slid in to touch it

Having a foot measured on a Brannock device is something most people know from a shoe shop, as a child or with their own children. A men's
Brannock foot-measuring device (metal, about 35 x 13 cm) lies on the table, its right heel cup at the far end. A
rigid, true-size foot-and-ankle model (a hollow plastic display foot, men's US 9) stands on its plate, but set down
25 mm too far forward: its heel is not in the heel cup. As the maker says to prepare the device, the width bar is at
its widest and the arch pointer is slid back. The instruction asks for the foot to be measured: its heel seated in
the heel cup, the width bar against the side of the foot and the arch pointer at the ball joint, then everything let
go. It does not say how.

The task is the task file [`measure_a_foot_on_a_brannock_device.py`](measure_a_foot_on_a_brannock_device.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters, and the procedure

The Brannock device has been the shoe shop's measuring tool for a century ([Retail Brew, "Tools of the trade"][retailbrew]: "it
became a fixture in shoe stores"; the company "said it has sold more than a million of the devices"). Anyone watching
a clip knows at once what the robot is doing. Measuring feet is a regular task only for shoe-shop staff, a small part
of the 3 million or so US retail salespersons, so the task is filed under Societal relatability, not Impact on jobs.

The maker's instructions ([Brannock Device Co., "Instructions for use"][brannock]) give the procedure this task checks:

- "The width bar should be set to its widest position and the arch length indicator should be slid back, so the foot
  can be positioned easily on the device." (the start state here)
- "Be sure the heel is properly located against the back of the heel cup, by grasping the customer's ankle and device
  together, as illustrated in photo."
- "Slide the pointer (A on diagram) forward so the inside curve of the pointer fits the ball joint of the foot ...
  When the pointer is properly located, the lower middle rib will be against the ball joint on the side of the foot
  (B on diagram). This yields the arch measurement."
- "Slide the width bar firmly to the edge of the foot."

Wikipedia ([Brannock Device][wiki]) lists the three measurements: foot length, arch length ("the length from heel to
the inside of the ball of the foot, or medial metatarsophalangeal joint") and width.

## The hand skill

The skill is light contact on curved anatomy: two small sliders, each brought along its track until it just touches a
bulge of the foot, without shoving the light, loose foot out of place. The order is forced by the device: every
reading is taken from the heel cup, so the heel must be against the back of the cup before either slider means
anything (in the scorer, neither slider counts unless the heel is seated). The model foot only rests on the plate
(0.30 kg, friction 0.375 on the plate): drawn back by its ankle, its heel slides into the cup; pushed on its ball joint
by the arch pointer or on its outer edge by the width bar, it turns about the heel cup. The two sliders are small parts
(the pointer's thumb tab is 10 x 14 x 30 mm, the width bar's finger tab 20 x 12 x 35 mm) on low-friction tracks
(0.4 N and 0.5 N), and each must be brought to touch a curved surface: the rib to the side of the ball joint (a 16 mm
bulge), the bar to the widest point of the outer edge. The arms are soft sideways (about 0.1 to 0.2 N per mm of
overshoot), so a slider pushed a few mm past first touch presses the foot with 1 to 2.5 N; pushed much further, the
pointer turns the foot and its rib slips past the bulge.

A hard shove shows what goes wrong: the arch pointer commanded 16 mm past first touch (up to 3.5 N) turns the unheld
foot about 2 deg about the heel cup, and the rib rides 13 mm past the ball joint, an arch reading about two sizes long
(score 0.6: heel seated and width bar touching, pointer wrong). The width bar is more forgiving: pressed with 4.4 N it
turns the foot less than 2 deg and moves its heel about 1 mm, and the device does not slide on the table (2 mm at most).

Two arms are not required. The maker says to steady the foot "by grasping the customer's ankle and device together",
and the scripted demo does that: one gripper closes round the ankle, draws the foot back until its heel sits against
the back of the cup and keeps holding it there while the other slides the pointer and the bar (with the ankle held,
the same hard shove still stops the rib at the ball joint: score 1.0). One hand alone also scores 1.0 when its pushes
are gentle (the demo's pushes, up to 1.4 N on the pointer and 2.5 N on the bar).

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the device and the foot are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a slide. The
device (plate, two heel cups, scale, and its two sliders) and the foot are built with `bodykit.py`, which adds such
bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. This
copy of `bodykit.py` adds prismatic links (`Link(slide=True)`): a slider on its track with Coulomb friction (N) and
travel limits, in a joint frame that may be turned about the vertical. The device is one loose articulation (it can
slide on the table); the width bar and the arch pointer are its two sliders. Both sliders have PhysX contact reporting,
so the task file reads the contact force between each slider and the foot every control step, and both meet the foot
through PhysX's compliant contact (3 N/mm, 10 N s/m: the give of the display foot's vinyl skin), so a slider pressed
against the foot presses steadily instead of chattering.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top. The device lies along x, heel cups at the far end, toes toward the robot.

- Brannock device (men's), loose on the table, 0.80 kg: a metal plate 34.6 x 13.0 cm, 1 cm thick, from x 0.2 to 34.8,
  y -6.5 to 6.5; a printed scale (visual) with a line for each men's size 5 to 15 (size 1 = 7 2/3 in, plus 1/3 in a
  size). Right heel cup: a wall 25 mm tall and 5 mm thick on a half circle of inner radius 35 mm, its back at x 34.0,
  y 0 (the origin of every reading); the left heel cup at the other end, facing back (not used).
- Width bar (0.05 kg), on the foot's outer (left, +y) side: a bar 10 cm long (x 10.5 to 20.5), 10 mm thick, 20 mm tall,
  its inner face at y 5.8 (the widest setting); it slides toward -y on its track (0.5 N friction, 30 mm travel). A
  finger tab 20 x 12 x 35 mm stands on its outer face at x 11.5.
- Arch pointer (0.02 kg), on the foot's inner (right, -y) side: a slider 88 mm long on the device's edge (y -5.3 to
  -6.6) with a rib (8 mm round, 17 mm tall) on its inner side reaching to y -4.4, and a thumb tab (10 x 14 x 30 mm)
  7 cm ahead of the rib; it slides along x on its track (0.4 N friction). At the start the rib is at x 20.0 (slid back),
  the tab at x 13.0.
- Foot-and-ankle model (right foot, men's US 9), loose, 0.30 kg, standing on the plate, toes toward the robot: 26.2 cm
  heel to toe, 10.0 cm wide at the ball; heel back at x 31.5 (25 mm in front of the heel cup's back), on the device's
  centre line. The ball joint (first metatarsal head, the inner bulge) is 19.0 cm from the heel's back, its inner edge
  4.9 cm right of the centre line; the widest point of the outer edge (fifth metatarsal head) is 17.0 cm from the heel's
  back, 5.1 cm left of it. The ankle is a 66 mm round stump rising to 14 cm above the sole, its centre 5 cm in front of
  the heel's back.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the device's pose (it is loose), the two sliders' travel, the foot's pose in the
device's frame, the contact force between each slider and the foot (PhysX pair-wise contact forces), and the points of
each finger's collision hulls. Nothing is latched; the score is taken at the end of the run:

- Heel seated (0.3): the heel's back within 3 mm of the back of the heel cup, the heel centred in the cup within 3 mm,
  the foot turned less than 3 deg on the device and standing flat (tilted less than 3 deg).
- Width bar against the foot (+0.3, only with the heel seated): the width bar touches the foot, the contact force
  between them 0.05 to 5 N (mean over the last 0.5 s).
- Arch pointer at the ball joint (+0.3, only with the heel seated): the pointer touches the foot, contact force 0.05 to
  5 N (mean over the last 0.5 s), and its rib within 5 mm of where it meets the ball joint of the seated foot (rib at
  17.68 cm from the cup's back: the model's arch reading).
- 1.0 (success, held 1 s): all three, every finger point 2 cm or more from the foot and from both sliders (both hands
  have let go), and the foot and both sliders still (moving less than 1 mm in 1 s).

Doing nothing scores 0 (the heel starts 25 mm out of the cup and neither slider touches the foot). A slider pressed
in and let go stays where it is (its track friction holds it) and keeps touching the foot with the little force its
friction can hold (0.1 to 0.5 N in the demo), so "touching" is checked after every hand has let go.

A real trial is scored from a photo and the video: the heel touching the back of the heel cup (a 3 mm feeler gauge or
a 3 mm drill shank does not fit between them); the foot standing on the device's centre line; the width bar touching
the foot (a strip of paper slid between them is held); the arch pointer's rib touching the side of the ball joint and
the rib within 5 mm of the rib's place for this model (marked once on the device's edge; one arch size is about
6 mm of rib travel); both hands clear.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=measure_a_foot_on_a_brannock_device.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Measure the foot model with the Brannock device: get its heel seated in the heel cup, bring the width bar against the side of the foot and the arch pointer to the ball joint, then let go of everything." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/brannock-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The right gripper comes down round the
ankle, fingers closing across the foot, closes on it and draws the foot back 37 mm (25 mm to seat it and 12 mm past,
since the arm stops short under load) and holds it there, so it keeps the heel pressed against the back of the cup. Meanwhile the left gripper waits beside the toes. Then, fingers closed, it reaches over the toes,
comes down behind the arch pointer's thumb tab and pushes it forward until the rib meets the side of the ball joint
(commanded 7 mm past first touch: up to 1.4 N), eases off and rises; then it comes down outside the width bar's finger
tab and pushes the bar in to the outer edge of the foot (commanded 20 mm past first touch: up to 2.5 N), eases off and
rises. The right gripper lets go of the ankle, rises, moves back along the foot with its open fingers on either side
of the ankle, and away to the right.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=measure_a_foot_on_a_brannock_device.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Measure the foot model with the Brannock device: get its heel seated in the heel cup, bring the width bar against the side of the foot and the arch pointer to the ball joint, then let go of everything." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/brannock-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/brannock-dream/*.json; do inspect-robots video "$log"; done
```

`BRANNOCK_DEBUG=1` prints, every 5 steps (`BRANNOCK_DEBUG_EVERY=1` for every step), where the heel is relative to the
back of the cup (along and across), how far the foot is turned and tilted, both sliders' travel and the rib's place
against its target, the contact force of each slider on the foot and its peak, where the device is, how close the
nearest finger is, which conditions hold, and where each gripper is.

`BRANNOCK_ONE=1` runs the demo one hand at a time: the right gripper seats the heel the same way, then lets go and
moves away before the left gripper works the two sliders with the same pushes. With the demo's gentle pushes it also
scores 1.0. With `BRANNOCK_PTR_PAST=16 BRANNOCK_BAR_PAST=35` (hard pushes) it scores 0.6 (the rib rides 13 mm past the
ball joint), while the two-hand demo with the same hard pushes still scores 1.0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other jobs: the scripted demo reaches success at step 777 (39 s of
simulated time) and took about 4.5 minutes at 1280 x 720 with video; about 13 minutes for a Dream-1 trial of 2,400 steps (it shared the GPU with two other runs for its first 5 minutes). Isaac Sim's first start on
a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## What is modelled and what is not

The device is modelled as itself at true size: the plate, both heel cups, the width bar and the arch pointer on their
own tracks with Coulomb friction, and a printed length scale; the width scale on the bar and the arch scale are not
drawn. Its 0.80 kg mass and the sliders' friction are estimates. There is no person: a rigid display foot stands in
for a customer's foot, so the foot does not spread under body weight and it moves when pushed, as a light model does;
the toe-length reading is not part of the task (it needs no manipulation). The foot is built from boxes, cylinders,
capsules and spheres, with the ball joint and the fifth metatarsal head as the two bulges the sliders touch; its
0.30 kg and its friction are estimates for a hollow plastic display foot. The sliders meet the foot through a compliant
contact (3 N/mm) for the vinyl skin.

## Building it for real

A Brannock device (men's or combination model, about 90 USD from the maker) and a rigid, true-size foot-and-ankle
display form for a men's US 9 (hollow plastic, about 26 cm long, cut above the ankle so that the ankle is about 14 cm
tall and 6 to 7 cm across), with its arch size read once on the device and marked. Lay the device on the table along
x, heel cups at the far end (the back of the right heel cup 34 cm out from the midpoint between the arm bases), set the
width bar to its widest and slide the arch pointer back; stand the foot on it on the centre line with its heel 25 mm in
front of the back of the right heel cup.

[brannock]: https://brannock.com/pages/instructions-fitting-tips
[wiki]: https://en.wikipedia.org/wiki/Brannock_Device
[retailbrew]: https://www.retailbrew.com/stories/2023/01/18/tools-of-the-trade-how-the-brannock-device-helped-the-shoe-industry-put-its-best-foot-forward
