# Get exactly one candy out of a PEZ dispenser into a dish: stem held head-down, head tipped back, dispenser put down

A PEZ candy dispenser, loaded with 12 candies, lies on its back on the table, and a small white ceramic dish stands
beside its head. The instruction: "Get exactly one candy out of the PEZ dispenser into the white dish, then put the
dispenser back down on the table." It does not say how.

## Why it matters

Taking a candy from a PEZ dispenser is one of the most familiar small-object gestures there is: hold the stem in one
hand and tip the cartoon head back with the thumb. The head is a lid on a spring hinge. The dispenser's patent
([US 2,620,061][patent], "Pocket article dispensing container") describes the mechanism: a spring under a movable base
pushes the stack of candies up the stem; the lid is hinged at the back of the stem's top; opening it frees the top
candy; and "the lid is automatically returned to the closed position by the spring" when the thumb lets go. The candy
([Wikipedia, "Pez"][wiki]) is "a pressed, dry, straight-edged, curved-corner block 15 mm x 8 mm x 5 mm", "each Pez
dispenser holding 12 candy pieces".

What makes it a hand task:

- The dispenser weighs 22 g. A push on its head with nobody holding the stem just shoves the dispenser away, and lying
  on its back on the table the head cannot open at all (its crown would have to go down through the table top). One
  hand must hold the stem, off the table, against the other hand's push on the head: the two hands push against each
  other through a light, free object.
- The head opens only while it is held. Let go early and the spring snaps it shut on the candy. It must be held back
  about 50 degrees or more, with the stem tilted head-down over the dish, until the candy has slid out over the open
  head and dropped.
- The candy must end up in a 10 cm dish (one candy, not two), and the dispenser goes back down on the table, let go.

The instruction states the goal only. Holding the stem tilted over the dish and pushing the head with the other hand
is what the object's shape and spring force, as for a person.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the dispenser, the candy and the dish are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a hinge. The
dispenser (a stem with the candy's seat at its top end, and a head on a spring hinge) is built with `bodykit.py` as a
floating PhysX articulation; importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without
changing yam_arena, and gives the task file the dispenser's pose and the head's hinge angle.

## The scene

Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left, z up from the table top.

- PEZ dispenser (plastic, 22 g with its candies), loose, lying on its back along y, face up. Stem 7.8 cm long, 1.9 cm
  wide, 1.2 cm deep, foot at y 11.5, top at y 3.7, centred on x 15.0, 18 g with the 11 candies below the top one. At
  its top end is the top candy's seat: a channel 16 x 9 mm and 6 mm long, open only at the top end. Head 3.3 cm wide,
  2.4 cm deep and 3.0 cm tall, 4 g, on a hinge along x at the back edge of the stem's top, with a torsion spring
  (0.03 N m/rad) that closes it; it opens up to 100 deg. Closed, its underside covers the seat. Its crown is at y 0.7.
- The top candy: 15 x 8 x 5 mm, 0.71 g, lying in its seat under the closed head.
- White ceramic sauce dish, loose: 10 cm across, 2.5 cm tall, 4 mm walls and floor, 120 g, centred at (15.0, -7.0).
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

How the candy comes out: with the stem tilted head-down and the head tipped back, the head's underside turns into a
ramp beyond the open end of the seat. The candy slides out over it once the stem's tilt plus the head's angle reach
about 103 deg (with the demo's 55 deg tilt, a head angle of about 48 deg). Below that, the underside still stands
across the seat's mouth. A head let go before the candy has left swings shut on it.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the dispenser's pose and its head's hinge angle, the candy's and the dish's poses,
and the points of each finger's collision hulls.

- 0 for the rest of the run once the dispenser is dropped: it moves down faster than 0.5 m/s with no finger point
  within 1 cm of it.
- A candy counts as in the dish when its centre is within 4.2 cm of the dish's centre (the inner radius less 4 mm),
  within 1.2 cm above the dish's floor, and it is still. A candy on the table counts for nothing (no penalty).
- One candy in the dish: 1.0 when the dispenser is set down (its lowest point within 5 mm of the table top), let go
  (every finger point 2 cm or more from it) and still (it moved less than 1 mm over 1 s); otherwise 0.5.
- Two or more candies in the dish: 0.5 when the dispenser is set down and let go as above, otherwise 0.25. In the
  simulation only the top candy is loose, so this case arises only in real trials.
- No candy in the dish: 0.1 if the head was tipped back 45 deg or more at some time while the dispenser was 2 cm or more
  above the table, otherwise 0.
- 1.0 is success (held 1 s). Doing nothing scores 0.

A real trial is scored by counting the candies in the dish after the run; a drop, the head's opening and the
dispenser set down and let go are checked on the trial video.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=get_one_candy_from_a_pez_dispenser.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Get exactly one candy out of the PEZ dispenser into the white dish, then put the dispenser back down on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pez-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The left gripper, tilted 50 deg with
its fingers toward the head, closes across the stem 15 mm from its foot, lifts the dispenser and turns it head-down
55 deg over the dish. The right gripper, pointing toward the left arm 75 deg from straight down (nearly level), sets
its fingertips on the head's face 15 mm to one side of the axis (clear of the candy's path) and pushes the face back
round the hinge, aiming 8 mm past it and at 90 deg; the arm gives, and the head reaches about 70 deg. The candy starts
to slide out over the open head at 48 deg and is in the dish by 54 deg, while the head is still pushed open. The right hand backs off and
the head springs shut; the left turns the dispenser back level, lays it on its back where it was, lets go and rises.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=get_one_candy_from_a_pez_dispenser.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Get exactly one candy out of the PEZ dispenser into the white dish, then put the dispenser back down on the table." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/pez-demo
```

Videos of a run: `inspect-robots video <log>.json` in its log directory.

`PEZ_DEBUG=1` prints a `DBG` line every 5 control steps (the head's angle and its largest so far, the dispenser's
lowest point, the nearest finger, its vertical speed, the candy's position, the candies counted in the dish, and each
arm's TCP and finger gap). `PEZ_ONE=1` runs the wrong way: the right hand alone pushes the face of the dispenser lying
on the table, as in the demo, while the left stays at the ready pose. The head does not move at all (0.0 deg: lying
on its back it would have to turn down into the table), the candy stays in its seat, and the run scores 0. Pushing the
head back only 40 deg (`PEZ_HEAD_TO=45`) leaves the candy inside and also scores 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other simulation jobs, the scripted demo while rendering 1280 x 720
video took about 3.5 minutes (583 steps, 29.2 s of simulated time, ending on success), running beside a Dream-1 trial;
the Dream-1 trial of 2,400 steps at 320 x 176 took about 13 minutes (0.33 s per step). Debug runs of the demo without
video (640 steps) took 3 to 5 minutes. Isaac Sim's first start on a new machine adds a few minutes while it compiles
its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A standard PEZ dispenser (any character head on a plain stem, about 2 USD with candy) loaded with one 12-candy
refill, and a small white ceramic sauce dish about 10 cm across. For each trial put back or replace the candy that came
out (reload with the head tipped back), let the head close, lay the dispenser on its back at the tape marks with its
foot toward the robot's left, and put the empty dish at its marks.

[patent]: https://patents.google.com/patent/US2620061A/en
[wiki]: https://en.wikipedia.org/wiki/Pez
