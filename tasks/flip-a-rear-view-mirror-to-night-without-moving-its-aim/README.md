# Flip a car's rear-view mirror to night with its tab, the housing steadied so the mirror's aim does not move

Flipping the interior rear-view mirror to its night setting, when the headlights of the car behind glare in it, is
something nearly every driver does. A section of a car's windscreen leans back on a bench stand at the rake of a real
windscreen (30 deg from the horizontal), and a day/night rear-view mirror hangs from it on its stem, aimed, in its day
position. The instruction asks for the mirror to be switched to night without changing where it is aimed, and then let
go; it does not say how.

What makes it hard is visible in the mirror itself. The day/night tab sits on a two-position over-centre hinge in the
middle of the housing's bottom edge, 38 mm below the ball joint the whole mirror hangs on. Pulling it over its centre
takes 0.10 N m about its hinge (about 5 N at the tab), and the hinge's spring pushes that same torque back on the
housing. The ball of this mirror holds the housing with 0.07 N m (a push of 0.55 N at the housing's end turns it,
which is how the driver aims it). So however one hand flips the tab, the housing tips on its ball before the tab goes
over centre, and the aim is lost: in the one-handed run the mirror tipped 12 deg. The other hand has to take the
housing by its end and hold it still while the tab is pulled over, and then both hands have to let go without nudging
it. This is what drivers do with a mirror whose ball has worn loose: one hand on the housing, a finger on the tab.

The task is the task file
[`flip_a_rear_view_mirror_to_night_without_moving_its_aim.py`](flip_a_rear_view_mirror_to_night_without_moving_its_aim.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

Every car has an interior mirror, and the manual day/night prism mirror is the common kind (automatic dimming mirrors
are an option on dearer trims). Owner's manuals give the procedure ([Kia owner's manual, "Day/night rear view
mirror"][kia]): "Make this adjustment before you start driving and while the day/night lever is in the day position
(1)." "Pull the day/night lever toward you (2) to reduce the glare from the headlights of the vehicles behind you
during night driving." The mirror is aimed in the day position; the lever tips the housing a few degrees on the
bracket inside it, so the dim reflection from the wedge-shaped glass's uncoated front face replaces the bright one
from its silvered back. The night image is the same view, dimmer, only if the ball joint has not turned with the
flip; if it has, the driver has to re-aim the mirror while driving, which the same manual warns against ("Do not adjust
the rear view mirror while the vehicle is moving"). Anyone watching a clip knows at once what the robot is doing.

The mirror is modelled on an ordinary replacement day/night mirror such as the CIPA CM32000 ([etrailer listing][cipa]:
"10" CIPA replacement rearview mirror ... features a day/night thumb switch", "10" long x 2-1/2" tall", wedge mount,
about 22 USD).

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the windscreen stand and the mirror are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a hinge. The
stand and the mirror are one fixed articulation built with `bodykit.py`: the stand, the windscreen glass, the button,
the wedge mount and the stem are its fixed base; the ball joint is three hinges through the ball's centre (about the
vertical, across and along the housing), each with 0.07 N m of Coulomb joint friction; the housing hangs on the third;
the tab hangs on the housing on a hinge with stops at day (0) and night (35 deg). Importing `bodykit.py` hooks
`yam_arena.scene` when the simulator starts, without changing yam_arena, and gives the task file each part's pose and
each joint's angle. The tab's over-centre spring is the hinge's spring drive, whose target the task file sets every
control step to the stop the tab is nearer (day while it is less than 17.5 deg off day, night beyond), so the torque
rises from 0 at either stop to 0.10 N m at the centre and then reverses. This copy of `bodykit.py` also lets a shape
carry an opacity: the clear glass of the windscreen collides but is not drawn (its black ceramic band is), so the
cameras see the mirror through it as a driver does.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Windscreen stand, fixed: a dark dashboard block, x 50 to 62, y -14 to 24, 18.5 tall. On it a section of 5 mm laminated
  windscreen glass, 36 cm wide (y -13 to 23) and 38 cm along its slope, leaning back 30 deg from the horizontal (60 deg
  from the vertical, the rake of a typical car windscreen): its lower edge on the dashboard at x 52.4, z 18.5, its upper
  edge toward the robot at x 19.5, z 37.5. A 3 cm black ceramic band runs round its edge; two side brackets hold its
  far half from below.
- Mirror button bonded to the inside of the glass at x 29.9, y 5, z 31.5, with the mirror's wedge mount slid on it and
  the stem down to the ball.
- Day/night rear-view mirror (like CIPA CM32000): housing 25.4 x 6.4 x 3.5 cm (long side along y), black plastic, its
  prism glass (24.4 x 5.4 cm) facing the robot, 180 g; its middle at x 25.5, y 5, z 24.0 (glass face at x 23.75; bottom
  edge 20.8 above the table). It hangs on a ball joint inside it, 6 mm above its middle and straight above its centre of
  mass, which holds it with 0.07 N m about each axis and lets it turn up to 25 deg each way.
- Day/night tab: 28 mm wide, 22 mm long, 5 mm thick, hinged along the housing's bottom edge in its middle, 3 mm behind
  the glass face (hinge at x 24.05, y 5, z 20.8), in the day position: pointing down and 15 deg toward the robot. Night
  is 35 deg further toward the robot (toward the driver). The over-centre spring holds it at whichever stop it is
  nearer: from 0 at the stop to 0.10 N m just before the centre at 17.5 deg.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the housing's orientation, the tab's hinge angle and the points of each finger's
collision hulls.

- The housing's turn is the angle of its rotation from its pose at the first control step (the aim the driver set),
  about any axis. Once it has been more than 1.5 deg, the score is 0 for the rest of the run: 1.5 deg of mirror turns
  the reflected view 3 deg, so at the rear window, about 2.5 m behind the mirror in a car, the view moves 13 cm, about a
  third of the window's height, and the driver would re-aim.
- Otherwise 0.2 once the tab has moved 10 deg or more off its day position.
- 0.5 when the tab is latched in night: 30 deg or more off day (it is over centre, and the spring holds it on its
  night stop at 35 deg).
- 1.0 (success, held 1 s): latched in night, the aim kept, every finger point 1 cm or more from the housing and the
  tab, and the mirror still (the housing and the tab each turn less than 0.05 deg over 1 s).

Doing nothing scores 0. A real trial is scored with a laser pointer and the trial video: before the trial, a laser
pointer on a tripod at the driver's eye point (about 60 cm in front of the glass) shines on the mirror's middle, and
the bright reflected spot is marked on a board 2 m behind the mirror. The board is filmed through the trial. The spot
that shows the view (the bright one while the tab is in day, the dim one from the glass's front face once it is in
night) must stay within 10.5 cm of the mark (1.5 deg of mirror turn at 2 m); a prism mirror's dim spot lands on the
day mark when the ball has not moved (check this once per mirror by flipping it by hand with the housing held, and
correct the mark by any offset). The tab's position, the let-go and the stillness are read from the trial video.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flip_a_rear_view_mirror_to_night_without_moving_its_aim.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Switch the car's rear-view mirror to its night (anti-glare) setting without changing where it is aimed, then let go of it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mirror-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The right hand comes in level from the
right, pointing along +y with its fingers closing along x, slides its open fingers 4 cm over the housing's right end
and closes on the glass and the back (commanded 4 mm inside each face). Then the left hand, which has come down beside
the windscreen and in under the mirror, level and pointing forward with its fingers closing along y, takes the tab by
its two sides 4 mm from its tip (commanded 8 mm inside each side) and turns it about its hinge toward the robot over
6 s, the hand turning with the tab, aimed 40 deg round (the arm turns short under load: the tab goes over centre and the
spring takes it on to its stop). It lets go of the tab and backs away along its fingers; then the right hand opens and
backs away to the right.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flip_a_rear_view_mirror_to_night_without_moving_its_aim.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Switch the car's rear-view mirror to its night (anti-glare) setting without changing where it is aimed, then let go of it." \
  --scorer task_score --max-steps 520 --store-frames --log-dir ~/yam-eval/mirror-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/mirror-dream/*.json; do inspect-robots video "$log"; done
```

`MIRROR_DEBUG=1` prints, every 5 steps (`MIRROR_DEBUG_EVERY=1` for every step), the tab's angle and the most it has
moved, the housing's turn now and its worst so far, each joint's angle, how close the nearest finger is, whether the
mirror is still, any broken rule, and where each gripper is and which way it points.

`MIRROR_ONE=1` runs the demo the wrong way: the left hand flips the tab exactly as in the demo while the right hand
stays at its ready pose, so nothing steadies the housing. The spring's torque, more than the ball's friction, tips the
whole mirror on its ball instead of letting the tab go over centre: the housing turns past 1.5 deg by step 251, with
the tab only 3 deg off day, and 12.4 deg in all, and the tab, which got only 12 deg off day, springs back to day when let go. Score 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other simulation jobs, the scripted demo while rendering 1280 x 720
video took 143 s (449 steps, 22.5 s of simulated time, ending on success), running beside the Dream-1 trial and the
screenshot run; the Dream-1 trial of 2,400 steps at 320 x 176 took 357 s (0.15 s per step). Debug runs of the demo
without video took 3 to 6 minutes with two or three runs sharing the GPU. Isaac Sim's first start on a new machine adds
a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A day/night prism replacement mirror with a wedge mount (CIPA CM32000 or similar, about 22 USD); a windscreen section
or, as a stand-in, a 5 mm glass or polycarbonate pane about 36 x 38 cm with the mirror's metal button bonded to it with
rear-view-mirror adhesive (a few USD), held at 30 deg from the horizontal on a simple wooden stand (a block 18.5 cm tall
under its lower edge, two side brackets); a laser pointer on a tripod and a board 2 m behind the mirror. The ball of a
new mirror is stiffer than this one's; the task uses a mirror whose housing turns at a push of 0.5 to 0.6 N at its end
(12.7 cm from the ball): check it with a small spring scale before each session, and ease the ball (a worn mirror, or
the socket's tension screw where it has one) until it does. The tab needs about 5 N at its tip to go over centre (check
with the spring scale). For each trial put the tab in day, aim the mirror so the laser spot sits on the mark, and
start.

[kia]: https://ownersmanual.kia.com/full_webhelp/CD/2022/en_US/topics/chapter4_13_1.html
[cipa]: https://www.etrailer.com/Rear-View-Mirrors/CIPA/CM32000.html
