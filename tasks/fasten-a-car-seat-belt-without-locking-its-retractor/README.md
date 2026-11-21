# Fasten a car seat belt: draw it across smoothly so the retractor does not lock, and latch the tongue in the buckle

A car front seat stands on the table with its three-point seat belt as installed in a car: the webbing comes out of the
retractor at the foot of the door pillar, runs up to the shoulder guide on the pillar, down through the sliding latch
plate (the tongue) and on to its anchor at the sill. With the belt unfastened the tongue hangs on the webbing beside
the pillar. The buckle stands on its short stalk at the other side of the cushion. The instruction is only "Fasten the
seat belt."

Nearly everyone fastens a seat belt on every car trip: the tongue is drawn across the seat and pushed straight into the
buckle until it clicks. The retractor is an emergency-locking retractor (an inertia reel). One that senses webbing
withdrawal locks when the webbing accelerates out of it, so a belt that is snatched stops dead and has to be let back a
little and drawn again, smoothly. The US standard for seat belt assemblies, FMVSS 209 (49 CFR 571.209,
[S4.3(j)][fmvss]), sets the floor: such a retractor "shall not lock before the webbing payout extends to the minimum
limit of 51 mm ... when the retractor is subjected to an acceleration no greater than 0.3 g". The same section limits
the retraction force of a combined lap and shoulder belt to "not less than 1 N and not more than 7 N".

## The hand skill

- The tongue is drawn 48 cm across the seat against the retractor's pull (2 to 3.5 N along each run of webbing),
  smoothly: a sharp start (webbing acceleration over 0.7 g in this model) locks the reel, the webbing stops dead and
  the tongue is pulled out of the fingers or held short of the buckle.
- It is carried with its blade pointing straight down, though the webbing pulls it toward the pillar and turns it in
  the hand.
- The 3 mm blade goes into a 4.5 mm slot (26 mm long, under a small funnel) on a buckle head whose stalk gives, and is
  pushed past the latch's spring (about 6 N) until the latch drops into the blade's window; then the hand lets go and
  the latch must hold the tongue against the webbing's pull.

One arm is enough (the other stays at its ready pose); the difficulty is in the speed of the draw and the aim of the
insertion, not in a second hand.

The task is the task file
[`fasten_a_car_seat_belt_without_locking_its_retractor.py`](fasten_a_car_seat_belt_without_locking_its_retractor.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) and
[`strapkit.py`](strapkit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on
Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs everything, including the
Dream-1 policy. The task needs no models from Arena's object library and uses no third-party assets: the seat, pillar,
belt and buckle are built in these files from boxes and cylinders.

`yam_arena` task files place single boxes, cylinders and spheres. The seat and the pillar are fixed bodies built with
`bodykit.py`, and so is the buckle (a `Linkage`: a fixed root, two sprung hinges for the stalk and a sprung slide for
the latch pawl). The belt is one articulation built with `strapkit.py` (rigid parts on revolute, prismatic and fixed
joints, with loop-closing joints outside the joint tree); this copy adds a loop joint that holds a part at a point but
lets it turn (the lap webbing's end on its anchor bolt). Importing them hooks `yam_arena.scene` when the simulator
starts, without changing yam_arena. Every control step the task file reads the belt's joints, sets the retractor
spring's preload, and locks or frees the reel by writing the slides' joint limits through Isaac Lab's articulation
API.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, heights above the table top.

- Car front seat, true size, fixed: cushion 45 cm wide (y -43 to +2), its top 4.5 cm above the table, from x 6 to 40,
  with 2 cm side bolsters (the front 15 cm of a real cushion is left off so that it clears the arm bases); backrest
  9 cm thick, its front at x 40, 62 cm tall; head restraint on two posts.
- Door pillar trim, fixed, on the robot's left (as for a left-hand-drive driver's seat seen from the front): y +9 to
  +15, x 33 to 47, 68 cm tall, the retractor housing at its foot, a sill beside the cushion.
- Three-point belt, 48 mm webbing: shoulder guide on a bracket at x 24.5, y 3, 60 cm up; lap anchor bolt on the sill
  at x 24.5, y 3, 4.5 cm up.
- Tongue (latch plate), 40 g: plastic head 50 x 13 x 28 mm with the webbing slot at its top; steel blade 22 mm wide,
  3 mm thick, 24 mm long, with a 12 x 11 mm latch window whose lower edge is 6 mm above the tip. At the start it hangs
  blade down on the vertical webbing at x 24.5, y 3, its slot about 14 cm up, its faces toward the pillar.
- Buckle on a short stalk at x 23, y -44.8 (the inboard side of the cushion): head 53 x 32 x 50 mm, red release button,
  mouth 9.5 cm up: a slot 26 x 4.5 mm, 30 mm deep, under a funnel 6 mm deep and 6 mm wider each side (3 mm at its
  ends). The stalk gives: two hinges at its root, 3 N m/rad each, up to 20 deg.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled:

- The webbing is two straight runs that the retractor keeps taut: shoulder guide to tongue, and tongue to lap anchor.
  Each run turns freely about its ends (universal joints at the guide and the tongue, a ball joint at the anchor) and
  lengthens on slides: the shoulder run on one, the lap run on five in series that share its length. Each slide is
  pulled back by the retractor spring (4 N/m, preloaded): 2.0 to 3.5 N along each run over the draw. The webbing is
  drawn as flat 48 mm strips on these parts; it does not sag, twist or touch the seat.
- The tongue turns freely about its webbing slot.
- The reel's lock (the inertia reel): every control step (50 ms) the payout (the two runs' lengths less their start)
  and its rate are read. If the rate rises by more than 0.7 g over a step, from rest or while paying out, the reel
  locks: no more webbing comes out (each slide's outward limit is set where it is) until 5 mm has gone back in. A belt
  whipping back onto its stop and rebounding does not count. FMVSS 209 only says the reel must not lock at 0.3 g or
  less; makers do not publish their lock point, so 0.7 g is an estimate.
- The latch: a steel pawl on a 2500 N/m spring slide sticks 2.75 mm into the slot, ramped on top and flat underneath,
  its underside 15 mm below the mouth. The blade's tip pushes it back (about 6 N); it springs into the blade's window
  once the tongue's head is within 3 mm of the mouth, and the webbing's pull then holds the window's lower edge on
  it. The release button is drawn only.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the belt's slides, the tongue's pose, the buckle head's pose, the pawl's position and
the points of each finger's collision hulls. "Latched": the pawl back within 0.8 mm of its rest with the tongue's head
within 3.5 mm of the mouth, in line with the slot (the window sits on the pawl). "Let go": every finger point 2 cm or
more from the tongue.

- 0.25: the blade's tip has been 10 mm or more inside the mouth, in line with the slot, at some time.
- 0.4: latched at the end, but still held (or moving).
- 0.6: latched, let go and still (under 2 cm/s) for 1 s, but the reel locked at some time in the run.
- 1.0 (success, held 1 s): latched, let go and still, and the reel never locked.

Doing nothing scores 0. A real trial is scored from the trial video and a tug: after the run a person pulls the
tongue by hand; latched, it stays in the buckle. A lock is heard (the reel clicks) and seen (the belt stops dead while
the hand still pulls).

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fasten_a_car_seat_belt_without_locking_its_retractor.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the seat belt." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/belt-dream
```

The scripted demo, which reaches full score (open loop from the known layout; right arm only). The right gripper comes
down over the hanging tongue, its fingers closing across the head's 13 mm thickness, and pinches it. It draws the
tongue across the seat to above the buckle in 5 s on a smooth (minimum-jerk) path: the webbing's acceleration peaks
at about 0.17 g. Under the webbing's pull the arm stops about 2.4 cm short of where it is sent and the tongue turns a
little in the grip, so the hand is sent past the buckle and turned 4 deg. It lowers the blade into the mouth and
pushes 10 mm past the latch point, holds, lets go, rises and goes back to the ready pose. It reaches success at step
360 (18 s).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fasten_a_car_seat_belt_without_locking_its_retractor.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the seat belt." \
  --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval/belt-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/belt-dream/*.json; do inspect-robots video "$log"; done
```

`BELT_DEBUG=1` prints, every 5 steps (`BELT_DEBUG_EVERY=1` for every step), the payout, its rate and acceleration (and
the peak so far), whether the reel is locked and how many locks there have been, the tongue's slot position and tilt,
the blade tip's depth in the mouth and its position in the buckle head's frame, the pawl's position, whether the blade
has entered and is latched, how close the nearest finger is to the tongue, and where each gripper is and how far it is
open.

`BELT_JERK=1` runs the demo the wrong way: the first 15 cm of the draw is a jerk in 0.1 s, as when a belt is snatched.
The webbing accelerates out at about 0.9 g, the reel locks after 3 mm, the webbing stops dead and pulls the tongue out
of the fingers, and it swings back to the pillar; the rest of the demo goes on with an empty hand. Score 0. With the
lock switched off (`BELT_LOCK_G=99`), the same jerk keeps the tongue in the hand and the demo still reaches 1.0, so it
is the lock that defeats the snatch.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: a debug run of the scripted
demo without video (about 350 steps, ending on success) took about 4 minutes from start to finish; a Dream-1 trial of
2,400 steps at 320 x 176 took about 6 minutes. Isaac Sim's first start on a new machine adds a few minutes
while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A car front seat (from a scrapyard) and its three-point belt with an emergency-locking retractor, its tongue and its
buckle on a stalk (seat belt assemblies are sold as spare parts; a matching set costs about $40-80), bolted to a
board or a frame as in a car: the retractor at the foot of a pillar post outboard of the seat, the shoulder guide 60 cm
up on the post, the lap anchor at the cushion's level beside it, the buckle stalk at the inboard side of the cushion.
Set the seat so its cushion is about 4.5 cm above the table (on a frame through the table edge, or with the table
lowered round it). An emergency-locking retractor that senses the vehicle's tilt as well must be mounted upright as
in the car, or it locks. Before each trial press the release button, let the belt wind back in slowly by hand (do not
let it whip back) and leave the tongue hanging at rest beside the pillar, blade down.

[fmvss]: https://www.law.cornell.edu/cfr/text/49/571.209
