# Take the plates off a contact grill for washing, then slide out the drip tray

A round contact grill (a George Foreman 360-style grill with removable plates, at true size: 30 cm plates in a 34 cm
body) stands on the left of the table with its lid open, standing upright at the back. Both cooking plates are latched
in: the lower plate lies in the body, the upper plate stands in the open lid with its cooking face toward the robot.
Each plate has a black release handle on its left and on its right side, 39.4 cm apart tip to tip. A drip tray sits
in a slot in the front of the body, its back tucked under the body's front edge. A blue silicone dish-drying mat lies
on the right. The instruction asks for both plates to be taken off and put on the drying mat, and for the drip tray to
be taken out from under the grill; it does not say how.

The procedure is the maker's. George Foreman GRP106QPG Use & Care (360 Grill): "Holding a plate with both hands, use
your thumbs to pull out the release handles (C)", and the latches "fit into the openings on each side of the grill".
George Foreman GRP3CAN Use & Care: "The grill plates may be removed from the appliance by pulling out on the grill
plate release handles. Wash in warm, soapy water and dry thoroughly." GRP106QPG: "Allow the drip tray to cool before
removing it from under the grill." (Both manuals are hosted by appliance-parts sites, e.g. Appliance Factory Parts.)

Why both arms are needed, visible in the scene: each handle slides outward on a spring and carries a latch tongue that
hooks under a lip of the grill frame below the plate's edge. The tongue clears the lip only while its handle is held
pulled out (6 mm or more of its 10 mm travel); let go, it snaps back. The two handles of a plate are on opposite sides,
39 cm apart, so one gripper can hold only one of them out: on one handle alone the lower plate only tilts about 9
degrees on its other latch, and the upper plate only swings a few degrees out of the lid, and each falls back in when
let go. The two grippers pinch the two handles, pull them apart and lift the plate off by them. The extra steps follow
from the grill itself: the upper plate stands in the lid, so it has to be drawn forward out of the lid and turned flat
before it can be laid on the mat; the drip tray's back is under the body's front edge, so it cannot be lifted out and
has to be slid out toward the robot first. The plates end stacked on the mat (the mat has room for one plate's
footprint within reach of both arms).

The task is the task file [`take_the_plates_off_a_contact_grill.py`](take_the_plates_off_a_contact_grill.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. No third-party assets: every object is built from boxes, cylinders and
triangle meshes written in the task file.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a spring. The grill
(body, lid, the two plate seats with their latch lips), the two plates and the drip tray are bodies built from several
shapes, and each plate is an articulation whose two handles slide on spring joints, so `bodykit.py` adds such bodies
and joints: importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. This copy adds
sliding (prismatic) spring joints to articulations and contact reporting on plain bodies.

What is modelled and what is not: the plates are 30 cm round cast plates 1.2 cm thick, 0.60 kg each (an estimate),
with nine 4 mm ridges on the cooking face and a ring on the back that they rest on. Each handle (4.5 x 3.9 x 2.0 cm,
0.02 kg) slides 10 mm outward on a 400 N/m spring (4 N at full travel; an estimate); its tongue overlaps the frame's lip
by 6 mm. The lid is fixed open (no hinge). The lower plate's sloped spout is not modelled, and the drip tray is
straight, not curved, and empty. The real latch is inside the plate's rim; here it is a tongue under the plate's edge,
hidden by the plate.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. It is
computed every control step from PhysX contact forces between each plate, each handle and the drip tray and every link
of both arms. There is no run-wide zero; a plate whose centre ever falls faster than 0.77 m/s (a free fall of 3 cm) gets
no credit, and nothing else is latched.

- Each plate, 0.15: taken off by its handles: its centre 3 cm or more from its seat while each of its two handles is
  pressed by a gripper with 1 N or more. +0.20 (0.35 in all): it then lies flat (within 10 degrees) on the drying mat,
  or on the other plate lying there, its centre over the mat, its lowest point within 6 mm of what it lies on, and no
  arm touches it or its handles (0.1 N).
- Drip tray, 0.20: slid out (its back corners and back middle 17.5 cm or more from the body's centre, i.e. clear of the
  body), resting on the table (lowest point within 4 mm, tilted under 10 degrees) and untouched.
- 1.0: all of that, no arm touching any plate, handle or the tray, and everything still (under 1 mm in 1 s), for 1 s.
  Otherwise the sum, at most 0.9. Doing nothing scores 0.

`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=take_the_plates_off_a_contact_grill.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Take both cooking plates off the grill and put them on the blue drying mat, and take the drip tray out from under the grill." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cgrill-dream
```

The scripted demo, which reaches full score (both grippers come in level from the front, pinch the upper plate's two
handles, pull them apart and draw the plate forward out of the lid, carry it over the mat while turning it face up, lay
it flat and let go; both come down on the lower plate's handles, pull them apart, lift the plate 5 cm, carry it over the
mat and lay it on the upper plate; the left gripper pinches the drip tray's front lip, slides it 5.5 cm out from under
the body and lets go):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=take_the_plates_off_a_contact_grill.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Take both cooking plates off the grill and put them on the blue drying mat, and take the drip tray out from under the grill." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/cgrill-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cgrill-dream/*.json; do inspect-robots video "$log"; done
```

`CGRILL_DEBUG=1` prints, every 5 steps (`CGRILL_DEBUG_EVERY=10` for every 10), for each plate where its centre is, how
far it has moved from its seat, its tilt, its lowest and highest points, how far each handle is pulled out, each arm's
contact force on each handle and on the plate, whether it is held by both handles, taken off, dropped or placed; for
the drip tray where it is, how far its back is from the body's centre, its tilt and contact force; and where each
gripper is.

`CGRILL_MODE=one` runs the demo the wrong way, with the left arm alone: it pinches the upper plate's left handle, pulls
it out and draws the plate forward (the right latch holds: the plate swings out about 3 degrees and stays in the lid),
lets go, then pulls the lower plate's left handle out and lifts (the plate tilts about 9.5 degrees on its right latch,
its left edge 5 cm up, and drops back when let go). Neither plate comes off, and the run scores 0.

## Runtime per trial

On one RTX PRO 6000 (Blackwell), on a shared machine: about 9.5 minutes for the scripted demo while rendering
1280 x 720 video (it succeeds at step 1,138, 57 s of simulated time; run with `--max-steps 1200`), and about 16.5
minutes for a Dream-1 trial of 2,400 steps (the policy's round trips were slow, about 1 s each). Isaac Sim's first
start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A contact grill with removable plates and release handles on both sides (George Foreman 360 Grill GRP106QPG or a
similar model), cold and unplugged, its drip tray, and a dish-drying mat. Whether both handles were held out when each
plate came off, and whether a plate was dropped, is judged from the overhead and wrist videos; flatness with a phone
level app; the drip tray's position with a ruler.
