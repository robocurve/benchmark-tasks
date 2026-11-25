# Fasten a belt at its third hole

Fasten the belt at its third hole, counting from the tip: put the buckle's prong through the third hole so that it
rests on the buckle's front bar, then let go of the belt.

Nearly everyone fastens a belt every day. Here a leather belt with a single-prong frame buckle runs through the
loops of a pair of jeans on a lower-torso display form lying on its back. The belt is through the buckle's frame but
not fastened: no hole is on the prong, which lies turned back under the strap. The robot has to bring the third hole
to the prong, get the prong up through it from the body side, and bring the prong's tip onto the frame's front bar
with the strap under that bar. The strap moves whenever it is pulled, so the hole must be found by counting from the
tip and kept where it is while the prong goes through.

## Files

| File | What it is |
|---|---|
| `task.py` | the task file for `yam_arena`: scene, scorer (`score`, `success`), instruction |
| `geom.py` | every size of the form, jeans, belt and buckle, and the start layouts (pure NumPy) |
| `beltkit.py` | the belt as one articulation: 38 strap links with sprung joints, real holes, frame and prong on the hinge bar, keeper |
| `form.py` | the form and the jeans with their five belt loops (one fixed part) |
| `partkit.py` | helper: multi-box rigid parts and per-step hooks for `yam_arena` task files |
| `measure.py` | what the scorer reads from the belt's poses (hole engaged, prong on the bar, strap under the bar, loops, keeper) |
| `demo.py` | the scripted demo (acts only through the arms' joint and gripper commands) |
| `make_layout.py`, `threaded_layout_*.json` | the three start layouts (made offline) |
| `test_offline.py`, `test_fixtures.json` | offline tests of the scorer on recorded belt states |
| `BUILD.md` | the real parts, how to set the scene up and reset it |

## Run

In the Isaac Lab-Arena + YAM container, from this folder:

```sh
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the belt at its third hole, counting from the tip: put the buckle's prong through the third hole so that it rests on the buckle's front bar, then let go of the belt." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>
```

Scripted demo (same scene, scorer and limits; the live scorer prints its verdict):

```sh
python demo.py task.py demo_out --seed 0      # --seed 1, --seed 2 for the other start layouts
```

Offline tests (no simulator; needs NumPy and the `yam_arena` package importable):

```sh
python test_offline.py
```

## Runtime, seeds and start layouts

One trial is at most 2,400 control steps (2 minutes at 20 Hz); the demo needs 700-800 steps. The reset seed
selects one of three start layouts (seed modulo 3): the first hole 7, 7.5 or 8 cm past the hinge bar, with the
buckle on the top centre line, 8 mm to the robot's right or 8 mm to its left. The environment variable
`BELT_START=0|1|2` overrides the seed. Within a layout the start is fixed (`object_jitter=0`). The layouts differ
by 5-10 mm of strap drawn through the buckle and by up to 16 mm of buckle position; the demo reads the belt's poses
and does not replay one layout's actions on another.

Scripted demo results in the official evaluation environment, same code, one run per layout: start layout 0 scored
0.25 (883 steps: the prong through the third hole from the body side, but its tip 8 mm from the front bar, limit
6 mm, so the pull test never started); start layout 1 scored 1.0 (776 steps); start layout 2 scored 1.0 (696
steps). On a second machine with the same code, start layout 0 scored 1.0 (740 steps). The demo's outcome is
sensitive at the millimetre level. The published demo video is the start layout 1 run.

## Scoring (what the code does, and how to judge a real trial)

The score is the state at the end of the run. Hands off means no gripper point within 15 mm of the belt or the
buckle frame for 1 s (in a real trial: both hands visibly clear of the belt in the video).

Fastened at hole 3 means:

1. the prong passes through hole 3 (counted from the tip) from the body side outward, and its tip rests on the front
   bar (tip within 6 mm of the bar): a close photo shows the tip out of the hole on the strap's outer face and on
   the bar;
2. the strap passes under the front bar and out through the frame, with the bar 0 to 15 mm off the strap's outer
   face (ruler);
3. the belt is in all five loops.

Pull test. The pull is a stimulus the judge applies; the pass condition is a displacement, not a force reading. In
simulation, once the hands have been off for 1 s and the belt is fastened at hole 3, the scorer applies 10 N to the
strap link nearest the right hip, straight out from the form, for 1 s, and reads the result 0.5 s later. In a real
trial: put a tape mark on the strap at the front bar, hook a luggage or spring scale under the belt at the right hip,
pull straight out to 1 kg for about 1 s, let go and look. The belt holds the pull if afterwards the prong is still
through hole 3 from the body side with its tip back on the front bar, the bar is 0 to 15 mm off the strap, the tape
mark moved less than 10 mm through the buckle, and the belt is in all five loops. Hands off, pull and look take
about 2.5 s, so the belt must be released at least 2.5 s before the run limit for full score.

| Score | State |
|---|---|
| 1.0 | hands off, fastened at hole 3, and the pull held |
| 0.25 | hands off, prong through hole 3 from the body side, strap under the front bar (bar 0 to 15 mm off), belt in all five loops, but the prong's tip not on the bar or the pull not held |
| 0 | anything else: any other hole (even fastened), the start state, a prong beside a hole, the belt out of a loop, a gripper within 15 mm of the belt at the end, anything that cannot be measured |

A wrong hole earns nothing: hole 2 is looser than hole 3 and therefore easier, so credit for it would reward the
wrong action. The frame does not have to lie flat: with hands off, a loosely fastened belt on a rigid form stands its
frame 20-30 degrees off the strap (the strap's stiffness lifts it; a pull lays it flat and it rises again). What
matters for a fastened belt, and what the photo shows, is the prong through the hole on the bar and the strap under
the bar. The scorer fails closed: anything it cannot measure counts as not done, and a gripper touching the belt
resets the hands-off clock, cancels any test in progress and sets the score to 0.

## Why the slack, and why the belt starts threaded

The belt is a size 29 (fold to middle hole) on a form whose waist measures 26.7 in over the jeans: one size up,
fastened at its middle hole, about 6 cm loose. To put a prong through a hole, the hole must reach past the prong's
tip and the prong must swing down, which takes 2-5 cm of spare strap; a wearer provides it by drawing in their
waist, a rigid form gives nothing, so the spare strap is in the belt. The belt starts through the frame because the
task is the fastening at a chosen hole, and a belt left in the trousers is normally still through its buckle.

## Cheap strategies (recorded runs of this folder's code, start layout 0)

| Strategy | Score | What the scorer saw |
|---|---|---|
| do nothing (demo variant, 200 steps; and the official CLI with `--policy yam_scripted`, 300 steps) | 0 | start state: no hole on the prong |
| pull the tail through and let go | 0 | no hole on the prong; the strap slid back 24 mm on release |
| get the prong through hole 3 but leave it standing, hands off | 0 | hole 3 engaged, prong 37 mm from the bar, frame at 78 degrees with the bar 18 mm off the strap |
| fasten correctly but keep the tail in the gripper | 0 | fastened, but a gripper on the belt |
| aim the script at hole 2 instead of hole 3 | 0 | the prong did not go through hole 2 |
| aim the script at hole 4 instead of hole 3 | 0 | hole 4 engaged, prong tip 10 mm from the bar (a wrong hole scores 0 in any case) |

## Fidelity

The belt is one size larger than the form's waist and is fastened at its middle hole, so it is about 6 cm loose
at hole 3 on a rigid form. It starts threaded through the frame, and the tail is not tucked under the keeper at the
end. With hands off, the loose belt stands its frame 20-30 degrees off the strap. The strap is a chain of 38 rigid
25 mm links joined by sprung joints (flat bending 0.16 N m per radian per joint,
from an estimated leather stiffness EI of 0.004 N m^2; twist and edge bending stiffer). Each of the five holes is a
real opening that the prong passes through and bears on. The frame and the prong hinge freely on the bar inside the
belt's fold (hinge friction estimated). Nothing snaps, attaches or fires on proximity; the robot's finger friction
alone holds the strap. The jeans are a rigid denim shell over the rigid form, with rigid belt loops of the stated
room. The form's front does not compress.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at bd62771e60320433893bd1d2df0fc79b5ddcd6d1, the program's official dependency,
installed by the user's environment. Nothing from it is redistributed here. Licenses are given as each source
states them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, see the SDK's `LICENSE.i2rt` | no |
| link2/link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor, table, top-camera stand, lights and cameras | room and policy cameras | SDK `station.py`/`scene.py`: shapes generated in code; no mesh or texture file loaded | SDK README: "This repository has no license yet" (Placeholder) | no |
| Station fruit, bowl and plate (Arena object library) | not used: this task's own objects replace them | - | - | no |
| Isaac Lab-Arena | simulation framework | NVIDIA Isaac Lab-Arena | Apache License 2.0 | no |
| Form, jeans, belt, buckle, keeper geometry; scorer; demo | the task | this folder (all objects generated in code from the sizes in `geom.py`) | authors' own, MIT (see LICENSE) | yes |

Isaac Sim and Isaac Lab are runtime dependencies under their own licenses; none of them is redistributed here.
