# Raise a loaded sit-stand desk converter with both handles

A VariDesk Pro Plus 30 sit-stand converter stands on the table with its keyboard tray toward two YAM arms. A monitor
is on its top and a keyboard on its tray, and it is locked at its fourth setting. The robot must take the handle at
each side and squeeze both release paddles together. It then lifts the top along its arc to the ninth setting,
releases both paddles there so that both locks drop into the same notch, lets go and moves back.

## Why both arms are needed, at the same time

The maker's patent for this mechanism (US 10,413,053, Varidesk LLC) gives the converter one locking set per side.
Each set is a handle that pulls an anchor out of one of a row of perforations in that side's pivot arm, and it
releases only its own side. The work surface is one rigid platform joining both arm sets, so one anchor left in holds the
whole top. Instruction sheets and reviews describe the same use: squeeze both handles, lift, release at the chosen
height so it locks. The paddles are 70 cm apart, so one gripper cannot reach both. The decisive moments come from
the two hands together: the top moves only while both paddles are held, and where it locks depends on when each
hand releases. A hand that releases early drops its pin into the next notch the top meets, and the top stops
there.

Dry-run controls (`tools/dryrun_demo.py`; the same variants run in Isaac with `--variant`):

| Control | What happens | Score |
| --- | --- | --- |
| `one_paddle`: the right hand goes to its handle but never squeezes | the right pin stays in notch 4; the left hand's lift is stopped by it; the top never moves | 0 |
| `release_mismatch`: the left hand lets go of its paddle at setting 7.6 while the right lifts on | the left pin rides on the teeth and drops into notch 8; the top stops there and both locks end in notch 8 | 0.5 |
| nominal: both squeeze, lift together, settle onto notch 9, release in the same step | locked at setting 9, load undisturbed, both hands clear | 1.0 |

With a rigid top, the two locks cannot end in different notches. On the real unit they can't either: that would
mean one side 47 mm higher than the other. A mismatched release instead stops the top short at the wrong notch,
and that is how this task scores it.

## Folder contents

| Path | What it is |
| --- | --- |
| `task.json` | the task card |
| `rdb_desk_converter/layout.py` | every dimension, mass and force, each marked published, derived or estimate; the arc, spring and lock geometry |
| `rdb_desk_converter/desk_converter.usda` | the scene asset, generated from `layout.py` by `tools/build_assets.py` |
| `rdb_desk_converter/fixed_scene.py` | placement, instruction, run limit, start state |
| `rdb_desk_converter/grading.py` | the scorer (NumPy only) |
| `rdb_desk_converter/adapter.py` | adds the converter to the stock Inspect Robots YAM station, logs diagnostics, feeds the scorer |
| `rdb_desk_converter/scripted.py` | the scripted demonstration (closed loop on joint readings, arm angle and latch heights) |
| `rdb_desk_converter/run.py` | runs one trial, writes `RESULT.json`, videos and the scorer trace |
| `tools/build_assets.py`, `tools/preflight_usd.py` | the asset generator; the pxr preflight (composition, joints, spawn poses, interpenetration, lock geometry over the travel) |
| `tools/kinematic_world.py`, `tools/dryrun_demo.py` | kinematic stand-in and the dry run through the real scorer |
| `tests/` | unit tests |

## The product, and where its numbers come from

| Quantity | Value used | Source | Status |
| --- | --- | --- | --- |
| Product | VariDesk Pro Plus 30, Vari | vari.com product page (DC-PP30), read 2026-11-18 | published |
| Surface | 30 x 29.75 in (762 x 756 mm) | vari.com spec table | published |
| Base | 24 x 24 in (610 x 610 mm) | vari.com dimension captions | published |
| Total depth | 41.75 in (1060 mm) | vari.com dimension caption | published; read as the depth at the top setting (derived) |
| Height range | 4.5 to 17.5 in work surface | spec sheet quoted by the Work While Walking review | published |
| Settings | 11 | vari.com, reviews | published |
| Mass, rated load | 46.39 lb (21.04 kg); 35 lb (15.9 kg) | vari.com spec table | published |
| Lift | "spring-assisted lift", "dual-handle design", "rowing lift" | vari.com | published |
| Keyboard tray | full width, 12.5 in deep at its middle, 3.5 in under the work surface | Work While Walking review | published |
| How the release works | one locking set per side: a handle pulls a sprung anchor out of perforations in that side's pivot arm; parallel pivot arms keep the platform level; it moves forward as it rises | US 10,413,053 B2 (Varidesk LLC), read on Google Patents; levers "on either side of the platform" (PCWorld review); "two levers on the side" (Impulse Gamer review) | published |
| Both squeezed to move | "Both handles must be held in at the same time in order to release the latch" | wikiHow, "Lower a Varidesk" (seen as a search-result summary only) | published (secondary) |
| Lift effort | "lifts with little effort thanks to spring assistance", "lowering it takes some muscle, especially to get it in the bottom-most slot" (PCWorld); weight "largely offset by the spring", depending on the monitor (Impulse Gamer) | reviews | qualitative only; no force figure is published |
| Arc | 367 mm arm, 5.0 to 80.4 degrees, settings 7.54 degrees apart | from the published rise (331 mm) and forward travel (305 mm) | derived; first-setting angle and equal spacing are estimates |
| Handles and paddles | paddle 70 x 37 x 10 mm, 20 mm under the 19 mm board, centred 45 mm behind the upper tier's front edge, centre 29 mm in from the side edge; 16 mm travel; 3.3 N to start, 12 N at full squeeze | product photographs | estimate |
| Lock | pin 10 mm into one of 11 radial slots, 93 to 118 mm from the arm's upper pivot; teeth 5 mm, slots about 8.4 mm | patent figure layout | estimate |
| Counterbalance | torsion spring 48.9 N m/rad on the lift; the hands push 3 N at setting 4, 11.3 N at most up to setting 9 and 2 N at setting 10 along the path, both hands together; the spring outweighs the load below setting 4 | chosen to match the reviews' description | estimate |
| Mass split | base 6.0, top 12.2, primary arms 1.9, secondary arms 0.5, latches 0.15 each (kg) | adds up to the published 21 kg | estimate |
| Load | 24 in monitor on its stand, 4.0 kg (typical listed 3.5 to 4.5 kg); full-size wired keyboard, 0.6 kg | task definition | - |

How to measure the estimates on a real unit (all with the stated load on it):

* Net lift force: hook a luggage scale under each handle with both paddles squeezed. Read the steady hold force
  at settings 4, 6.5 and 9. The model expects 1.5 N, 5.7 N and 3.3 N per handle.
* Paddle force: push a kitchen or luggage scale up under a paddle until the top is free.
* Handle geometry and paddle travel: ruler.
* Notch spacing: from the surface height at each setting (the model's heights are in `task.json`).

No product image, mesh or logo is included. The display shapes are drawn from the dimensions above.

## The model

One fixed-base articulation, a tree with no loop joints:

```
base --lift (revolute; torsion spring)--> lift_arm --level (revolute; mimic: level = -lift)--> top
base --rear (revolute; mimic: rear = lift; display only)--> rear_arm
top --latch_left_slide / latch_right_slide (prismatic; sprung down)--> latch_left / latch_right
```

* Lift and level: a real parallelogram is a closed loop. Its second arm is replaced by a mimic coupling that turns
  the top back by the arm's angle, so the top stays level and translates along the arc of the 367 mm arm, exactly
  as on the real parallelogram. The lift joint's travel is limited to settings 3 to 11 (the real unit goes to
  setting 1).
* Counterbalance: a linear torsion spring on the lift joint (a stated force curve, `layout.net_push`). It acts
  on the arm, so the hands feel it only through the top.
* Locks: each latch slides vertically in the top. It carries a paddle collider under its handle and a pin collider
  that drops 10 mm into one of 11 radial slots between steel teeth on that side's lift arm, around the arm's upper
  pivot. Squeezing a paddle 14 mm or more lifts its pin clear. A released pin rides on the tooth ends until a slot
  passes under it. Lock, jam and release come from these contacts only.
* Slot play: an engaged pin has 1.1 mm of side play in its slot (0.0117 rad of arm angle, `layout.SEAT_PLAY`,
  computed from the same boxes as the asset). The loaded top rests on the low side, so notch k reads about
  k - 0.089. The asset is authored in that seated pose, so nothing moves at reset. Isaac run 1 was authored at the
  slot centre: the top settled by exactly 0.0117 rad, and the old reset check (0.1 setting, 4 mm) stopped the trial.
  The reset check now allows the whole play (0.198 settings, 10.6 mm). The scorer names a notch from up to 0.3 of a
  setting either side of its centre; seated latches are what prove a pin is in a slot.
* Fingers: the hand comes in level from the front with the jaws closing vertically. The upper pad lands on the
  board beside the edge, the lower pad goes under the paddle. Closing lifts the paddle to its stop and clamps the
  33 mm stack.
* Loads: free rigid bodies resting on the top and tray by friction (rubber feet on laminate).
* The base is fixed to the table. A 21 kg unit on rubber feet needs about 100 N to slide, far more than an arm
  gives.
* Colliders are invisible boxes. Display meshes are drawn over them: rounded black work surface and tray, handle
  mouldings, steel frame, foam-covered arms, the monitor (stand, panel, dark screen), the keyboard with its keys.
  The teeth, pins and the card are visible as they are.

`tools/preflight_usd.py` (pxr) composes the asset under `/World/envs/env_0/desk_converter_fixture` and checks:

* no duplicate sibling names, every relationship target present;
* one articulation root, six bodies, five joints plus the base weld, masses as declared;
* every joint's frames coincide at spawn, so all joints read 0, which is also the reset target;
* the top spawns seated in notch 4, the reset check's window covers the whole slot play, and no colliders
  interpenetrate at spawn;
* the lock geometry over the whole travel: a squeezed pin clears every tooth (1.95 mm), an engaged pin sits in every
  slot from 3 to 11 with 1.1 mm either side at the slot's centre, and just touches the low tooth when seated.

## Reach

Every grasp point from setting 4 to 9.6 was checked with inverse kinematics from nine seeds, for both arms:

* both arms reach all of them, with at least 30 degrees of joint margin (the limiting joint is wrist roll);
* collision hulls clear the tray by at least 12 mm at the grasps up to setting 9.3 (9 mm at the pre-grasp, fingers
  open), the
  monitor by more than 200 mm, and the other arm by 408 mm;
* along the whole kinematic dry run of the demonstration (hulls sampled every 8 steps), the arms stay at least
  10.4 mm from the tray, 48 mm from the keyboard, 258 mm from the monitor, and 240 mm from each other. The
  demonstration lifts to setting 9.25, settles back onto notch 9, backs out, rises and stays there (a joint-space
  return to READY would sweep the arms through the raised tray).

The grasp points run from (470, +-352, 232) at the seated start to (300, +-352, 398) at setting 9, each 48 mm behind
the board's front edge. That depth is set by the YAM fingers: with the jaws open their roots, 61.5 mm behind the
fingertip centre, span the board's thickness (`tests/test_hands.py` checks it on the URDF collision hulls). Isaac run
2 had the grasp 80 mm in; the hands stopped 18.5 mm short against the board's edge, pushed the top with about 100 N
each, never touched the paddles, and the top never moved. Only the converter's
place on the table was chosen for reach; it is at its true size.

## What the model sees

The policy gets the overhead camera, one camera on each wrist (all 320 x 176 RGB) and the joint and jaw positions.

* Overhead: the converter's black work surface (about 250 to 300 pixels wide), the tray, the keyboard, the monitor and
  the green card are all in view from the first frame.
* Wrist cameras: the handle, the paddle and the board's edge under each hand.

A review-only camera (1280 x 704, never a policy input) looks across both handles from the robot's left front.

## Scoring

`grading.py` reads, every control step:

* the setting, from the lift joint's angle;
* each lock, seated when its latch is within 2 mm of its lowest position, which needs its pin inside a notch;
* the loads' displacement and tilt relative to the top;
* whether any arm link touches the converter or its load, and each fingertip centre's distance from its handle.

Tiers (score at the end; 0.2 is earned once and kept):

| Score | Condition |
| --- | --- |
| 0.2 | at some moment the setting reading was 4.7 or more (setting 5 reached, allowing for the 0.09 seated play; with a lock in, the play allows at most 4.09) |
| 0.5 | at the end the setting reading is 6.7 or more (setting 7 or higher) |
| 0.8 | at the end both locks are seated, the reading is 8.7 to 9.3 (notch 9), and the top is level (sides within 5 mm) |
| 1.0 | also: monitor and keyboard each moved at most 30 mm and tilted at most 10 degrees, and both hands have been clear (no contact, 50 mm from the handles) for the last 1 s |
| 0 | latched if the monitor or keyboard falls (tilt past 45 degrees or off its surface) |

A real trial uses the same rules:

* setting: count the notches, or measure the work surface's height against the green card;
* locks, each side on its own (one seated lock alone holds the rigid top, so pressing the top does not test both):
  a side's lock is seated when its paddle has dropped fully back, 20 mm below the board (ruler, within 2 mm, the
  same 2 mm latch height the simulation reads); a pin left on a tooth holds its paddle 14 mm or more higher. To confirm,
  squeeze one paddle alone and press the top down: it must not move, because the other lock holds. Then swap sides;
* load: photograph it against marks made at the start;
* hands: watch the video.

## Running

```
pip install -e .            # in the Inspect Robots YAM environment
python -m rdb_desk_converter.run scripted --out runs/scripted
python -m rdb_desk_converter.run scripted --variant one_paddle --out runs/one_paddle
python -m rdb_desk_converter.run scripted --variant release_mismatch --out runs/mismatch
python -m rdb_desk_converter.run dream --out runs/dream
python tools/dryrun_demo.py                       # kinematic dry run, no simulator
python -m pytest tests
```

* Run limit: 2,400 actions (120 s at 20 Hz). The kinematic dry run of the demonstration ends at 570 steps.
* Seed: 0. The layout is fixed; nothing is randomised.
* `run.py scripted` exits 0 only on 1.0, or on the expected score of a control variant.
* `RESULT.json` holds the score, the final grade, `peak_interarm_contact_n` and the diagnostics. It is written
  even when a trial stops on an error.

Diagnostics in `RESULT.json`, for debugging only (no policy ever receives them):

* the state after reset;
* samples every 5 steps: setting, latch heights, mimic error, top position against the arc, top roll and pitch,
  fingertip positions against the grasp points, jaw command and reading, and contact forces per pair;
* a per-step contact trace: pad on paddle, pads on board, pin on teeth and arm touches, each from its own contact
  view;
* a per-step trace of commanded and measured joints;
* the demonstration's phases, events, trims and servo log.

## Results

| Run | Policy | Score | Steps | What happened |
| --- | --- | --- | --- | --- |
| 4 | Scripted demonstration | 1.0 | 575 of 2,400 (142 s wall time) | squeeze at step 220 (both latches 16.0 mm, about 22.5 N per pad); lift to setting 9.18; settled; both paddles released in one step; both locks seated in notch 9 (reads 8.911); monitor moved 0.0 mm, keyboard 0.1 mm; hands 94 mm from the handles, no contact; peak arm-to-arm contact 0 N |
| 4 | Dream-1 (`dreamlabs/dream-1-beta`) | 0.0 | 2,400 of 2,400 (469 s, 54 requests) | the arms moved above and in front of the converter and never touched it; both locks stayed in notch 4 |

Earlier scripted runs, each fixed before the next (the package files of run 4 are the ones in this folder; the
RESULT.json hashes of scene, asset, layout and scorer match them):

* **Run 1: no score, stopped at reset.** The asset was authored with the pins at the centre of their slots. The
  loaded top settled 0.0117 rad onto the slot play, and the reset check (0.1 setting, 4 mm) stopped the trial. Fix:
  the asset is now authored in the seated pose, and the reset check allows the whole play.
* **Run 2: 0.0.** The grasp was 80 mm behind the board's front edge. The open finger roots met the edge 18.5 mm short,
  the pads never reached the paddles, and the top never moved. Fix: grasp 48 mm in (`tests/test_hands.py`), trims
  capped, a slight lift while squeezing.
* **Run 3: 0.8.** Locked at notch 9 with the load undisturbed. The last retreat step changed IK branch, and joint 1
  swung the arm into the converter, so "clear" never held. Fix: that step removed, joint jumps over 0.15 rad per
  step refused, lift tolerance relaxed.

## Licences

All code and the generated asset in this folder: MIT (see `LICENSE`). No third-party meshes, textures or images
are used.
