# Fix a hook tight to a hollow wall with its toggle bolt

A short section of an installed hollow wall stands upright on the bench, its face toward the robot. A steel coat
hook hangs loose at a hole low on it, on a 3/16 in (10-24) spring-wing toggle bolt whose wings have already opened in
the closed cavity behind the board. About 2.8 turns of thread are left to take up. The toggle nut cannot be reached:
turned on its own it just spins with the screw. It only holds, by friction of its wings on the back of the board,
while the hook is pulled out from the wall. So one arm has to keep pulling the hook out (and plumb on the pencil
line) while the other turns the screw's hex washer head with the ratcheting nut driver from the holder.

## How this differs from hold-and-turn tasks

Where one hand holds a part while the other turns a fastener, the holding hand grips the part that would otherwise turn
and takes its torque. Here the part that would turn is the toggle nut, behind the board and out of reach; nothing grips
it. It stops turning only by friction of its wings on the back of the board, and that friction exists only while the
other hand pulls the hook straight out from the wall, against the nut driver's push on the screw head. The pulling hand
takes no torque: it keeps an outward pull on the hook through the whole tightening and keeps the plate plumb, without
seeing whether the wings bear.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. Two helper files
must sit next to `task.py`, which imports them:
- `parts.py`: the parts kit (multi-colour parts with joints), in a version extended for this task (see below).
- `tilt.py`: lets the scripted demo lean a gripper; it affects only the scripted demo, never Dream-1 or scoring.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fix the hook tight against the wall, square to the pencil line."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/dream --epochs 2

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/demo --epochs 2
```

The layout is fixed (there are no layout lots), so `--epochs N` repeats the same trial N times and averages the
scores; the submitted runs used `--epochs 2`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 2 to 3 minutes of wall time on
  an L40S once Isaac Sim has started. The demo script lasts about 53 s (it waits clear while the tug test runs); in
  the submitted demo run each trial ended on success after 924 control steps (46.2 s).
- **Seeds:** submitted runs use `-E object_jitter=0`; the layout is fixed. The hook and the nut driver are the loose
  objects; with jitter the bolt would start off the bottom of the hole (it still sits in the hole's play).

## The setting

The wall section is 20.5 cm wide and 30 cm tall: a 2x3 frame (bottom plate on the bench, top plate, two studs) with
12.7 mm wallboard on both sides, so the cavity is closed as in a finished wall. The hole is 15 cm up, low on the
section, so both arms work at or below 20 cm above the bench. The wall is plumb (no lean). The plumb pencil line runs
down through the hole; the hook hangs below its bolt, its 75 mm round arm standing straight out from the wall.

## The mechanism

- **Thread:** the toggle nut rides the bolt on a helical joint: a slider (`thread`) on the bolt, and the nut's
  hinge (`nut`) on that slider, tied by a PhysX mimic joint so the nut travels 1.06 mm along the bolt per turn
  relative to it (10-24, 24 threads per inch). The bolt turns in the plate on a hinge (`bolt`, 0.002 N m s/rad drag, no
  drive). The nut's hinge has 0.002 N m s/rad drag (thread friction), so a nut that nothing holds
  turns with the screw. Nothing switches between "spins" and "holds": the nut holds only when contact friction of its
  wings on the board's back, which exists only while the hook is pulled out from the wall, outweighs that drag.
- **Thread lock:** a real 10-24 thread is self-locking: a tightened screw does not turn back under the clamp load.
  The simulated thread has only velocity drag, so under load it slowly backs off (seen on the GPU: a toggle drawn
  up 0.7 mm lost it again within seconds). So an `on_step` callback (`_thread_lock`) moves the thread joint's lower
  limit up to the furthest the nut has been drawn along the bolt, less 0.02 mm (a callback before each reset puts
  the limit back, and the nut driver's ratchet limit and shaft angle, so every trial starts the same). Turning the screw forward is
  unaffected, and a toggle spinning with the screw does not move along it, so it changes nothing about when the
  toggle grips. The consequence: in the simulation the screw cannot be loosened again (the ratcheting driver only
  tightens anyway). The thread's upper limit is 0.5 mm past the board's back, which bounds the clamp force.
- **Loose hook:** the 12.7 mm hole is wider than the 4.8 mm bolt, so the loose hook hangs on the bottom of the hole,
  3.7 mm low, flat against the face. Only a tug tells it from a tight one.
- **Screw head:** spring-wing toggle bolts usually ship with mushroom or round combination (slotted and Phillips)
  heads. The wings are also sold on their own (for example Lawson part 90979, "Toggle Bolt Anchor Wing, 10-24",
  https://www.lawsonproducts.com/en-ca/products/toggle-bolt-anchor-wing-10-24-90979) and fit any 10-24 machine screw.
  This task uses a 10-24 hex washer head screw so that a nut driver can turn it: a 5/16 in (7.9 mm) hex 2.4 mm tall on
  a washer face 10.3 mm across and 0.6 mm thick, 3.0 mm (0.118 in) overall, within the 0.105 to 0.120 in head height
  of ASME B18.6.3 #10 indented hex washer heads.
- **Nut driver and the socket rule:** a stubby ratcheting 5/16 in nut driver. Its socket (13 mm across, 10 mm long)
  has a round-cornered bore 10 mm across and 4 mm deep, with a 45 degree lead-in opening it to 12 mm at the mouth
  (tilted boxes), so it slides over the hex's 9.2 mm corners from up to about 1.4 mm off centre and stops on the
  washer face. A round bore cannot grip a hex, so a rule stands in for the hex's grip (`_socket_drive`): while the
  socket is seated (its axis within 12 degrees of the bolt's, its mouth within 1.5 mm of the bolt's axis and at least
  2 mm down over the hex), a torque on the bolt makes it follow the socket's clockwise (tightening) turn (a stiff
  spring, 3 N m/rad, from the bolt to where the socket has turned it, capped at 0.35 N m, past which the socket
  slips), and the equal and opposite torque is applied to the driver's handle every control step, so the hand on the
  driver feels the screw's resistance (the ratchet's pawl ties socket and handle together while driving). The
  reaction goes on the handle rather than the 15 g socket link: applied to the socket link it set the freewheeling
  link spinning in a GPU trace. The anticlockwise back swing does not drive the screw back. Off the head nothing links the screw to anything but the plate's light drag (0.002 N m s/rad):
  there is no joint drive on the bolt. I did not use a real hex bore: in earlier builds a Phillips tip pressed on a
  cross recess never dropped into it as it turned, and a hex socket with 0.2 mm clearance needs the same rotational
  alignment.
- **The screw's resistance** (`resist_torque`): a 10-24 screw drawing a toggle snug takes about 0.3 N m and very little
  while loose. The simulated thread has no friction, so any torque past snug would turn into an axial force far beyond
  a real one. A torque against the socket's stands in for thread and head friction: nothing while the nut is short of
  the point where a flush plate meets the wings, and from 0.1 mm before it, all of the socket's torque but 0.05 N m
  (which draws the clamp up to about 300 N through the frictionless thread, as a snug 0.3 N m does on a real one). Its
  reaction goes onto the plate, as head friction would. So the driver hand turns against very little while the toggle
  is loose and up to the 0.35 N m cap once it is snug (in the demo run's log: 0.35 N m on the handle, 0.30 N m of it
  taken by the friction torque).
- **Names in the code:** the nut driver is the object `screwdriver` in `task.py` (its socket and shaft are the link
  `shaft`).
- **Ratchet:** the socket and shaft are a link on a hinge in the handle. Its upper joint limit is the ratchet pawl:
  every control step an `on_step` callback lowers it to where the shaft has been (plus 0.02 rad play), so the shaft
  can turn anticlockwise in the handle (the back swing) but drives the screw clockwise.
- **Board:** the wall's contact material is slightly springy (5 x 10^4 N/m, 60 N s/m, PhysX compliant contact), as
  wallboard paper is, so a screw turned past tight builds a bounded clamp force instead of an impulse.
- **`parts.py` changes** from the stock kit: joints may hang from another joint's link (`parent=`); a slider may be
  tied to a hinge by a thread (`thread=(hinge, lead)`, written as `PhysxMimicJointAPI`); shapes may be turned about
  the part's own z axis (`yaw=`, used for the octagonal hole and bore) or have no collider (`collide=False`, the pencil
  line and the thread slider's marker); a part may set its contact offset, turn link-to-link collision off, use a
  compliant material, and start turned by any rotation (`quat=`: the wall and the hook stand upright);
  `on_pre_reset` callbacks run just before a reset.

## Scoring as implemented

`score()` and `success()` in `task.py`. Every check is one a fitter makes from in front of the wall with simple
tools: a 1 mm feeler gauge, a spring scale, a pencil and a protractor. Nothing hidden in the cavity is measured.

- **Flush** (`_flush`): a 1 mm feeler gauge cannot go under any edge of the plate: each corner of the plate's back is
  less than 1 mm from the wall's face.
- **Square** (`_off_plumb_deg`): the angle between the plate's long axis (bolt end to hook end) and straight down (the
  plumb pencil line): at most 3 degrees for 1.0, at most 10 for 0.5. A hook upside down fails.
- **Let go and still:** an `on_step` callback measures every control step how close both arms' fingers and grippers
  come to every shape of the hook (all links). It samples every face of each collision hull about 3 mm apart, not
  only the hull's vertices, because a hull face can rest on a part while its nearest vertex is centimetres away. All
  must be more than 5 mm away (the nut driver may stay in a hand). Until it has measured, or if it cannot read all
  six hand links, nothing scores (fail closed). The nut driver's box must also be at least 5 mm from every shape of
  the hook. `near_misses.py` checks offline that a gripper face 0.5 mm from the plate's edge, with its nearest hull
  vertex 11 mm away, counts as not let go. The hook must be at rest (under 1 cm/s and 0.5 rad/s) for 1 s before the test.
- **Tug test** (`_pull_step`, `_apply_pull`): once the robot has handled the hook (a gripper has come within 5 mm of
  it at some step; the scorer does not test the untouched hook) and the hook is flush, within 10 degrees of plumb, let
  go, clear of the nut driver and still for 1 s (20 control steps), the scorer tugs it: an `on_step` callback applies a steady 20 N
  force (a spring scale reading of about 2 kg) straight out from the wall on the hook's arm, just inside its
  upturned end, for 10 control steps (0.5 s), then removes it for 10 more (0.5 s). Tight means no corner of the
  plate came 1 mm off the face at any step of the tug, and afterwards the centres of the plate's two ends are each
  within 1 mm of where they were before it (as pencil ticks at the ends would show) and the hook is at rest. Fail
  closed: the callback records the force it set, and the test fails unless 20 N was applied on at least 9 of the 10
  pull steps. A gripper within 5 mm or the nut driver within 5 mm at any step of the test, or the hook moved after
  it, starts the test over (after a new 1 s at rest). A loose hook hanging flat, with play left on the thread, comes off the
  wall under the tug until its wings meet the back of the board, and fails. Each finished
  test prints one line to the run log (`[tug test] max gap ... mm, moved ... mm`).
- **Credit:** 1.0 when the last test passed and the hook is still flush, square, let go, at rest and within 1 mm of
  where the test left it; 0.5 the same at 3 to 10 degrees; 0 otherwise, including a trial that ends before a test is
  finished. Success needs the same 1.0 state and must hold 1 s.
- **Reset:** the 1 s count, the "handled" record and the test start over at every scene reset.
- **Checked on the GPU** (this task.py, with `HOOK_DEBUG=1` printing the state every 0.5 s): in the demo run the hook
  drawn tight passed the tug test in both trials (the plate edge came 0.283 and 0.345 mm off the face; the plate's
  ends moved 0.042 and 0.005 mm) and ended 0.0 and 0.1 degrees off plumb.
- **Control runs** (the demo with `HOOK_CONTROL` set in the environment, one trial each; `demo()` reads it only for
  these runs, and it is never set for the demo or Dream-1):
  - `HOOK_CONTROL=nopull`, the screw driven with nobody pulling the hook: the socket sat on the head for about 26 s
    and turned the screw 40 turns (to its hinge's limit), while the nut moved at most 0.2 mm along it: the toggle
    spun with the screw. The hook swung round with it, never tightened, and the trial scored 0.
  - `HOOK_CONTROL=crank`, no tool: the left hand pulled the hook out (the wings bearing on the board) and cranked it
    to and fro round its bolt four times, up to 24 degrees off plumb: the nut did not move along the thread
    (0.0 mm, as logged to 0.1 mm), and the trial scored 0.
  Both control runs used task.py as it was just before its final change (the friction torque's form), which acts only
  while the socket is seated on a snug screw, so neither control reaches it.
- **Earlier runs** of the version before (a weaker socket rule): a hook pressed flat with play left on the thread came
  2.9 mm off the face under the tug and failed.
- `near_misses.py` checks every tier and boundary (2.9/3.1/10/10.5 degrees off plumb, upside down; a gap under the
  tug just under and at 1 mm, straight out or with the hook end rocking out; a slip of just under and just over 1 mm
  after the tug; 0.9 and 1.1 mm off the face; the hook end rocked 1.2 mm out), a hook hanging flat with its wings
  0.5 mm off the back of the board (rocks out under the tug: 0), half the thread taken up, the toggle spun with no
  travel, the tug never applied, the nut driver left on the head or held 4 mm in front of it, and the hook pulled off
  the wall and back. Offline there is no robot and no physics, so it sets the arms as measured
  clear, the hook as handled, and gives the hook's pose under the tug. It also checks that an unhandled hook is never
  tested (a tight, plumb hook then scores 0). Checked separately offline: with the arms' state unknown (`None`) the
  tight-and-square state scores 0.

## Scripted demo

`demo()`, fixed waypoints from the published start layout only (no object state is read):

- Right arm: picks the nut driver from its holder by the handle (pads 27 mm apart on 35 mm: a firm grip, as a looser
  one let the driver wobble when turned), lifts it, turns it level through three set poses on the way over (the wrist
  turns over between the straight-down grip and the level one), and brings it in level, socket toward the wall, over
  the hex of the sagging bolt, aimed 1.3 mm left of and 2.5 mm above its centre (the held socket was measured to settle
  that far right of and below where the arm is sent, at this pose). It presses 3 mm past seated, then makes eight
  200-degree clockwise strokes (wrist +110 to -90 degrees, the wrist's range there) with a ratchet back swing between
  them; once the toggle is tight the socket slips. It eases off, draws the driver 30 mm back and holds it there, clear
  of the head, while the tug test runs.
- Left arm: leans 60 degrees to the robot's right (`tilt.py`), so its wrist stays to the left of and clear of the
  right hand, and pinches the hook's round arm half way along it. Before the socket goes on, it pulls the arm 40 mm out
  from the wall and 5 mm to the robot's right (commanded; the soft arm moves the hook only until the wings meet the
  back of the board, and the sideways lean holds it plumb against the swing measured on the GPU), and keeps pulling
  while the right arm turns, then eases off and lets go once the driver is out, and moves away.

## Fidelity

True size, a short section of an installed wall standing plumb on the bench with the hole low on it. The thread is
an ideal helical joint with velocity drag standing in for thread friction; its self-locking is a stop that follows the
nut (see above), so the screw cannot be loosened in the simulation. The toggle starts already opened behind the board
with about 2.8 turns of thread left. The socket's grip on the hex is the rule described above (the robot must still
seat the socket and turn it). The board is rigid with a slightly springy surface, so a tight hook gives a little under
the 20 N tug (its plate edge came 0.28 and 0.35 mm off the face in the demo run), well inside the 1 mm gauge. No damage
to the board is modelled.

## Licences

All geometry is built from primitives in `task.py`; no third-party assets.
