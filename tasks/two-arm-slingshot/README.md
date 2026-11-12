# Two arms playing Angry Birds

The left YAM arm braces a toy slingshot; the right grasps its preloaded red ball,
pulls the pouch backward and opens its fingers. A passive native PhysX spring
accelerates the pouch and ball toward two towers. The task uses the existing
Isaac Lab/Arena Dream-compatible embodiment and robot joint controller.

## Status

The native scripted demonstration reaches **1.0 in 341 controls (17.05 s)** with no unsafe event. First-run wall time was 117.92 s.
Watch the [overhead demonstration](evidence/demo/top_cam.mp4)
or inspect the [setup](evidence/demo/setup-top_cam.png).
See [validation.md](validation.md) for receipts, controls and limitations.
A Dream-1 trial scored **0.0 in 1,200 controls** using the instruction below.
The model run, demonstration and setup image are uploaded under the media ID in task.json.

## Fixed setup

Coordinates are metres, x forward, y robot-left, z up. Seed 0; no jitter.
The existing station has two YAM arms at (0, ±0.305, 0.75) and a 1.10 × 1.30 m
tabletop at z=0.7475. geometry.json is the complete primitive inventory; box
sizes there are half extents. build.py regenerates all geometry and poses.

- Wooden-look free slingshot frame: origin (0.30, -0.05, 0.7475), mass 300 g.
  Base 140 × 190 × 18 mm; two 24 × 24 × 104 mm fork posts, 140 mm apart.
  Raised left grasp rail 126 × 18 × 18 mm, local center (0, 0.19, 0.10), on two
  supports joined to the fork by a crossbar. A center stem connects the base to
  the narrow horizontal guide; the pouch has a rigid U-shaped carriage around it.
- Rigid pouch: mass 25 g, 60 × 60 × 6 mm floor, 8 × 60 × 40 mm rear wall,
  two low side rails and a 6 mm high front lip above the floor. Rest origin
  (-0.04, 0, 0.13) relative to the frame.
  Passive prismatic spring: 45 N/m, 0.04 N s/m damping, fixed zero target,
  travel -180 to +25 mm. No actuator changes this spring during a run.
- One red ball: diameter 50 mm, mass 35 g, center (0.26, -0.05, 0.8775).
  Rigid foam-toy approximation, nominal friction 0.8/0.65 static/dynamic.
- Two towers at (x,y)=(0.61,-0.075) and (0.67,-0.075). Each has three free 30 × 70 × 50 mm
  blocks, 18 g each, centers z=0.7730, 0.8230, 0.8730. No glue or break events.
- Catch tray: 680 × 460 × 24 mm floor centered at (0.85, 0, 0.7360),
  two 24 mm thick, 300 mm high side walls and a 400 mm high back wall.
  The tray is a static extension of the station tabletop.
- Left arm starts open at TCP (0.30, 0.14, 0.92); right arm starts in READY
  with open fingers. The overview camera is above and behind both arms at
  (-0.12,0,2.55), aimed at (0.55,0,0.75), 20.4 degrees from vertical. Both wrist
  cameras are retained. The camera stand stays behind the arm bases.

This is a **guided slingshot approximation**, not a calibrated elastic-band or
deformable-foam simulation. The guide constrains pouch orientation/lateral motion.
Two rendered band curves track the measured endpoints; they exert no forces.
The passive spring, finger contacts, projectile flight and block collisions are
native PhysX. No object is welded to a gripper or assigned a launch velocity.

## Scoring

One shot, at most 1,200 control steps (60 simulated seconds at 20 Hz). The
full plan is 424 controls (21.2 seconds); success terminates it at 341. Wall time is measured
per run in receipt.json. Reset restores robot joints, bodies, velocities and rubric;
use a fresh process/output directory for each validation trial.

Each tower earns 0.5 after its original top block falls below z=0.8125 and all
three blocks remain below 40 mm/s linear speed and 0.3 rad/s angular speed for
one continuous second. Credit is recomputed from current state. Both down gives
1.0; the shared runner additionally requires one second of full-score hold.

Credit requires a draw over 100 mm while left TCP is within 40 mm of the handle
with measured opening below 40%, and right TCP is within 50 mm of the ball with
opening below 80%. Launch requires the ball to cross x=0.53 with forward speed
over 0.5 m/s in either adjacent observation (a collision may slow it within
the crossing sample), the right gripper open over 80% and more than 120 mm away, and the
frame still braced. TCP/opening conditions are geometric grasp proxies.

The score latches zero for frame displacement over 45 mm, frame tilt over
15 degrees, a TCP beyond x=0.55, any dynamic body outside x=(-0.12,1.22),
y=(-0.27,0.27), z=(0.7075,1.5), invalid state, an unbraced draw, an invalid
launch crossing, or a top block moving more than 15 mm before valid launch.
Ball-triggered cascades between towers are allowed. See design-review.md for
loopholes and acceptance limits, especially 20 Hz observation and grasp proxies.

## Install and run

Use Linux, Python 3.12 and an RTX GPU with the NVIDIA driver. Accept the NVIDIA
license before starting Isaac. The tested stack is Isaac Sim 6.1.0.0, Isaac Lab
package 17.0.2, Arena package 0.3.0 from release/0.3.1, and Inspect Robots 0.60.0.
Install the pinned YAM adapter and its Arena prerequisites:

```bash
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git
cd yam-isaac-lab-arena
git checkout bd62771e60320433893bd1d2df0fc79b5ddcd6d1
./setup.sh
source ~/IsaacLab-Arena/yam.env
```

From this task folder:

```bash
uv pip install --python "$VIRTUAL_ENV/bin/python" -e . inspect-robots==0.60.0
PYTHONPATH=. python check.py --ik
PYTHONPATH=. python tools/run_recovery_pilot.py . /tmp/slingshot-demo --policy recovery
PYTHONPATH=. python tools/run_recovery_pilot.py . /tmp/slingshot-idle --policy hold
PYTHONPATH=. python tools/run_recovery_pilot.py . /tmp/slingshot-unbraced --policy direct
```

Use fresh output directories; run trials sequentially. The task-local runtime
supports the passive spring joints absent from the stock OBJECTS interface.
Only robot joint targets control motion after reset. The bundled tested runner
records 640 × 360 videos and stops at first success. Inspect completed receipts,
reset checks, source stability, scorer output and all three camera recordings.

With DREAM_API_KEY configured in the Dream client:

```bash
inspect-robots run --embodiment resist_isaac --policy dream \
  -E task="$PWD" -E condition=recoverable -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "In this simulated tabletop toy game, use the left arm to hold the toy slingshot frame and the right arm to pull back and release the preloaded foam ball toward the two toy block towers in the catch tray. Knock down both towers. Keep both arms behind the launch line." \
  --scorer task_score --max-steps 1200 --seed 0 --epochs 1 \
  --store-frames --no-prompt --log-dir /tmp/slingshot-dream
for log in /tmp/slingshot-dream/*.json; do inspect-robots video "$log"; done
```

The packaged runtime is included so this task needs no sibling task folder or
unmerged project checkout. Inspect Robots 1.0 compatibility is not claimed.
