# Fit a security bezel on a 1U rack server

A bimanual robot fits the plastic security bezel to the fixed front of a 1U rack server: the two tabs at the bezel's right end slide into the right ear's slots, then the left end is pushed in until its spring latch rides a ramp and clicks behind its catch. Instruction: “Fit the security bezel on the front of the server so it sits flush and clicks shut.”

Real procedure (Lenovo ThinkSystem SR655, "Install the security bezel"): “Carefully insert the tabs on the security bezel into the slots on the right rack latch. Then, press and hold the release latch and pivot the security bezel inward until the other side clicks into place.” The bezel, tab, catch and latch sizes here are assumed, not that product's.

## Real-world setup

Set a 482.6 mm wide, 43.2 mm high (1U) steel server front on the table, facing the robots. The ears' inner faces are 437 mm apart, and the steel panel between them sits 26 mm behind the ears' front faces. The right ear has two 20.8 mm wide, 12 mm high slots (3.8 mm lateral and 6 mm vertical total tab free play) at heights 10.5 and 32.5 mm. The left ear has a 28 mm long steel wedge catch (25 degree ramp) flush with its inner face, and a 14 mm high window behind it that the latch springs into. Set a 430 x 41 x 25 mm, 350 g plastic security bezel upright in front of it. Seated between the ears, the bezel has 3.5 mm of side play at each end. It has two 15 x 17 x 6 mm tabs, a continuous 25 mm deep upper edge, and a spring returned acetal left latch with 14 mm travel and a 350 N/m spring. The latch rides the catch's ramp, moves in 8.5 mm, and returns behind it. The latch and catch use friction 0.25 (acetal on steel); every other contact uses the simulator's default 1.2. Every coordinate and piece size is in `task.py` and `task.json`.

The two YAM grippers have a 93.9 mm maximum opening. The demo pinches the bezel's real 25 mm deep top edge at two points 325 mm apart and adds no grip handles. The right hand slides the tabs into the slots with the bezel turned 8 degrees open (so the left end clears its ear) and lets go. The left hand then releases its edge and pushes the left end in with closed fingers, holding for 1.5 s, which drives the latch over the ramp.

## Scoring

Measure the right tab centres inside the two slots by eye or with a ruler. Award 0.25 when both are seated. Compare the bezel front face with the plane of the ears using a 2 mm feeler gauge along the full width, and check yaw with a square or photo: award 0.25 when the face remains within 2 mm, yaw is at most 2 degrees, and it is at rest over the last 2 seconds. Measure the latch's inward travel from its fully extended position with a ruler or calipers, and check that its nose sits behind the steel catch by at least 1.5 mm: award 0.50 when it is back within 2 mm of its extended position and both right tabs remain seated. (In the simulator the travel is the latch body's displacement along its joint axis, computed from poses.) Score zero for any robot-link contact force above 100 N during the trial, if the bezel falls 20 cm or more below the table top, or while a gripper fingertip remains within 25 mm of the bezel. On a real bezel (a flexing plastic latch, key-locked) the latch check is that the left end stays in under a light pull. On a real rig the 100 N limit is read from the arms' wrist force sensors. Success is 1.0 held for 1 second.

## What makes this task distinctive

The two ends of one 430 mm part do different jobs. The right end is located by two tabs that must slide into slots with 3.8 mm of play; the left end is a spring latch that only closes if the hand pushes the end in far enough, with enough force, for the latch to ride a ramp and snap back behind its catch. The part can only be held by its 25 mm top edge. The right slots are open toward the bezel, so nothing locks the right end against being pulled straight out; the scorer reads tab position, latch spring travel and catch overlap from real contact geometry, with no scripted interlock.

## Run and reset

On a workstation with Isaac Lab Arena and `yam_arena` installed, run the offline checks and scripted trial:

```bash
YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q tasks/server-front-bezel-hook-and-latch/test_task.py
./workstation/run_task.sh server-front-bezel-hook-and-latch
```

The trial limit is 600 control steps at 20 Hz (30 seconds). The demo lasts 25.1 seconds. No random seed or object jitter is used by `run_task.sh`. The run writes a score log, camera videos and setup image under `~/robocurve_runs/server-front-bezel-hook-and-latch/`. To reset, retract the left latch, swing the left end away, unhook the right tabs, and put the bezel back at its marked starting pose.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo: `task_score` 1.0, ended by success at step 456 (22.8 s of trial time); 60 s wall clock for the Isaac run. At the end both tabs are seated, the front face is 0.61 mm from the ears' plane with yaw 0.15 deg, the latch is back at its extended position (0.00 mm travel) with 5.04 mm of catch overlap, and the largest robot-link contact force was 24.9 N (limit 100 N) with no fault.
- Dream-1: `task_score` 0.0 after the full 600-step limit; 72 s wall clock. The right hand moved to the bezel's right end and stopped beside it, the left hand stayed high on the left; the bezel never moved from its start (flush error 177.5 mm).
- Offline tests: 25 passed.

## Third-party assets

None. The ramp mesh (`catch_ramp.obj`) and the rigid assembly are authored for this task.
