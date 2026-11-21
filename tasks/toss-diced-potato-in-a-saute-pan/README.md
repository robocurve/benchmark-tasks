# Toss diced potato in a saute pan

A true-size 8 in aluminium saute pan stands on a cold burner with twelve 13 mm dice of par-cooked potato in it, each browned on the face it lies on. The robot has to take the pan by its handle, turn the dice over by moving the pan alone, and set the pan back down level on the burner. No utensil is on the table, and no part of the robot may touch the food. Instruction: "Toss the diced potatoes in the saute pan to turn them over, without touching the food, then set the pan back on the burner."

Install once, from this folder: `pip install -e . --no-deps`

Run a trial:

    inspect-robots run --embodiment yam_pan -E task=task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt \
      --instruction "Toss the diced potatoes in the saute pan to turn them over, without touching the food, then set the pan back on the burner." \
      --scorer task_score --max-steps 1200 --seed 0

The score is printed as `task_score`. Use `--policy dream` for Dream-1.

- **Runtime per trial:** about 4 minutes on one unshared GPU for the scripted demo (540 steps: it ends on success), including the simulator's start-up; a trial that runs all 1,200 steps should take about twice as long (not timed). On a GPU shared with other simulator runs the same trial took 8-10 minutes.
- **Seeds:** none. The start is fixed and `object_jitter=0`; `--seed 0` was used for every run below. A fresh run of the same command repeats exactly on one machine, but tumbling dice amplify any difference (another GPU, a second episode in one process, a 0.01 rad change in the motion), so the count of turned dice should be read as one draw from a spread (see Measured results).
- **Scoring checks, no simulator:** `python -m pytest -q test_score.py` (65 tests: every band, each shortcut, the latches, the pan's size and mass; 5 of them check the demo plans' joint limits and are skipped where `yam_arena` is not installed). `python task.py` prints each plan's length.
- **Files:** `task.py` (layout, scoring criteria, the open-loop demo and its shortcut variants), `pan_toss/pan.py` (the pan, dice and burner: geometry, mass, colliders), `pan_toss/embodiment.py` (the `yam_pan` embodiment: spawns them, reports poses, reads robot-on-food contact from PhysX), `pan_toss/plan.py` (joint plans), `dev/` (the tools that produced the measurements below).
- **Third-party assets:** none. Every part is built from boxes, cylinders and simple meshes in the task's own code.
- **Changes to the harness:** none on disk. `pan_toss.plan.install()` wraps `yam_arena.scripted.ArmPlan` in memory so the stock `yam_scripted` policy can play a ready list of joint targets (the stock planner only makes top-down grips; this demo holds the handle with the gripper level). Any other task's demo still gets the stock planner.

## What the robot has to do, and why it is hard

The pan toss is one fast motion of a held container that makes a dozen loose pieces fly a few centimetres, tumble and land back inside. Nothing is grasped except the handle. Too gentle a motion and the dice only slide; a little too strong and they leave the pan (see the sweeps under Measured results: the usable window in flick size is narrow). The robot must also hold a 0.49 kg loaded pan by friction on its handle, 18 cm from the load's centre of mass, through that motion, and put it back on the burner.

## How this differs from the tasks on the board

- **The spatula-flipped egg (TK-84YE4P), and the steak and pancake flips rejected as duplicates of it,** test sliding a tool under one object and turning it. Here there is no tool and no contact near the food: the only grasp is the pan's handle, the food is twelve loose pieces, and they are turned by the pan's own motion (dynamic and non-prehensile).
- **Turning the food with the gripper earns nothing.** Any contact force between any robot link and any die, at any time, sets the score to 0 for the rest of the trial. It is read from the physics engine's contacts, not from how close a gripper comes, so a hand passing over the pan is not penalised and a fingertip nudge is.
- **The score is read from the final state,** not from a first event: where each die lies and which face is under it once everything is at rest, with the pan back on the burner.
- **The instruction does not give the method:** it says to toss and turn the food and to keep off it, not how to move the pan.

## Scoring

Once the trial is over and everything is at rest:

- **Turned:** a die inside the pan whose browned face is not underneath it: the face's outward normal is no lower than 30 degrees below horizontal (vertical component -0.5 or more). A die lying flat has that normal straight down (-1: not turned), level (0: on a side, turned) or straight up (+1: turned), so the threshold sits far from every flat resting pose and only a propped die needs the angle read.
- **Lost:** a die that touched the table, the burner or the floor at any time (latched), or that is not inside the pan's bowl at the end.
- **Net** = turned dice (lost ones never count as turned) minus lost dice.
- **Food touched:** any robot link in contact with any die at any time (latched): score 0.

| Outcome | Score |
|---|---|
| The robot touched the food | 0 |
| Net 0-2 (nothing done, the pan lifted and replaced, the dice only slid about, the pan tipped out) | 0 |
| Net 3-4 | 0.2 |
| Net 5-6 | 0.4 |
| Net 7 or more | 0.6 |
| Net 7 or more, and the pan back on the burner (centre within 3 cm, level within 5 degrees, base on the burner top), both fingertip midpoints 8 cm or more from the pan and handle, pan and dice at rest for 1 s | 0.8 |
| The same, and no die lost | 1.0 |

Possible scores: 0, 0.2, 0.4, 0.6, 0.8, 1.0. The episode ends with success once 1.0 has held for 1 s.

**The cube problem, and what chance gives.** A cube has six faces, so a die that tumbles at random ends with its browned face down one time in six: a toss that tumbles every die turns 10 of 12 on average, and no open-loop motion can do better. Three things keep chance from earning the score. First, jostling does not tumble dice: on a floor with friction 0.35 a 13 mm cube slides rather than tips, and changes face only when it leaves the floor or is driven hard into a wall. A hard, level back-and-forth shake (`PAN_DEMO=shake`: five bursts with the pan at up to 0.7 m/s, die centres never more than 13 mm above the floor, where 6.5 mm is lying flat) turned 3 of 12 and scored 0.2; a gentler one (0.36 m/s) turned none. Second, a toss a little too weak turns few and one a little too strong throws dice out, each of which is subtracted (see the sweeps below). Third, the top band needs 7 of 12 net with none lost and the pan put back.

## Measured results

All in the simulator, fixed layout, `--seed 0`, through the command above (so with Inspect Robots' default guards on). Logs: `~/yam-eval/pan-toss-<label>/`.

| Run (label) | Turned (of 12) | Lost | Robot touched food | Score |
|---|---|---|---|---|
| Demo (`demo1`) | 11 | 0 | no | 1.0 |
| Demo again (`demo2`) | 11 | 0 | no | 1.0 |
| Demo, 640 x 352 cameras (`demo-hd`, the video) | 11 | 0 | no | 1.0 |
| Demo once more, after the last scoring edit (`demo3`) | 11 | 0 | no | 1.0 |
| `PAN_DEMO=shake` (`shake2`) | 3 | 0 | no | 0.2 |
| `PAN_DEMO=poke` (`poke`) | 0 | 0 | yes (peak contact force 2.9 N) | 0 |
| `PAN_DEMO=touch` (`touch`): fingertips push the dice once, then the full demo | 9 | 0 | yes (peak contact force 3.1 N) | 0 |
| `PAN_DEMO=none` (`none`) | 0 | 0 | no | 0 |

**Run-to-run spread.** A fresh simulator process given the same commands repeats exactly on this machine: `demo1` and `demo2` agree to the last digit, so the four demo runs are one sample, not four. Any change does give a different draw, because tumbling dice amplify the smallest difference. Inside one demo run the count after tosses 1 to 5 was 6, 11, 10, 9-10, 11. The `touch` run, whose dice start the toss from where the fingertips left them, ended with 9 turned and none lost. Five further episodes in one simulator session, with the elbow flick changed in steps of 0.01 rad around the demo's 0.30 (0.28 to 0.32), ended with 11 (one die lost: 0.8), 10, 10, 10 and 12 turned: four of five at 1.0. In the four of them that were traced, the count after each toss from the second on ranged from 7 to 12 (the first episode's log was overwritten, so how its die was lost is not known). On another GPU or driver the count will differ in the same way; the top band (7) was set from this spread: it is the lowest count seen after a second toss.

Sweeps in one simulator session (`dev/sweep.py`; later episodes in a session start from a slightly different physics state, so repeats of one setting differ). Flick = (elbow, wrist) target offsets in rad.

With the harness's per-step limit (the limit the command-line guard applies), five tosses unless noted:

| Flick | Highest die centre above the cooking surface | Turned at the end | Lost |
|---|---|---|---|
| (0.30, 0.30): the demo; three tosses | 47-57 mm | 10 | 0 |
| (0.40, 0.30); three tosses | 175-189 mm | 9 | 1 |
| (0.28, 0.30) | not traced | 11 | 1 |
| (0.29, 0.30) | 40-57 mm | 10 | 0 |
| (0.30, 0.30), a second episode in the session | 47-64 mm | 10 | 0 |
| (0.31, 0.30) | 71-82 mm | 10 | 0 |
| (0.32, 0.30) | 74-95 mm | 12 | 0 |

Before the step limit was applied (targets stepped instantly, which the command line does not allow; three tosses each), kept because it shows the window and the spread:

| Flick, hold | Highest die centre above the cooking surface | Turned at the end, per episode | Lost |
|---|---|---|---|
| (0.15, 0.30), 0.10 s | not traced | 4 | 0 |
| (0.20, 0.30), 0.10 s | 55-84 mm | 8, 12, 10, 7, 12 | 0 |
| (0.25, 0.30), 0.10 s | 86-109 mm | 11, 12, 10, 11 | 0 |
| (0.20, 0.40), 0.15 s | 39-80 mm (first episode) | 9, 7, 8 | 0 |
| (0.22, 0.40), 0.20 s | 63-116 mm | 6 | 5 |
| (0.25, 0.45), 0.15 s | 176-230 mm | 2 | 7 |
| (0.30, 0.40), 0.10 s | 218-292 mm | 1 | 9 |

What the trace of the demo run shows (`dev/trace.py` on the per-step log, `PAN_LOG`):

- **Pan motion:** peak speed 1.35 m/s, pitch from 12.9 degrees nose-down to 21.9 nose-up at up to 157 degrees/s. The dice's centres rise up to 59 mm above the cooking surface.
- **Torque:** peak 5.7 N m at the wrist pitch motor (limit 7), 13.8 N m at the elbow and 11.3 N m at the shoulder (limit 27). Holding the loaded pan level takes about 2.0 N m at the wrist pitch joint, 0.42 m from the load's centre of mass.
- **Grip:** the handle is held only by friction between the finger pads and the silicone sleeve. The pan shifts in the fingers by at most 16.2 mm and 3.7 degrees (finger compliance and slip together) and ends 14.7 mm and 3.5 degrees from where it was gripped after five tosses; it is set down 10.5 mm from its starting spot.
- **Contact with the curved wall:** no die's centre comes closer than 6.45 mm to the cooking surface (a cube lying flat is at 6.5 mm), so nothing sinks into or passes through the 3 mm shell, and nothing sticks: the dice slide to the far wall when the pan is tipped and ride up the corner radius in the flick.
- **The demo's toss is bigger than a cook's.** The wrist is the arm's weakest joint, so most of the swing comes from the elbow: the whole forearm scoops up and the pan rises 30 cm at its peak, where a cook's wrist does the same within 10-15 cm.


## The scripted demo

Open loop, from the fixed starting layout only (where the pan, its handle and the burner are): it reads no simulator state. The left arm lines its gripper up behind the handle, slides one finger over and one under the silicone sleeve and closes, lifts the pan 8 cm, and tosses five times. Each toss tips the pan nose-down for 0.4 s so the dice gather at the far wall, swings the elbow and wrist up, and comes back level under the falling dice. Then it sets the pan back on the burner, lets go, draws back off the handle and returns to its ready pose. The joint targets carry a fixed allowance for the sag of the loaded arm, computed from the pan's known mass. 550 steps (27.5 s).

Inspect Robots' default guard lets each joint target change by at most 5% of that joint's range per control step (0.157 rad for the elbow, 0.163 rad for the wrist pitch, at 20 Hz). The plan is built inside that limit, so the guard passes it unchanged, and any policy run with the command above works under the same limit. An earlier version of the toss, tuned without the guard, stepped its targets faster; through the command line the guard slowed it and it turned 1 die of 12 and scored 0. The toss was retuned inside the limit.

Shortcut variants, switched by `PAN_DEMO` (they exist to show the scoring, and score low by design):

- `PAN_DEMO=shake`: the same grasp, lift and set-down, with a hard level back-and-forth shake instead of the toss.
- `PAN_DEMO=poke`: the closed fingertips reach into the pan and push the dice about.
- `PAN_DEMO=touch`: one push at the dice with the closed fingertips, then the full demo.
- `PAN_DEMO=none`: the arms stay at their ready pose.

`PAN_DEBUG=1` prints the scorer's state every quarter second, `PAN_LOG=file.npz` writes every step's poses, joints and torques, and `PAN_TOSS` replaces the toss; these are debugging switches only.

## Real-world setup

- **Pan:** a commercial 8 in (20 cm) aluminium fry pan with a riveted strap handle and a removable silicone handle sleeve, about 0.45 kg; no lid, no non-stick coating needed. The handle's line is marked on the table so the pan is reset pointing at the left arm's base.
- **Burner:** a portable electric hotplate, unplugged and cold, taped or clamped to the table. Any flat-topped stand about 4 cm tall and 24 cm square serves.
- **Food:** waxy potato cut with a 13 mm grid cutter into cubes, boiled 4 minutes, drained and dried; one face of each die seared dark in a separate pan (or painted with a food-safe dark glaze), then all tossed in 5 ml of neutral oil. Twelve dice are laid browned face down with tongs on the marked ring pattern. Dice that break or lose their browned face are replaced.
- **Safety:** everything is cold; the dice weigh 2.4 g each and the loaded pan under 0.5 kg. A die thrown out of the pan can travel about a metre: the table has a 15 cm lip or screen on its far and side edges, and nobody stands within 1 m of the pan's far side during a trial. The usual emergency stop cuts the arms' power.
- **Camera for scoring:** a phone on a stand looking straight down into the pan, used after the trial; the trial video (the rig's cameras) is kept for the touch and lost-dice checks.

## Scoring: real measurement and simulated stand-in

| Criterion | Real measurement (how) | Simulated stand-in |
|---|---|---|
| Die turned | The top photo shows a browned top face directly. For each remaining die, lift it straight up with tweezers and look at the face it lay on: browned underneath means not turned. A die propped on the wall or on another die: lay a protractor against a side photo; turned if the browned face points no lower than 30 degrees below horizontal. | The same angle from the die's pose: the browned face's outward normal has a vertical component of -0.5 or more. |
| Die inside the pan | Count the dice in the pan's bowl in the top photo; a die on the rim, the handle, the burner, the table, the floor or the robot is not inside. | The die's centre is above the cooking surface, below the rim plane and inside the wall's radius at its height, in the pan's own frame. |
| Die lost during the trial | From the video: any die that touches the table, the burner or the floor, even if it ends up back in the pan. | Latched each control step: the die's centre is outside the pan's bowl and within 11.3 mm (a 13 mm cube's half diagonal, so the cube must be touching) of the table top or of the burner top, or below the table top. |
| Robot touched the food | From the video (top and wrist cameras): any contact between an arm or gripper and a die. | Latched every physics step (240 Hz): a nonzero contact force (above 0.0001 N) between any link of either arm and any die, from a PhysX rigid contact view filtered to the dice. Distance is not used. |
| Pan back on the burner | A ruler from the burner's marked centre to the pan's centre (3 cm or less); a spirit level or an inclinometer app across the rim (5 degrees or less); the base sitting on the burner top. | The pan's pose: centre within 3 cm in plan, cooking surface within 5 mm of its resting height and 5 degrees of level. |
| Hands clear | A ruler: both grippers' fingertips 8 cm or more from the handle and the pan. | Each fingertip midpoint 8 cm or more from the handle's centre line and from the bowl. |
| At rest | Nothing moves in the last 1 s of the video. | Pan and every die under 2 cm/s and 1 rad/s for the last 1 s. |

| Quantity | Real | Simulated |
|---|---|---|
| Pan | 8 in aluminium fry pan, about 0.45 kg, curved wall | 203 mm rim, 134 mm floor, 20 mm corner radius, 40 mm deep, 3 mm shell, 463 g; the shell collides as 129 convex pieces (32 sides), not a filled bowl |
| Handle | strap with a silicone sleeve, about 19 cm, rising about 20 degrees | 19 cm strap, 13 x 3.0 x 1.8 cm sleeve, 20 degrees; sleeve friction 1.0 against the fingers' default 0.5 (PhysX averages: 0.75) |
| Dice | 13 mm par-cooked potato cubes, about 2.4 g, slightly rounded edges, some variation | twelve identical sharp 13 mm cubes, 2.4 g |
| Potato on oiled aluminium | sliding friction roughly 0.2-0.4 depending on oil and surface moisture; almost no bounce | friction 0.35 static, 0.30 dynamic; restitution 0.1 |
| Stickiness, starch, oil film | present, uneven | not simulated |
| Browning | a seared face | a painted face; nothing cooks |

## Known limits

- Par-cooked potato dice are close to rigid, which is why this food was chosen, but they are not the sharp identical cubes simulated here; softer or wetter food (onion, cooked rice) would not be represented by this scene.
- The count of turned dice varies from run to run for the same motion, as it does on a real stove. The top band (7 of 12) leaves the demo a margin; it is not a guarantee.
- The contact criterion cannot tell a deliberate poke from a die that bounces off the arm in flight: both score 0. A toss that throws dice at the arm is a failed toss either way.
