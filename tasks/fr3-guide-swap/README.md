# Hand-guide a Franka arm to its service pose with its grip's enabling button held mid-band, then teach the pose

A Franka Research 3 (FR3) stands on the far edge of the bench, stopped mid-task, its hand raised and turned away. The robot guides it by hand to its service pose, with the hand straight down over a tool stand: one gripper closes across the FR3's Pilot-Grip, holding its Enabling Button at the middle position and its Guiding Button in, and draws the arm there. At the pose it lets the buttons go, so the arm stops and holds, and the other gripper presses Teach to store the pose.

**Ability tested:** keeping a squeeze inside a narrow band while the same grip pulls a load. The FR3 moves by hand only while the Enabling Button sits at its middle position and the Guiding Button is pressed. Pressed too lightly, the arm holds; pressed through to the end, it stops, and moves again only after a full release and a new half-press. The pull that drags the seven-joint arm loads the same pads that squeeze the buttons. Then the pose is reached by feel: let go too early and the arm stops short. A picture shows where the hand should end, not how hard to squeeze on the way.

**Why it matters:** robots are set up and serviced by hand. A technician teaches a collaborative arm its poses by guiding it through its own enabling device. Three-position enabling switches (IEC 60947-5-8) work this way on purpose: released stops, middle enables, and a full squeeze, the reflex of a startled hand, stops too.

**Hands:** in the demo the right hand guides and the left presses Teach. One hand could do both, letting go of the grip and then pressing Teach; the score does not check which hand does what.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "The Franka arm stopped mid-task. Guide it by hand with its grip's buttons to its service pose: the hand pointing straight down over the tool stand, its fingers about 5 cm above it, the grip turned toward the right-hand arm. Let the buttons go there, press Teach, then let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                    # full score, with a video
python demokit.py task.py OUT --no-record --mode hard        # cheat checks: a full squeeze; the arm pushed with no buttons; the arms wait
python demokit.py task.py OUT --no-record --mode push
python demokit.py task.py OUT --no-record --mode none
```

The demo is a scripted reference. It reads the FR3's joints, which the cameras see; a policy gets only the cameras. The right hand comes along the Pilot-Grip from its outer end, tilted 60 degrees, and closes across its flanks to a set width, which holds the Enabling Button in its middle band. It draws the grip along the arm's path to the service pose in small steps, then opens; the arm holds. The left hand, tilted 60 degrees from straight down, brings its closed fingers' ends over Teach on the Pilot-Disc on top of the wrist (link 6, about 38 cm above the bench), presses it 3 mm, and both move clear.

## Building it for a real trial

- **Arm:** a Franka Research 3 with the Franka Hand, in Desk's programming mode, its base on the bench's far edge facing the YAMs, at about 80 cm in front of the YAMs' bases.
- **Start:** the arm held at the start pose (joint angles in `task.py`, `Q_START`): its flange about 29 cm above the bench, 10 cm right of the stand, turned 23 degrees.
- **Tool stand:** a wooden block 10 x 10 x 3 cm under the service pose.
- **Teach:** on the Pilot-Disc on top of the wrist, as on a real FR3; at the service pose it faces up, about 38 cm above the bench.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| The Enabling Button's travel and middle band | 6 mm; middle band 1.5 to 4.0 mm; full past 4.5 mm | **Not measured; approximated.** Dial gauge on the button, watching Desk's status light |
| The buttons' forces | 7.5 N at the middle position, 15 N full (Enabling); Guiding 1.5 N/mm | **Not measured; approximated.** Push-pull gauge on each button |
| The drag in Free mode | 2 N m s/rad per joint, no weight felt | **Not measured.** Luggage scale at the grip, drawn at a steady 0.1 m/s |
| The grip's width across its buttons | 60 mm handle, buttons standing 6 mm proud | From Franka's visual mesh (63 to 73 mm); caliper on a real grip |
| Teach's travel and force | 4 mm, 1.5 N/mm; counts at 2 mm | **Not measured; approximated.** Push-pull gauge on the button |

## Safety for a real trial

The FR3 is powered and moves when guided: keep its external enabling device and an emergency stop in reach, run it in Desk's programming mode at reduced speed, and keep people out of both arms' reach.

## Scoring

See `scoring` in `task.json` and `Trial` in `task.py`. `python3 check_score.py` runs that scorer on fifteen hand-built states (the full result, a hand left near the hand, a finger or Teach, the partial results, a release with the Guiding Button still in, a full squeeze, the arm pushed with no buttons, the hand driven onto the bench, and doing nothing) with no simulator. In a real trial, from Desk's taught pose and log, and video:

- **0.25:** guiding enabled (the Enabling Button at its middle position and the Guiding Button in) and the flange moved at least 10 cm along its path while enabled.
- **0.5:** also the flange within 5 cm and 15 degrees of the service pose.
- **0.75:** also within 1.5 cm and 5 degrees, with both buttons let go there so the arm holds.
- **1.0:** then Teach pressed with the arm still at the pose, every fingertip of both YAMs at least 5 cm from the Franka arm's surface (its links, hand, fingers, grip, buttons and Pilot-Disc), held 1 s.
- **0** if the Franka arm touches the bench or the tool stand, a joint reaches its limit, or a YAM pushes the held arm along (more than 2 cm/s for 0.5 s).

In the simulator the enabling logic is code: the buttons are sprung slides, and their travel sets whether the joints are free or held (a stiff hold).

## Results

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | The right hand closes across the Pilot-Grip (Enabling Button 2.4 to 2.5 mm, in its middle band; Guiding Button 4 mm), draws the arm 15.6 cm to the service pose, lets the buttons out (the arm holds 2.9 mm and 1.0 degree from the pose); the left hand, tilted 60 degrees, presses Teach on the Pilot-Disc on top of the wrist 3.2 mm; both move clear of the arm's surface; held 1 s | 1.0, at step 638 (31.9 s) |
| `hard` | The grip squeezed through to the full position (6 mm): the arm stops and holds; the drag that follows moves nothing | 0.0 |
| `push` | No buttons: the arm pushed at its grip with closed fingers; it holds and does not move | 0.0 |
| `none` | The arms wait | 0.0 |
| Dream-1 | The arms move about; the right hand comes near the Franka arm's wrist but presses neither grip button, and the arm stays at its start pose to the 2,400-step limit | 0 |

No seed is needed: nothing in the scene is random. Runtime per trial, on one GPU: the demo's 638 steps take about 2 minutes while recording three cameras; a run to the 2,400-step limit takes about 7 minutes.

## Assets

The FR3's link meshes (`fr3_link0.stl` to `fr3_link7.stl`), the Franka Hand's mesh (`fr3_hand.stl`), and the arm's kinematics and joint limits (in `task.py`) are from franka_description (https://github.com/frankaemika/franka_description), Copyright 2023 Franka Robotics GmbH, under the Apache License 2.0; the licence and notice are in `FRANKA_DESCRIPTION_LICENSE.txt` and `FRANKA_DESCRIPTION_NOTICE.txt`. The grip, its buttons, the Teach button, the fingers and the tool stand are made by the task's own code from plain shapes. Franka, Franka Research 3 and Desk are trademarks of Franka Robotics GmbH, named for identification only.

## Licence

MIT
