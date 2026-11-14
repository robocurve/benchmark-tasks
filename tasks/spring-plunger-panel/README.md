# Open a panel held shut by a spring plunger

A small cabinet's hinged front panel is held shut by a spring plunger: the plunger's steel pin passes through a strike
lug on the panel's free edge, and its spring pushes the pin back the moment the red knob is let go. To open the panel,
one hand keeps the knob pulled back about 12 mm until the strike has swung clear of the pin, while the other hand pulls
the panel open by its handle. Pulling the latched panel gets nowhere, and letting go of the knob before the strike has
cleared re-latches it. The instruction names the panel, the plunger's red knob and the angle that counts; that one
hand must keep the knob back while the other swings the panel open, and when to let go, are for the policy to work
out.

Category: **Societal relatability**. This tabletop panel is an analogue of a spring-latched cupboard door or access gate: the familiar operation is holding a spring-return bolt clear while swinging the door past it. The goal is opening the door, with no maintenance operation claimed.

The task runs on Dream's YAM station in Isaac Lab-Arena (`yam_arena`), the simulation of the program's walkthrough.

## Files

| File | What |
| --- | --- |
| `task.py` | The yam_arena task file: the cabinet, the scorer (`score`, `success`) and the check for a forced latch |
| `mechanism.py` | Lets a task file hold a jointed object: the cabinet becomes one fixed articulation with a hinge and a spring plunger |
| `demo.py` | Scripted policies: the full-score demo and three controls; writes the videos, the screenshot and the results |
| `tests/` | `test_task_file.py` needs no simulator (geometry, clearances, scoring); `test_isaac.py` runs the demo and controls in Isaac Sim, and the demo on the Dream-1 command path (`eval_path.py`: `inspect_robots.eval` with the CLI's default guardrails, no model called) |

`mechanism.py` hooks `yam_arena` when the task file is loaded, without modifying it, so the task runs with the standard
`yam_arena` embodiment, the `dream` policy and the `task_score` scorer. It also records the top camera when
`TOP_CAM_VIDEO` is set (below); the videos of the submission are recorded this way, the demo's and the Dream-1 run's.

## Run a trial

In the Isaac Lab-Arena environment of the simulation walkthrough (`source ~/IsaacLab-Arena/yam.env`, or wherever your
install's `yam.env` is), from this folder:

```bash
TOP_CAM_VIDEO=~/yam-eval/panel-dream/dream.mp4 \
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the teal panel on the grey cabinet so that it stands open 37 degrees or more. A spring plunger locks it: pulling its red knob draws the pin back." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/panel-dream
```

`task_score` prints the trial's score: 1.0 on success (the panel open 37 degrees or more for 1 s, the latch never
forced), otherwise `score()` at the end of the episode. With `TOP_CAM_VIDEO` set, the task records the station's top
camera at its full 640 x 360 (the model gets it at 320 x 176): `dream.mp4`, one frame at the start and one per
control step, and `dream.png`, its first frame. `inspect-robots video` makes the per-camera videos at 320 x 176 from
the stored frames, as usual.

The scripted demo and the controls:

```bash
python demo.py --out runs/demo                                        # full score: demo.mp4, setup.png, demo.json
python demo.py --policy hold pull_only early_release --out runs/controls --no-video
python -m pytest -q                                                   # both test files
```

- **Runtime per trial:** about 1 to 2.5 minutes for the full 600 steps on one RTX PRO 6000 Blackwell shared with
  other jobs (depending on their load), rendering the three cameras at 640 x 360. Isaac Sim takes about 30 s to start, and about 5 minutes the
  first time on a new install, while it compiles its shaders. A Dream-1 trial adds the model's round trips.
- **Seeds:** none. The layout is fixed (`-E object_jitter=0`; the cabinet is a fixed object, which yam_arena never
  jitters).

## Interface

As in yam_arena:

- **Control:** 20 Hz, at most 600 steps (30 s). Each target is held for 12 PhysX steps at 240 Hz.
- **Action:** fourteen absolute targets: the left arm's six joint angles (rad) and its gripper (0 closed, 1 open, the
  pads then 94 mm apart), then the same for the right arm. Targets are clipped to the joint limits.
- **Observation:** the station's `top_cam`, `left_cam` and `right_cam` images, and the fourteen joint positions.
- **Start:** both arms in the ready pose, grippers open; the panel shut with the pin through the strike.
- **Arms:** "left" is the robot's left (+y), the arm on the left of the screenshot.

## Scoring

`score()` reads the panel's hinge angle: **1.0** at 37 degrees or more, **0.5** from 7 degrees (the strike has swung
clear of the pin), **0.25** from 2 degrees (the pin is out of the strike's bore), otherwise **0**; the score at the end
of the episode counts. `success()` is the 1.0 condition: held for 1 s, it ends the episode.

The score is **0** if, at any control step, the latch was forced: the simulation's stand-in for the rubric's "bent,
loosened or detached". From the hinge angle and the plunger's travel, `pin_overlap_mm()` places the pin in the panel's
frame and measures how deep it is inside the strike lug or its standoff; more than 1 mm is forcing. With the pin
engaged, the panel has 0.13 degrees of play and is forced from 0.36 degrees. Pushing the panel more than 0.25 degrees
past closed (about 1 mm at its free edge) is forcing too. Touching the plunger or the cabinet is not. Which hand does
what, the arms' path and when the knob is let go are not scored.

On the real panel the same angle is read with a protractor at the hinge; as a check, the free edge stands out from its
closed position by 26 cm x sin(angle): 156 mm at 37 degrees, 32 mm at 7, 9 mm at 2 (the protractor decides).

## The demo and the controls

`demo.py`'s policies read the simulator's state (the panel angle and the plunger travel), plan with IK on the arm's
URDF, and only ever send yam_arena's fourteen joint and gripper targets.

1. Both arms move from the ready pose to their pregrasps in 2 s; the right hand by way of a point 15 cm in front of
   the panel, so it never brushes it.
2. In 1.5 s the right hand reaches the pull bar from the front and the left hand the red knob from the left; both
   close (1.1 s).
3. The left hand pulls the knob back 13.3 mm, allowing for the arm's give and trimming on the measured travel.
4. Once the pin is back 12.2 mm, the right hand swings the panel open along the bar's arc, to 41 degrees in 5 s.
5. When the panel passes 0.12 rad (7 degrees), the left hand lets go and backs away; the spring returns the pin and the
   panel keeps opening.

It scores 1.0: full score from step 200 (10 s), and the success, held for 1 s, ends the episode at step 219 of 600
with the panel at 41 degrees. The pin goes back 13.3 mm and is never forced (its deepest overlap with the strike is
0).

The controls show that the score can't be had by forcing or by mistiming:

- `hold`: both arms stay in the ready pose. Score 0: the panel stays shut.
- `pull_only`: the right hand pulls the latched panel along the same arc; the plunger is never touched. The panel
  stops at 0.13 degrees, its play, held by the pin, without forcing it. Score 0.
- `early_release`: the left hand pulls the pin fully back, then lets go while the panel is still shut; 0.6 s later the
  right hand runs the same opening. The pin is already back in the strike: the panel stops at 0.13 degrees. Score 0.

## The Dream-1 run

One trial of `dreamlabs/dream-1-beta` with the command under "Run a trial" (12 requests of 48 steps): **score 0**.
Both arms reach toward the cabinet, the left hand to the plunger's knob and the right hand to the panel, but the
pin is never drawn back and the panel stays shut for the whole 600 steps.

## Real-world build and reset

- **Cabinet:** a 17 x 28 x 20 cm box of 10 mm plywood, open at the front, on a 16 x 28 x 9 cm plinth, clamped to the
  table at the position in task.json.
- **Panel:** 3 mm aluminium, 26 x 20 cm, on two butt hinges along its right edge. A 10 mm steel pull bar, 6 cm long,
  stands on 4 cm standoffs.
- **Strike:** an 8 mm thick steel lug with a 7.2 mm bore, on a standoff at the panel's free edge.
- **Plunger:** a GN 817 type B style spring plunger (no rest position) on a bracket on the cabinet's left wall, its
  6 mm pin passing through the strike. Fit a light spring (about 0.3 N at rest, 2.4 N fully pulled) and secure the
  plunger with a lock nut so it cannot be unscrewed.
- **Reset:** pull the knob, close the panel and let the knob go; a light tug on the handle must not open it. Return
  both arms to the start pose. Nothing is consumed.
- **Safety:** forces are small; keep hands clear of the hinge and of the pin and strike while the panel moves.

## Fidelity

- Isaac Sim 6.1 PhysX at 240 Hz: Isaac Lab-Arena 0.3.1 and yam-arena 0.1.0 (kairosiann/yam-isaac-lab-arena at
  `bd62771`), on Dream's calibrated station.
- The cabinet, panel and plunger are rigid boxes, cylinders and a capsule, on one fixed articulation: a revolute hinge
  (0 to 66 degrees, 0.025 N m friction) and a prismatic slide for the pin, 14 mm of stroke against a 150 N/m spring
  preloaded 2 mm (0.3 N at rest, 2.4 N fully pulled). The moving parts collide with each other, not with the cabinet.
- The strike's bore is square (7.2 mm) around the round 6 mm pin: 0.6 mm of clearance each side, as on the round bore.
- The plunger follows a GN 817 type B, but its stroke, spring and the hinge friction are nominal values, not
  measurements, and no manufacturer CAD is used. Pin, strike, panel and cabinet are rigid, with no wear or damage
  modelled.

## Licence

MIT (`LICENSE`). The robot, the station and the simulator come from the Isaac Lab-Arena environment, not this folder.
