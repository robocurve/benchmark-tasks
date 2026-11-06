# Calibrate a tool tip by touching one point from four angles

A steel pin stands in a holder in front of the left arm. Nearby is a fixed post with an 18 mm square pocket on top. The
robot takes the pin, puts its lower end in the pocket, and holds it still in four poses with its end on the same spot, leaning 10 to 35 degrees from
vertical, the pin's axis in each pose at least 15 degrees from every other. This is the motion a
technician uses to teach a robot where its tool tip is: the tip stays on one point while the wrist turns. Each
credited pose earns 0.25.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
cd tasks/tool-tip-calibration   # the task= path below is relative to this folder
INSTR="Calibrate the steel pin's tip on the reference post. Take the pin from its holder, put its lower end in the square pocket on top of the orange-framed post, and hold the pin still there in four different poses, at least 1 second each, keeping its lower end on the same spot. In every pose the pin must lean between 10 and 35 degrees from vertical, and its axis must be at least 15 degrees from its axis in every other pose."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tool_tip_calibration.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tool-tip-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tool_tip_calibration.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tool-tip-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/tool-tip-dream/*.json ~/yam-eval/tool-tip-demo/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step. Use `-E control_hz=20`: a pose's 1 s hold is counted as 20 control steps.

### The demo tilts the gripper
The stock demo waypoints grip straight down, turned only about the vertical. This task needs a tilted grip, so the
task file adds one small extension: a waypoint's `yaw` may be a `TiltedYaw`, a float equal to 0.0 that also carries a
full gripper rotation. When the task file loads, it wraps `yam_arena.demo.grip_rotation` so that it returns that
rotation, and it behaves exactly as before for every ordinary yaw. Nothing else changes; the stock `yam_scripted`
policy plays the demo. Only the demo uses this. Scoring doesn't depend on it.

## Scoring
| Score | Condition | Check on a real table |
|---|---|---|
| +0.25 per pose, up to 1.0 | for 20 control steps (1 s) in a row: the centre of the pin's lower end inside the pocket (within 9 mm of its centre along each table axis, between 1 mm below the floor and the frame's 7 mm top), the pin leaning between 10 and 35 degrees, and still (through the second, its end stays within 2 mm and its axis within 2 degrees of where they started); its axis at least 15 degrees from its axis in every pose credited before, and its lower end within 4 mm of its spot in the first credited pose | the video shows the pin's end in the pocket; a phone level or protractor along the pin gives its lean |
| 1.0 | 4 poses credited; the trial ends 1 s later | |
| 0 | no pose credited | |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0; the best score reached counts (`SCORE_OVER = "best"`). Holding one angle longer, or returning to an angle already credited, adds
nothing. A released pin can't score: it can't stay leaning 10 degrees or more in the pocket, since nothing near the post can hold it up, and a pin standing upright by itself leans under 10 degrees.

## Runtime and seeds
- The scripted demo ends with success at step 498 (24.9 s of simulated time): four poses, the last credited at 24.0 s,
  then the 1 s success hold. On an L40S the whole demo run, including starting Isaac Sim, took about 110 s on a warm
  machine; a full 2,400-step trial with Dream-1 took about 4.6 minutes (it scored 0.0). Isaac Sim's first start on a
  machine takes about 6 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
