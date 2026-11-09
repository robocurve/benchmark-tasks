# Set a lathe tool to centre height with a pinched steel rule

A lathe tool must cut exactly at the height of the spindle's centre line. Set high or low, it rubs instead of cutting,
leaves a nub at the centre of a faced end, and cuts tapers and threads to the wrong form. From US Army TC 9-524,
*Fundamentals of Machine Tools*, chapter 7
([PDF](https://archive.org/download/MManuals/UsArmyMachinistFundamentals-machineTools1996.pdf)):

> "Set the tool bit exactly on center to avoid leaving a center nub on the workpiece."

The error is a fraction of a millimetre, too small to see. The shop check makes it visible: trap a thin steel rule
lightly between the tool's cutting edge and a round bar held in the chuck. The rule stands vertical when the edge is
on centre, and leans one way when it is high and the other when it is low, by about the height error divided by the
bar's radius. On an 8 mm bar, 0.25 mm of error is several degrees of lean.

Done by: lathe setters and operators (O*NET 51-4034).

So the robot must take the rule, hold it upright in a gap of a few millimetres between the tool and the bar, turn the
cross-feed crank until the tool traps it, let go without knocking it out, then turn the tool's height crank until the
rule stands vertical, reading only the rule.

The task is a task-local plugin for the stock `yam_arena` adapter (Isaac Lab-Arena 0.3.1, Isaac Lab 3.0, Isaac
Sim 6.1, the calibrated `station` scene). It registers:
- the embodiment `yam_lathe_centre_height`: the stock robot, cameras and 14-D joint action and observation, unchanged,
  plus the lathe's bodies and joints, scored from the measured tool pose;
- the policy `lathe_centre_height_scripted`: a scripted demonstration, plus the failure paths below.

## Setup
Coordinates: x forward from the line between the arm bases, y left from their midpoint; heights above the table top.

| Part | Description | Position |
|---|---|---|
| bed | grey cast iron, 15 x 28 cm, 3 cm tall, fixed | x 22 to 37 cm, y -15 to +13 cm |
| headstock and chuck | grey headstock 10 x 8.5 cm, 15.5 cm tall; dark chuck 8 cm across facing right; fixed | x 25 to 35 cm, y 4.5 to 13 cm |
| bar | steel, 8 mm across, axis along y, 50 mm out of the chuck, fixed | axis at x 30 cm, 10 cm high; free end at y -4 cm |
| tool | 8 mm square steel bit in a black holder; the cutting edge is a thin strip at the bit's top front | edge at x 28.8 cm (8 mm from the bar's surface), y -2.5 cm, 0.75 mm above the bar's centre line |
| cross-feed crank | yellow knob 16 mm across, 30 mm tall, on a 24 mm orange arm, spinning on it; 2.54 mm of tool travel toward the bar per clockwise turn (0.100 in) | axis on a grey pedestal at (20, 23) cm, hub 6 cm high |
| height crank | the same knob and arm on top of the tool holder; 1.06 mm of tool drop per clockwise turn (24 threads per inch) | axis at (23.8, -2.5) cm, hub 12 cm high |
| steel rule | 150 x 13 x 1 mm, satin steel, black 10 mm marks, red top end, free | standing in a 1.3 mm slot in a blue block (3 x 4 x 3.5 cm) at (17, -12) cm |

- **Start:** both grippers open at the ready pose. One fixed layout; nothing is randomized.
- **The cranks** drive the slides one way (a screw is not back-drivable): pushing the tool turns nothing.
- **The trap:** the cross slide is spring-loaded toward its crank-set position, so the edge traps the rule against
  the bar with about 0.8 N; the rule then stays where it is when let go.

## Scoring
Measured at 20 Hz from the tool's pose and the robot's measured joints.

- **Edge error** e = |cutting edge height - spindle centre height|.
- **Credit** = (e0 - e) / (e0 - 0.25 mm), clipped to 0..1, where e0 is the start error (0.75 mm).
- **Rest state:** the tool and both cranks still, both grippers at least 80% open with their tool points at least
  5 cm from both crank knobs. A credit counts (best so far) only after a rest state has held for 1 s.

| Score | Condition | How to check on a real lathe |
|---|---|---|
| 1.0, success, the trial ends | e at or under 0.25 mm | centre-height gauge, or face a bar end: no nub |
| 0 to 1 | e between 0.25 and 0.75 mm, linear | centre-height gauge |
| 0 | e at or above the start error, including the tool untouched or the height crank turned the wrong way | |

- **No credit for touching:** gripping, moving or turning anything earns nothing by itself; only the edge's final
  height counts, at rest with the hands off.
- **The rule is the instrument, not the score:** the score reads the edge height. The rule is how a machinist (or a
  policy) can tell where that edge is: in the cameras, 0.25 mm of height is less than a pixel.
- The scorer is pure Python, in the package's scoring module.

## Run a trial
Install the pinned adapter, which installs Isaac Lab-Arena and Inspect Robots, then this package into the same
environment. On the Robocurve launchable the environment already exists:

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/lathe-centre-height
cd tasks/lathe-centre-height
INSTR="$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')"
COMMON=(--embodiment yam_lathe_centre_height -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176
        --instruction "$INSTR" --scorer task_score --max-steps 4800 --store-frames)
# The scripted demonstration (full score)
inspect-robots run "${COMMON[@]}" --policy lathe_centre_height_scripted --log-dir runs/demo
# Dream-1 (needs a Dream API key in ~/.dreamlabs/credentials)
inspect-robots run "${COMMON[@]}" --policy dream --log-dir runs/dream
# Failure paths
for m in wrongway half norule idle; do
  inspect-robots run "${COMMON[@]}" --policy lathe_centre_height_scripted -P mode=$m --log-dir runs/$m
done
# One MP4 per camera, next to each run's frames
for log in runs/*/*.json; do inspect-robots video "$log"; done
```

- `-E telemetry=<file.jsonl>` writes a per-step trace: both cranks' turns, the tool's advance and edge error, the
  rule's pose and lean, the grippers and the score.
- The demonstration reads the measured crank angles and the rule's pose, and stops the height crank when the rule's
  lean reaches vertical (what a machinist sees). It never reads the edge height.
- Failure paths: `wrongway` turns the height crank the wrong way (the edge rises 0.75 mm more); `half` stops the
  height crank halfway; `norule` skips the rule and turns the height crank by the computed amount, open loop;
  `idle` holds both arms still.

## Results
Measured 2026-11-08 to 2026-11-09 UTC on one L40S, the settings above (one episode each). Edge = cutting edge height
above the spindle centre at the end; lean = the rule's lean at the end (positive: its top away from the robot).

| Run | Score | Steps | What happened |
|---|---|---|---|
| demo, `--epochs 2` | 1.0, 1.0 | about 1,900 each | the same in both episodes: rule vertical at edge +0.192 and +0.183 mm (each episode starts at 0.750 mm) |
| scripted demo | 1.0 | 1,901 (95 s) | rule taken and slid edgewise into the gap; cross-feed 2.94 turns trapped it (edge 0.75 mm high: lean +10.8 deg); height crank turned 0.52 turn until the rule stood vertical (lean +0.24 deg): edge **+0.191 mm** |
| `wrongway` | 0 | 1,783 | height crank turned 0.71 turn the wrong way: edge +1.497 mm, the rule still leaning +10.8 deg |
| `half` | 0.75 | 1,723 | height crank stopped halfway: edge +0.362 mm, the rule visibly leaning (+8.8 deg) |
| `norule` | 1.0 | 318 | no rule; height crank turned the computed 0.71 turn open loop: edge 0.000 mm (see Known limits) |
| `idle` | 0 | 1,200 | nothing moved: edge +0.750 mm |
| Dream-1 | 0 | 4,800 (240 s) | knocked the rule over in its stand; turned neither crank: edge +0.750 mm |

- The rule as an indicator (run of 2026-11-08 with the final edge geometry, the height crank turned slowly through
  centre): edge +0.74 mm, lean +10.8 deg; +0.43, +10.8 (the rule still held by friction); +0.32, +8.0; +0.20, +3.4;
  +0.09, -1.5; +0.03, -4.0; 0.00, -5.6. So the rule stands vertical at about +0.1 mm, turns about 40 degrees per mm
  once it moves, and sticks briefly before it does.
- Media: `demo.mp4` (top camera), `demo_wrist.mp4` (right wrist camera: the rule beside the bar), `dream_run.mp4`,
  `setup.png`.

## Known limits
- The cranks turn about vertical axes so the overhead camera sees them; on a real lathe the cross-feed handwheel faces
  the operator. The height crank's axis stays fixed while the holder moves up to 8 mm toward the bar.
- The cross slide and the height screw are spring-driven slides that follow the cranks' measured turns, not modelled
  screws.
- The rule is rigid; a fingertip pad's contact patch is modelled with torsional friction (point contacts alone let a
  pinched plate spin about the contact normal).
- In the simulator the rule stands vertical at about +0.1 mm of edge height (contact and friction between the edge
  strip and the 1 mm rule), and it sticks briefly before it starts to turn. Stopping at vertical lands inside the
  0.25 mm band, but with a margin of about 0.06 to 0.15 mm.
- One fixed layout: a policy that memorized the crank turns could pass without the rule.
