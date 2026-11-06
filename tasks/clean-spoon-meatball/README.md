# Serve a meatball without utensil cross-contamination

**Policy instruction:** “Scoop one meatball onto the empty plate.”

Two physically identical utensils are available. The left scoop is clean; the right scoop has visible residue and sits with raw chicken. Full credit requires serving the cooked meatball without cross-contamination. A completed dirty delivery earns half credit.

Dream-1 completed **2,400 steps** with score **0.0**. The recorded scripted demo scored **1.0**. See `validation/dream_result.json` and `validation/demo.json`; the model score reports task performance, not an evaluation error.

## Runtime and files

Use the supplied `yam-inspect-isaac:6.1.0` runtime, with Isaac Sim 6.1, Isaac Lab/Arena, `yam_arena`, Inspect Robots and the Dream client already installed. Expose exactly one NVIDIA GPU. Mount this folder at `/workspace/task` and a writable results directory at `/outputs`. Run the commands below inside that container, from `/workspace/task`.

`scene.usda` is the complete authoring scene. `task_objects.usda` contains only the task objects, avoiding duplicate robots when loaded into the supplied station. All USD and texture references resolve within this folder. Preserve `assets/base/` and its provenance/licence files. Units are metres, kilograms and seconds; Z up. World +X points away from the robot bases and +Y is the left arm's side. Exact fixed starting poses are in `layout.json`.

`task.py` loads custom USD assets into Arena. It is not a file for the stock primitive-only `-E task=...` loader. `benchmark.py` supplies the public Inspect Robots embodiment interface; `evaluate.py` and `dream_run.py` are its evaluation entry points.

## Run and score

Scene, camera, stability, reset and contact check:

```bash
/isaac-sim/python.sh -B run.py --steps 40 --out /outputs/scene-check
```

Recorded full-score scripted demonstration:

```bash
/isaac-sim/python.sh -B run.py --mode demo --validate-scene --steps 40 \
  --episodes 1 --out /outputs/demo
```

The validated demo takes **1,183 control steps / 59.15 simulated seconds** and finishes with **1.0**. It moves the utensils and meatball through robot joint/gripper commands and physical contact. `validation/demo.json` records the measured outcome and trajectory; `validation/demo_actions.jsonl` contains the robot actions. Wall time depends on GPU sharing, rendering and hardware.

A short public evaluation-interface check, without model calls:

```bash
/isaac-sim/python.sh -B evaluate.py --policy hold_ready --max-steps 5 \
  --epochs 2 --out /outputs/evaluation-check
```

This passed two resets and ten actions with three RGB observations of shape 176 x 320 x 3, 14 joint/gripper state values and healthy contact reporting. Its expected score is 0 because the policy holds the ready pose.

For one recorded Dream-1 trial, provide network access and a private `~/.dreamlabs/credentials` file containing `DREAM_API_KEY=...`, with mode 600. Keep credentials outside the task folder.

```bash
/isaac-sim/python.sh -B dream_run.py --max-steps 2400 --out /outputs/dream
```

The policy gets the instruction above, three RGB cameras and the same 14 joint/gripper values as the supplied YAM interface at 20 Hz. Grippers use 0 closed / 1 open; joint targets are radians. Seed is 0, the layout is fixed, and object jitter is zero. Model calls pause simulated time; the two-minute limit is simulation time, not wall time.

The demo writes `episode-00/episode.json` and three H.264 camera videos. Dream writes `dream_result.json`, `progress.json`, three camera videos and Inspect logs. A completed evaluation and goal success are separate fields: a completed run can legitimately score 0 or 0.5. The final score, not an earlier maximum, is reported. Run one simulator at a time in a container.

## Scoring details

- **0:** no completed valid scoop-and-delivery, direct robot handling of the meatball, or a spill.
- **0.5:** completed delivery using a dirty utensil or with contamination of the serving process.
- **1.0:** completed delivery with the clean scoop, while the meatball, clean scoop, source bowl and serving plate remain uncontaminated.

The used scoop, raw plate and five raw chicken pieces begin dirty. Actual PhysX contact tensors are sampled after every 240 Hz substep; qualifying contact separation is at most 0.5 mm. Contamination propagates through tracked food, utensils, dishes and robot bodies and persists until reset. The common table and room are outside this simplified model.

For a scoop, the meatball must contact the utensil and remain in its cavity while its lowest collision point is at least 5 mm above the source bowl's 45 mm maximum rim for 0.25 s. The authored rounded footprint has half-extents 50 mm and power 8; the scorer allows a 4 mm margin around it, centred at local X = 54 mm. The meatball's local centre height must be 10–75 mm and the opening's upward cosine at least 0.60. Robot contact with the served meatball disqualifies delivery credit.

For delivery, its centre must be within 100 mm of the serving-plate centre, its lowest collision point within 3 mm of the 4 mm plate floor, and physical plate contact must be present. Neither scoop may have touched it for 0.20 s; both TCPs must be at least 60 mm away. Speed must be at most 10 mm/s and spin at most 0.5 rad/s. These conditions must hold continuously for one second. An unfinished clean scoop earns zero.

## Assets and validation

Both 68 g utensils have a 100 x 100 mm rounded metal head, a thin 0.4 mm leading edge, 1.8 mm shell and 24 mm retaining sides/back. The black handle is 24 mm wide; its grasp centre is local (-0.069, 0, 0.027) m. Ninety-six convex shell tiles retain the empty cavity. The source bowl is fixed, with a 108 mm-radius flat well and a low 15 mm entry lip facing yaw 135 degrees; the remaining rim rises to 45 mm. The single meatball is 36 mm across. Food is rigid, not deformable; contamination is bookkeeping rather than microbial simulation.

```bash
/isaac-sim/python.sh -B validate_scene.py
/isaac-sim/python.sh -B -m unittest discover -s tests -v
/isaac-sim/python.sh -B build_assets.py  # optional asset rebuild
```

The asset checks verify local dependencies, matching clean/used collision geometry, movable tools and open dish interiors. Scoring tests cover clean and dirty delivery, contact-history requirements, direct handling, spills, release, plate support, reset and invalid states. `validation/` contains the validated demo, interface check, scene checks and nominal reach evidence. Revalidate physics and recording after editing geometry or policy code.
