# Fold a travel hair dryer's handle

Folding a compact hair dryer reduces its volume before drawer or suitcase storage. The bounded mechanical operation ends once the folded device rests released on the counter.

## Recorded results

| Policy | Revised score | Recorded actions | Trials |
| --- | ---: | ---: | ---: |
| Scripted reference | 1.0 | 878 / 1000 | 2 |
| Dream-1 beta | 0 | 1000 / 1000 | 1 |

The board hosts the setup image and both uncut videos as **M-SY8Q2Y**. [Revision evidence](evidence/REVISION.json) records the native replay results. Revised scoring of seed-0 recordings: scripted 1.0 in both trials, first reached at action 859 of 1,000 (878 recorded). Dream-1 beta 0.0 after 1,000 actions; the hinge stayed at 0.283° throughout.

The reference can inspect simulator body/contact state. Dream receives only
three 320 × 176 RGB images and 14 measured joint/gripper positions. The extra
side camera is for review. Guard changes and service session renewals are
recorded in the model summary; one model trial is not a success-rate estimate.

## Score a real trial

Score the current hinge fold from its open position: below 45° earns 0, 45° or more earns 0.4, and the folded zone (80–93°, around the 90° storage stop) earns 0.7. Award 1.0 once the intact dryer rests entirely on the counter, both hands are clear of the dryer and cord, and the handle stays in that zone for one continuous second. During that second each visible barrel end must stay within 1 cm of where it began. Restart the clock on loss of fold, release or support, or from a new rest position if the barrel moves farther. Detachment, leaving the counter or invalid simulation input overrides all credit with 0.

With the cool, unplugged dryer fully open, take a photo square to the hinge axis. Fold it to its storage stop and take a second photo. On the photos draw the barrel centreline, the line from hinge pin to grip tip, and the 45° and 80° boundaries from the open grip using a paper protractor. Register the overlay to the barrel and pin in the trial view. Use a second view if the grip or fingers are hidden. The handle can move inside the folded band.

Place a centimetre scale in view outside the workspace. Use a fixed top view and a second side view to track the centres of the two barrel ends against their positions when the hands clear. For a tilted barrel, use the two views to resolve lateral and vertical motion. Time one second of counter support, hands clear and continuous fold using video timestamps. If an end moves more than 1 cm, start the rest measurement again at its new position. Check the plug, cord and hinge remain attached. An empty trial scores 0.

## Install and run

Prepare the pinned native runtime in [ENVIRONMENT.md](ENVIRONMENT.md), then run
these commands **from this task folder**, in that environment:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[test]'
python tools/verify_source.py
PYTHONPATH=. python -m pytest -q tests
PYTHONPATH=. python -m rdb_hairdryer.run scripted --out runs/scripted-new
```

For a fresh model trial, install the model extra and provide your own private
credential file; the output directory must not already exist:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[dream]'
python tools/evaluate_dream.py --task fold-a-travel-hair-dryer-handle --out runs/model-new --credential-file /path/to/private/credentials.env
```

The portable Dream runner keeps the stock SDK, inputs, guards and action budget.
Only package discovery and credential-path defaults differ from the original
recording entry point; no new model result is claimed for that packaging change.
One simulated trial lasts at most 50 seconds at 20 Hz.
The recorded model trial took 235.7 seconds of wall time
including initialization and inference; hardware and service latency vary.

## Specification and reset

The exact evaluated instruction, fixed starting layout, numeric rubric,
fidelity assumptions and reset are in [task.json](task.json).
[TECHNICAL.md](TECHNICAL.md) gives the mechanism and physical approximations;
[TASK-DESCRIPTION.md](TASK-DESCRIPTION.md) gives the real procedure and references.
The scripted runner scores every native physics substep and writes its original
`RESULT.json`, videos and grading inputs into its selected output directory.
`tools/verify_source.py` checks source identity, instruction and ordinary budget.

The evidence folder contains clearly marked summaries and hashes of the
original results and independently completed audits. Full raw input archives
are retained separately, rather than represented by truncated traces here.
The original recordings use the same reset, assets, actions and instruction as this revision. The archived summaries identify the scorer at recording time.

## Assets and licensing

The task props, their materials and the review-camera placement are original and covered by the task's [MIT license](LICENSE). Product references supply dimensions and context. The scene loads the following standard assets from the pinned runtime.

| Asset | Source and license |
| --- | --- |
| Two YAM arms with parallel grippers, link meshes, collision geometry, USD conversion and split arm covers | I2RT Robotics, MIT. The runtime ships the notices with the [YAM assets](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam). |
| Calibrated station, worktop, camera stand, lights, overhead D435 and two wrist D405 pinhole cameras | Loaded from the benchmark's YAM runtime. Its [README](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/README.md#license) states that the runtime as a whole has no declared license. These runtime assets are installed separately. |
| Standard ground plane | NVIDIA Isaac Sim content, loaded by Isaac Lab under the [Isaac Sim license](https://docs.isaacsim.omniverse.nvidia.com/latest/common/license-isaac-sim.html). |
