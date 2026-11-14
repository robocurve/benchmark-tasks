# Turn a desktop monitor to portrait on its adjustable stand

The scripted demonstration reaches 1.0 in 787 actions. A standard Dream-1 beta seed-1 trial completes 1,800 actions and scores 0.0, with no errored trial. The adapter logged connection/rate-limit retries before completing the trial.

The robot adjusts a real-size desktop monitor from low landscape to portrait for
reading a document, leaving its centre 320 mm above the desk. The panel is moved
directly: there are no axis-release buttons, indexing pins or control puzzle. The
monitor has a counterbalanced height slide and friction tilt/pivot joints. Starting
low, the panel cannot turn through its rotation sweep without hitting the desk;
raising and tilting it gives clearance. Both hands must be clear at the end.

## Real product basis and approximations

Dimensions and adjustment ranges are based on the Dell P2217 in the
[manufacturer's user guide](https://dl.dell.com/manuals/all-products/esuprt_electronics_accessories/esuprt_electronics_accessories_monitors/dell-p1917s-monitor_user%27s-guide_en-us.pdf),
printed pages 14 and 18–19, with the adjustment instructions on pages 57–58:

- Panel: 503.4 mm wide, 330.4 mm high, 50.3 mm deep; 3.00 kg without stand/cables.
- Stand foot: 242.6 mm wide and 180 mm deep. Total monitor/stand mass: 5.33 kg.
- Landscape overall height: 367.4–497.4 mm; height travel 130 mm.
- Tilt: 5 degrees forward and 21 backward; swivel: 45 degrees each way;
  pivot: 90 degrees each way.
- The guide directs users to extend the monitor vertically and tilt it upward
  before rotation to avoid hitting the bottom edge.

The simulation is a dimensioned rigid-body approximation, not manufacturer CAD or
a calibrated reproduction of internal mechanisms. A low-rate preloaded spring
approximates the height counterbalance (32.93 N at its low stop, 31.63 N high).
Estimated slide friction is 2 N; tilt, pivot and swivel friction efforts are
0.95, 0.18 and 0.8 N m. Passive damping and estimated internal mass distribution
are disclosed in `task.py`. These stand joints receive no goal-following actuation.
The robot moves the panel using physical contact with standard 50 N YAM grippers. Contact friction is estimated as 0.8, with a 6 mm minimum torsional patch radius.
The authored pane/case, stand column, base and joint hubs represent the visible
geometry; electronics, cables, glass fracture and cosmetic damage are omitted.

Two fixed 200 × 200 × 350 mm metal pedestals raise the YAM arm bases above the
standard desk for access to the full-size screen. They are part of the physical
setup and present for both the model and the demonstration. Arm/gripper models,
control gains and action interface remain standard.

## Run

In the program's Isaac Lab-Arena environment, from this directory:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E device=cpu \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the monitor to portrait for reading a document. Leave its centre 32 cm above the desk, facing straight ahead and upright, with the base where it started. Let go and leave it stable." \
  --scorer task_score --max-steps 1800 --seed 1 --store-frames --log-dir model_out
python demo.py task.py demo_out --max-steps 1800
DEMO_RESET_CHECK=1 python demo.py task.py reset_out --max-steps 1800
DEMO_VARIANT=idle python demo.py task.py idle_out --no-record
DEMO_VARIANT=no_pivot python demo.py task.py landscape_out --no-record
DEMO_VARIANT=held python demo.py task.py still_held_out --no-record
DEMO_VARIANT=low_rotate python demo.py task.py low_rotation_out --no-record
```

The scripted controller turns halfway, releases and changes its grip on the upper bezel, then completes the turn. Visual pose feedback corrects robot/grasp compliance. It uses simulator poses for planning, but all task motion is
produced by robot joint commands. It does not write monitor poses or joint states
during an episode. Passive monitor joints are initialized only during reset.

## Measured validation

| Trial | Score | Actions |
|---|---|---|
| Full scripted demonstration | 1.0 | 787 |
| Dream-1 beta, seed 1 | 0.0 | 1,800 |
| No movement | 0.0 | 180 |
| Raise and tilt, leave landscape | 0.0 | 640 |
| Reach the target but keep holding | 0.25 | 852 |
| Try rotating while still low | 0.0 | 640 |

The low-rotation control physically reaches the desk and moves the base up to
85.4 mm, and scores zero. The full demo finishes at 89.78-degree pivot,
316.24 mm centre height, -0.96-degree tilt and 2.00-degree swivel; base
translation is 0.182 mm. Its hands-off stability window passes. After every
scripted trial, actual same-process reset restores low landscape with all
four passive joints near zero; 20 subsequent idle actions score 0.

The demo has 39.35 s of simulated control time. The measured model trial took
about six minutes wall time, including startup and backend retries; it has
90 s of simulated control time. The layout is fixed. These measured trials
are feasibility/model evidence, not a broad robustness estimate.

## Score

At the end of the trial:

- 1.0: portrait within 3 degrees; centre height 320 ± 10 mm; upright and facing
  ahead within 3 degrees; hands clear; all maintained for 2 s with centre drift
  below 2 mm and pivot drift below 1 degree.
- 0.5: portrait within 3 degrees and hands clear, but full-score conditions absent.
- 0.25: raised more than 100 mm and rotated more than 30 degrees, but still held
  or incomplete.
- Otherwise 0.

The base must remain within 10 mm and 3 degrees of its initial pose. Moving or
tipping the base, or any panel corner coming within 1 mm of the desktop,
prevents all credit. The base and desktop conditions are retained throughout the episode;
moving the monitor clear afterwards cannot erase it. Heights, orientations,
clearance and final hands-off stability can be checked from video and simple
measurements. Simulator force traces are not scoring conditions.

## Third-party assets and licenses

All task-specific geometry, code and controller are provided under the MIT
License in `LICENSE`. No manufacturer CAD, picture, logo or manual is bundled.

| Asset/dependency | Source | License | Bundled |
|---|---|---|---|
| YAM arm/gripper model | i2rt, revision `120c3c81400171174604e503943f8d1ebc891058` | MIT | No |
| Station floor, desk, camera mast, lights and cameras | Shapes generated by yam-isaac-lab-arena, revision `0059d02a1a91bc6834c1d0846022891df0c78730` | SDK README states the repository has no license yet; YAM models have their separate MIT license | No |
| Isaac Lab-Arena | revision `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` | Apache 2.0 | No |

No SDK library object is used. Product dimensions/operating instructions above are
factual references; internal spring/friction estimates are not manufacturer data.
