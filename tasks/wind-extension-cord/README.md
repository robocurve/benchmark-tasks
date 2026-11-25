# Wind an extension cord onto a reel

Wind the loose extension cord evenly onto the reel, then release it.

## Setup

An open-frame manual reel sits on the YAM station table. The 280 × 200 × 320 mm frame supports a plastic drum with a 96 mm hub diameter, 164 mm flange diameter, and 112 mm between its inner faces. The axle is 160 mm above the table. A KIPP K1468.408-style revolving grip (25 mm diameter, 82.5 mm long, M8 steel stud) travels 88 mm from the axle, on the right-hand side. The axle is 320 mm forward of the arm bases, on the centerline. Secure the reel frame at the feet for a physical trial.

A 900 mm long, 8 mm diameter PVC-jacketed cord leaves a fixed attachment at the top of the drum, 38 mm toward the side opposite the crank from the centerline of the drum. Its other end has a 26 × 20 × 16 mm plug body. Set the crank knob above the axle before laying out the loose cord. Turn the reel 25° so the cord leads forward and to the left of the robot, then lay the cord across the tabletop with its plug end hanging over the far edge. Keep the reel and cord unplugged. Leave clearance below the tail so it can move freely. Reset to this layout for each trial.

### Physical reproduction

The [physical build and inspection guide](PHYSICAL_BUILD.md) provides a dimensioned shaft, drum, bearings, and crank assembly using stock hardware and fabricated flanges. It describes mounting the assembly on a standalone base or adapting the stand from a [Brennenstuhl Garant E](https://www.brennenstuhl.com/en-EG/products/cable-reels/garant-e-cable-reel-25m-h05vv-f-3g1-5). Use a short, electrically disconnected [H05VV-F 3G1.5 cable](https://cz.prysmiangroup.com/sites/default/files/atoms/files/PdfDataSheet%20-%20EN_H05VV-F.pdf) and a [KIPP-style revolving grip](https://www.kippusa.com/en-us/products/METRIC/Handwheels-cranks-and-handles-for-handwheels-position-indicators/Grips-for-handwheels/Cylindrical-grips-revolving/p/agid.16178). The [asset notice](assets/NOTICE.md) describes the simulation geometry and license.

## Scoring

Measure the contiguous wound length from the hub attachment to the first point where the cord leaves the winding bay. The required gripper clearances are 30 mm from any cord centerline and 40 mm from the crank grip center.

- **0** if either clearance fails, less than 42 cm is wound, or the cord makes fewer than 0.85 turns.
- **1.0** if at least 84 cm is wound in at least 1.65 turns, and the wound section satisfies the single-layer and spacing checks below.
- **0.5** for other released outcomes that meet the partial-credit thresholds.

Follow the wound cord from the hub attachment toward the plug. It must progress in one angular direction, with each successive wrap consistently farther toward the same flange. At any angular position shared by two consecutive wraps, the center-to-center spacing along the axle must be 6–30 mm. The outside of the cord must stay within 61 mm of the axle (13 mm above the hub surface). The two checks reject crossed wraps and a second layer, including cases where the overall coil is wide.

For a physical trial, use the indexed photos, straightedge and depth-probe procedure in the [inspection guide](PHYSICAL_BUILD.md#inspect-the-released-coil). In simulation, the scorer interpolates the cord centerline in angle between capsule centers. It checks both sets of centerline knots for pitch extrema and caps centerline radius at 57 mm. Early success requires a full score for one second with the drum nearly stationary.

## Run

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=demo_task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Wind the loose extension cord evenly onto the reel, then release it." \
  --scorer task_score --max-steps 880 --seed 0
```

Run from this folder with the YAM Isaac Lab Arena environment active. The action limit is 880 steps at 20 Hz, or 44 seconds of simulated control time. The reset layout is fixed. The recorded seed-0 trial scored 1.0 in 23.2 simulated seconds under the revised scorer. It released with 90 cm wound in 2.70 turns. Adjacent-turn spacing ranged from 8.6 to 25.6 mm, and the cord outer radius was at most 59.4 mm. The scripted controller reads the drum angle and incoming cord position to guide the YAM joint commands. For a model trial, use the standard policy with `-E task=task.py`.
