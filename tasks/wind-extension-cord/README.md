# Wind an extension cord onto a reel

Wind the loose extension cord evenly onto the reel, then release it.

## Setup

An open-frame manual reel sits on the YAM station table. The 280 × 200 × 320 mm frame supports a plastic drum with a 96 mm hub diameter, 164 mm flange diameter, and 112 mm between its inner faces. The axle is 160 mm above the table. A KIPP K1468.408-style revolving grip (25 mm diameter, 82.5 mm long, M8 steel stud) travels 88 mm from the axle, on the right-hand side. The axle is 320 mm forward of the arm bases, on the centerline. Secure the reel frame at the feet for a physical trial.

A 900 mm long, 8 mm diameter PVC-jacketed cord leaves a fixed attachment at the top of the drum, 38 mm toward the side opposite the crank from the centerline of the drum. Its other end has a 26 × 20 × 16 mm plug body. Set the crank knob above the axle before laying out the loose cord. Turn the reel 25° so the cord leads forward and to the left of the robot, then lay the cord across the tabletop with its plug end hanging over the far edge. Keep the reel and cord unplugged. Leave clearance below the tail so it can move freely. Reset to this layout for each trial.

The [Brennenstuhl Garant E reel](https://www.brennenstuhl.com/en-EG/products/cable-reels/garant-e-cable-reel-25m-h05vv-f-3g1-5) is a reference for the general mechanism. The exact dimensions above describe the [NVIDIA USDCraft-10K](https://huggingface.co/datasets/nvidia/USDCraft-10K)-derived scene. The cord diameter matches [Prysmian H05VV-F 3G1.5](https://cz.prysmiangroup.com/sites/default/files/atoms/files/PdfDataSheet%20-%20EN_H05VV-F.pdf). The replacement grip follows the [KIPP dimensional catalog](https://www.kippusa.com/en-us/products/METRIC/Handwheels-cranks-and-handles-for-handwheels-position-indicators/Grips-for-handwheels/Cylindrical-grips-revolving/p/agid.16178). Asset license and modifications are in [assets/NOTICE.md](assets/NOTICE.md).

## Scoring

Measure the unwound tail from the first point where the cord leaves the volume between the flanges and inside their outer diameter. Subtract that length from 90 cm. Use side and front photos to count the turns and measure lateral spread. Subtract the 8 mm cord diameter from the width between the two outermost edges of the wound cord to obtain the centerline spread.

- **0** if either gripper still holds the reel crank or cord, less than 42 cm of cord is wound, or fewer than 0.85 turns are present.
- **1.0** if at least 84 cm are wound in at least 1.65 turns and the wound centerline spans at least 16 mm along the axle.
- **0.5** for other released outcomes at or above the partial-credit thresholds.

The scorer checks the gap to both gripper centers. The episode ends early if full credit persists for one second with the drum nearly stationary.

## Run

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=demo_task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Wind the loose extension cord evenly onto the reel, then release it." \
  --scorer task_score --max-steps 880 --seed 0
```

Run from this folder with the YAM Isaac Lab Arena environment active. The action limit is 880 steps at 20 Hz, or 44 seconds of simulated control time. The reset layout is fixed. Three scripted trials with seed 0 scored 1.0, and the recorded trial lasted 23.2 simulated seconds. The scripted controller reads the drum angle and incoming cord position to guide the YAM joint commands. For a model trial, use the standard policy with `-E task=task.py`.
