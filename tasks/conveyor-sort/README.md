# Sort blocks from a moving conveyor

Five red and blue blocks travel left on a conveyor. Sort them into the matching bins, which are between the conveyor and the robots. The submitted runs use the original station camera.

The station table is z=0.7475 m; x is forward from the arms and y is left. A dark rubber-like 0.12 × 1.20 × 0.02 m belt is centred at (0.34, 0, 0.7575), moving in +y at 0.025 m/s. Ten yellow visual bands move with it. Fixed red and blue plastic-like bins are centred at (0.15, 0.22) and (0.15, -0.22). Each has a 0.14 × 0.20 × 0.01 m floor and 0.01 m thick walls rising 0.06 m above the table.

Five 0.04 m plastic-like cubes, each 0.05 kg, begin on a fixed-height loading shelf at y=-0.50 m, z=0.7875 m, with x=0.46, 0.55, 0.64, 0.73, 0.82 m in the order red, blue, blue, red, red. At 0, 16, 32, 48 and 64 s, respectively, each block slides along the shelf at 0.06 m/s toward x=0.34 m; once it reaches the belt, PhysX carries it in +y. A missed block passing y=0.60 m at the far end is removed from play to prevent accumulation. Reset restores the blocks and arms. The shelf transfer is a kinematic approximation to a feeder and the removal approximates a reject chute; neither mechanism is modelled physically.

The belt is a kinematic collider with PhysX surface velocity. CPU PhysX is required: in a GPU-physics test, blocks fell through the conveyor. The RTX GPU still renders cameras. Rollers and deformation are not modelled. The submitted runs use Dream’s original calibrated overhead station camera; the setup screenshot uses a raised view to show the loading shelf and belt. Wrist cameras retain their normal mounts. Cameras render at 640 × 360 and supply 480 × 270 frames to the policy and recording.

## Run

The tested environment used yam-arena commit `0059d02a1a91bc6834c1d0846022891df0c78730`, Isaac Lab-Arena commit `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`, Isaac Sim 6.1.0.0, inspect-robots 0.60.0 and inspect-robots-dream 0.1.0. Install yam-arena and its dependencies as in its guide. From this folder, in the Arena environment:

```sh
uv pip install -e . --no-deps
python -m pytest -q test_score.py
python check_conveyor.py
TASK="$PWD/task.py"
INSTRUCTION='Sort the red and blue blocks from the moving conveyor into the matching colored bins.'
inspect-robots run --embodiment yam_conveyor --policy dream \
  -E task="$TASK" -E view=original -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=480 -E cam_height=270 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir dream
inspect-robots run --embodiment yam_conveyor --policy yam_scripted \
  -E task="$TASK" -E view=original -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=480 -E cam_height=270 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir demo
for log in dream/*.json demo/*.json; do inspect-robots video "$log"; done
```

The run commands select the original calibrated station camera; the right arm partially obscures the loading shelf from that angle. The default `view=raised` shows the entire shelf. These are separate simulation runs, and Dream may act differently when it sees a different view.

The loading schedule is fixed and independent of the seed; `object_jitter=0` disables reset jitter. The 2,400-step limit is 120 seconds of simulated time at 20 Hz. With the default raised view, the scripted demo scored 1.0 after 2,011 steps and took about 216 seconds wall-clock on the L40S VM; Dream-1 scored 0.0 at 2,400 steps and took about 285 seconds. With `view=original`, a separate scripted run scored 1.0 after 2,011 steps and took about 234 seconds, and a separate Dream-1 run scored 0.2 at 2,400 steps and took about 280 seconds. Runtime varies with hardware and policy latency.

## Score

At the end of the run, each correctly placed block earns 0.2. To count as placed, its entire horizontal bounding box must be inside its matching bin's 0.12 × 0.18 m interior; its bottom must be within 0.012 m of the bin floor at z=0.7575, and its linear speed must be below 0.01 m/s and angular speed below 0.5 rad/s. If all five meet the criterion for one second, the run ends at 1.0.

For each other block, lift credit requires a gripper opening below 0.8 on the 0–1 scale and a tool-centre distance no greater than 0.06 m from the block. Let `L = clamp((bottom_z - 0.7675)/0.04, 0, 1)` and `P = clamp(1 - horizontal_distance_to_matching_bin_center/0.45, 0, 1)`. Its partial score while held is `0.2 × L × (0.2 + 0.78 × P)`. This gives continuous credit for lifting and moving toward the matching bin; the closed-gripper proximity check approximates a grasp rather than measuring fingertip forces. The score is evaluated at the end, so a dropped block loses lift credit. Staged, rejected, or piled blocks earn no lift credit without a nearby closed gripper.
