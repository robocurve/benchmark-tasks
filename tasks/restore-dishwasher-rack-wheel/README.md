# Restore a dishwasher rack that comes off its track

Press the loose wheel back onto the dishwasher rack, then roll the rack into the tub on both tracks and let go. A backed-out front roller can fall off as the rack moves across the door joint. Reseating it restores rolling support at that corner. [Manufacturer appliance specification](https://www.danby.com/en-us/shop/ddw621wdb/) · [Replacement wheel](https://www.danbyapplianceparts.com/product/basket-wheels/)

## Setup and reset

The reference is the Danby DDW621WDB countertop dishwasher and its 12176000008767 roller. Turn it off, unplug it, drain any standing water and empty the rack. Open the door all the way on a level counter. Set the front face at x = 410 mm and y = 20 mm, facing negative x. The cabinet is 550 mm wide, 500 mm deep and 438 mm tall. The open door extends 360 mm from the face. Two 28 mm running surfaces support the basket, with their centerlines at y = ±220 mm relative to the appliance and their height 73 mm above the counter. The 4 mm steel guide lips sit on centerlines 10.5 mm and 9 mm outside the left and right running tracks, respectively. Mark the two sides of the hinge crossing at 37 mm outside and 38 mm inside the face.

Use a 390 × 460 mm wire basket that weighs about 0.8 kg. The two axle rows are 290 mm apart. Start with the rack center 80 mm in front of the cabinet face, the three sound wheels on their tracks and the left front roller 3.65 mm outward from its seated datum. Stand the loose wheel on its original rail with its bore axis lined up with the rack axle. The modeled wheel is 34 mm in diameter and 9 mm across the tread. The simulated split hub has two spring lips. The hub, axle and rails use estimated dimensions. The rack's structural bars contact the robot, and the smaller wire dividers are visual. The appliance stays off.

For a real trial, mark the roller with contrasting paint at the spoke and the corresponding tread so a camera can see rotation. Use the same undamaged axle and wheel for every reset, with the hub unloaded when backing out the wheel. Verify it is clear of the retaining lip before starting. Restore the starting positions and clear the robot, scorer and all three camera buffers. Keep the counter around the rack clear.

Use a 47 g oak press block with a 31 × 20 mm flat contact face, a 40 × 32 mm foot and a 28 × 29 × 39 mm rectangular grip. Its center at reset is (180, 340) mm, with the foot on the counter. Use its flat face to push the roller axially. If you use the block, put it back on the counter before moving the basket.

## Score

Use a side view to inspect the wheel seated squarely over the axle, with the axle and hub axes within 9° and the radial and axial offsets within 2.5 mm. For initial credit the seated roller supports the rack on the track and stays clear of the grippers for 0.5 seconds. For the next tier record the repaired wheel rolling between the two hinge marks, with its lowest point within 3 mm of the rail running surfaces. At least 75 mm of travel must occur while both tracks support the rack. The observed wheel arc must match the translation within 14 mm throughout the crossing. For full credit, the basket's front lower bar must be at least 5 mm inside the cabinet and stationary on its tracks with both arms clear for 0.5 seconds. Tipping over 15°, leaving the work area or damaging the cabinet, rack or wheel gives no credit. The contrasting wheel and hinge marks make carrying and dragging distinguishable from rolling.

The external I2RT YAM arm and gripper assets are MIT licensed. The task uses the stock Isaac Lab-Arena YAM adapter. The procedural assets and helper use the included MIT license. [YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run and score

The fixed scene uses seed 0, zero object jitter, three 320 × 176 RGB streams, and 360 controls at 20 Hz. The scripted demonstration uses fixed arm waypoints and timings. Inspect records the terminal `task_score`.

```sh
python -m pytest -q test_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 360 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction 'Press the loose wheel back onto the dishwasher rack, then roll the rack into the tub on both tracks and let go.' \
  --log-dir rollout
```

## Evaluation results

Both runs use seed 0, zero jitter, three RGB views at 320 × 176, and a maximum of 360 controls at 20 Hz. The Dream-1 adapter also receives ordinary joint and gripper readings. Physics runs at 480 Hz. Recorded software versions are Inspect Robots 1.0.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0 and PyTorch 2.11.0+cu130.

| Run | Terminal score | Controls | Wall time |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.00 | 357 | 159.7 s |
| Dream-1 | 0.00 | 360 | 168.6 s |

For the model run, use the same command with `--policy dream -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90` in place of `--policy yam_scripted`.

The repaired wheel rolled 78.3 mm at the rim while its axle traveled 80.9 mm across the hinge. The rack remained on both tracks with the grippers clear for 3.45 s. The physical state and camera reset checks passed. The flat block briefly occludes the hub during the push. The left wrist view shows the roller away from the block and through the track transition.
