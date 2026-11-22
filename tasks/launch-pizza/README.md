# Launch a pizza from a peel onto an oven stone

A home pizza oven has a low opening and a stone recessed inside. The baker carries the topped pizza level on the peel, brings its leading edge to the stone, then gives the handle a short, sudden backward pull. That stroke removes the blade while the stone takes the dough's weight. The scripted demo commands the pull over 0.43 seconds. Scoring uses the transfer outcome, so other withdrawal speeds can earn full credit when the dough settles flat with its toppings in place.

Oven-rack tasks test alignment and seating of rigid hardware. Cake unmolding releases food by inverting its container. BEHAVIOR-1K's *Make Pizza* starts the dough on a cookie sheet and tests preparation, and RoboCasa's *Cut Buffet Pizza* tests slicing. Here the robot must bring a flexible, topped load through the oven mouth and pull its support straight back at the right moment. The early-withdrawal control uses the same pull before the dough is far enough over the stone and scores zero.

The instruction is “Launch the pizza from the peel onto the oven stone.”

## Physical setup

Use an Ooni Karu 12 with its cordierite stone installed and its door removed. Run it cold on a level workbench with no fuel or burner attached. The stone is nominally 330 × 330 × 15 mm, with a 338 × 99 mm mouth. Use a stock Ooni 12-inch bamboo peel, about 505 × 305 × 12 mm and 717 g. Use its normal handle. [Oven](https://eu.ooni.com/products/ooni-karu) · [Peel](https://ooni.com/products/ooni-wooden-pizza-peel) · [Launch method](https://uk.ooni.com/pages/getting-started-cook)

Use coordinates relative to the YAM station with x forward, y left and z up. The oven stands on a 747.5 mm high bench. Center the stone at (642, 0) mm in the horizontal plane with the mouth facing the arms. The stone top is 842.5 mm above the floor in simulation. For a physical setup, use the stone top and the front edge of the oven as the transfer datums.

Set an ordinary level hardwood prep worktop in front of the cold oven. Its top is 500 × 460 mm and is 845 mm above the floor in simulation. Its oven-facing edge is at x=450 mm, with a 10 mm gap to the front of the mouth. In the physical setup, set the worktop 2–3 mm above the stone. A separate height-adjustable bench with a flat worktop can set this height. Place the stock peel flat on the worktop, with the blade center at (285, 0) mm and its leading edge pointing into the oven. The worktop supports the weight during the approach.

Prepare a 240 mm raw dough round from a 210 g portion of refrigerated, premade plain pizza dough. Bring it to 21 ± 2 °C before shaping and use the same dough product across a batch. Stretch against a 240 mm template and leave a narrow raised rim. Distribute 1.0 ± 0.2 g of semolina flour over the peel blade away from the rig. Before topping, gently shake the peel to check that the dough moves freely, then center it 46 mm toward the leading edge of the blade. The front of the dough overhangs the blade by about 5 mm. Place four 28 mm pepperoni slices at the corners of a 116 mm square centered on the dough. Keep the handle and gripper pads clean and dry. Replace the dough round each trial and wipe the blade, stone and prep top before resetting. [Ooni's dough recipe](https://eu.ooni.com/blogs/recipes/classic-pizza-dough) · [Launch and shake method](https://ooni.com/blogs/ooni-insights/9-tips-for-launching-the-almost-perfect-pizza)

## Scoring

Inspect the final state after it settles. Measure the uncovered stone edges, the crust's bounding rectangle and the mouth with a ruler or scaled photo. A ruler placed on the cold stone and an inspection photo taken through the mouth help check the final position.

- **0.0** if the trial video does not show the peel carrying the crust through the mouth, if any dough is left on the peel, if the peel still crosses the mouth, or if the crust extends more than 3 mm past a stone edge. Also score zero if the crust is folded, evidenced by a horizontal span under 208 mm in either axis or an edge more than 28 mm above the stone.
- **0.5** if the complete crust settles flat on the stone with the peel clear.
- **1.0** if the 0.5 conditions hold, the center of the crust's bounding rectangle is within 45 mm of the stone center, and all four slices rest on the crust with each slice center within 103 mm of the crust center.

The simulator checks that the crust enters at stone height while supported above the blade, then measures the settled deformable nodes, topping poses and peel pose. It uses a 28 mm/s nodal speed limit for settling and small clearances for contact between the crust and stone.

## Running

With Inspect Robots and YAM Arena active, run from this directory.

```bash
python task.py
export WARP_CACHE_PATH="${TMPDIR:-/tmp}/pizza-warp-cache"
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E headless=true -E control_hz=20 \
  --instruction 'Launch the pizza from the peel onto the oven stone.' \
  --scorer task_score --max-steps 1200 --seed 0 --store-frames --log-dir /tmp/pizza-run
```

The trial cap is 60 seconds at 20 Hz. The nominal waypoint plan takes 12.2 seconds. The score must hold for one second to end the trial early.

## Assets and fidelity

`assets/oven.usdc` (1.2 MB) and `assets/peel.usdc` (83 KB) adapt NVIDIA USDCraft-10K assets `ds_R359_08` and `ds_R647_01`, respectively. Both are Apache-2.0. The adapted oven mimics the dimensions and aperture of the reference home oven. The peel retains its stock handle. [USDCraft-10K](https://huggingface.co/datasets/nvidia/USDCraft-10K)

`assets/crust.usdc` (24 KB) is an authored deformable volume with an estimated material model for cold dough. `assets/topping.usdc` (1.6 KB) is a rigid slice instanced four times. The crust bends but does not tear. Its painted surface represents the color of a topped pizza. Dusting is represented by lower blade friction, and the handle keeps its original contact material. The oven stays cold.
