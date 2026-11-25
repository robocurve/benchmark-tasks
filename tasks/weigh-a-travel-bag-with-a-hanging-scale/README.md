# Hook a hanging travel scale onto a bag and hold it steady

The robot raises the hinged carry handle, threads the open metal hook under it, lifts the whole bag and holds the scale until the reading locks. It then lowers the bag to the bench and hangs the unloaded scale back on its peg so the locked weight can be read. The scale measures the load while it hangs freely. An unstable swing or a hand supporting the bag can change the reading.

## Physical trial

Use a handheld digital hanging scale with an open metal hook and automatic lock. The Travel Smart TS601X uses that hook and retains the stable reading for 60 seconds after weighing. The manual leaves its minimum lock load unspecified. The PCE-HS 50 N is an alternative with an open hook, a published 200 g minimum, 20 g divisions and an automatic hold that keeps the reading until reset. It uses a taller housing and weighs about 400 g including its fittings. For a substitute, adjust the peg height to keep the unloaded hook bowl 9.2 cm above the bench. [Travel Smart instructions](https://www.conair.com/on/demandware.static/-/Library-Sites-usConairShared/default/dw85d8da44/Instructional%20Booklets/TS601X.pdf) · [PCE instruments instructions, §§2 and 6.7](https://www.pce-instruments.com/us/api/getartfile?_dsp=inline&_fnr=1090788)

Use a 23 × 18 × 11 cm zippered travel or electronics case with a hinged top carry handle. Pack and weigh the case to 642 g within 15 g on a reference bench scale. Fold the handle toward the robot's left. Its open clear interior is about 110 mm wide and 34 mm high. Use a rigid Ø8 mm peg on a bench stand with 83 mm of exposed reach along the y axis. Set its center 27.6 cm above the table and centered at x 35.4 cm as in `task.json`. Hang the already zeroed scale by its top loop, with the peg pointing toward the bag.

The heavier scale option and packed bag total about 1.04 kg, within the YAM arm's 2 kg nominal payload. [YAM arm specifications](https://i2rt.com/products/yam-6-dof-arm)

Before using a physical unit, zero it and hand-weigh the packed bag three times in automatic hold mode. Each run must lock within 85 g of the 642 g target and retain the same reading for at least 5 seconds after unloading. Reset and tare before each check. This verifies low-load locking and retention on the actual unit. The simulation uses the same task mass and reading tolerance.

Film continuously with both hands, the hook-to-handle contact and the whole bag visible. Position the side camera near bench height, slightly to the side of the scale-face camera, and place a ruler with a 25 mm mark behind the lowest edge of the bag. Use a synchronized close view of the display if needed to see when the reading locks. Score the tiers cumulatively.

- **0.25** once the bag is entirely at least 25 mm above the bench and its handle hangs from the hook alone. In the video, both grippers and the peg must stay clear of the bag and hook. The gripper may hold the scale body.
- **0.6** when a reading within 85 g of the 642 g reference locks while the bag still meets the 0.25 conditions. The clearance and sole support must continue through the settling interval shown before the lock. A reading from pulling the hook or steadying the bag with the other hand cannot qualify.
- **1.0** after earning 0.6, gently lower the bag onto the bench and hang the unloaded scale back on its peg with the same lock reading. Both hands are clear and the state holds for one second.

A drop after lifting zeros the run. For reset, fold the handle, hang the scale, select lock mode and tare without a load. The earlier benchmark task tests estimating an unknown solid block's mass with an uncalibrated spring. RoboCasa uses a fixed countertop scale. This task tests threading an open hook through a hinged carry handle and stabilizing the freely suspended bag before setting it down. [Earlier benchmark task #77](https://github.com/robocurve/benchmark-tasks/pull/77) · [RoboCasa ScalePortioning](https://robocasa.ai/docs/build/html/tasks/composite_tasks.html)

## Simulation

Gravity loads an extensible contact-driven hook. A load-cell spring converts slide deflection to mass. The display latches the median of 43 consecutive upright samples when the housing position stays within an 8 mm span and the readings stay within a 50 g span. The scorer independently requires at least 3 N of upward hook-to-handle normal force, competing contact forces at or below the 0.10 N sensor tolerance and 25 mm of clearance under every part of the bag. The support checks must hold throughout the measurement window, and the bag must be steady when the display locks, before a correct reading can earn 0.6. A spoofed reading can appear on the display and earns no measurement credit. Scale and bag geometry are generated from code. A hinged carry handle represents flexible webbing, and the spring and lock window approximate real scale electronics.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=weigh_a_travel_bag_with_a_hanging_scale.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Weigh the blue travel bag with the hanging scale. Hook it by the carry handle, lift the bag clear, and wait until the weight locks on the display. Lower the bag gently onto the bench. Return the unloaded scale to its peg with the reading still locked." \
  --scorer task_score --max-steps 1200 --store-frames
```

Use `--policy dream` for the independent model run on the same fixed layout. The run limit is 60 simulated seconds. With seed 0, the full scripted motion earns 1.0 in 464 steps with a locked 0.64 kg reading. The `BAGSCALE_LOW_LIFT=1` control also locks at 0.64 kg, with about 17 mm of clearance. It scores 0.0. RTX rendering takes a few wall-clock minutes per run. `BAGSCALE_UNSTEADY=1` lowers the bag immediately after lifting, before the readout can lock.
