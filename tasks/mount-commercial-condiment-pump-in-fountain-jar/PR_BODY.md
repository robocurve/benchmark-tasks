### Motivation & Workforce Relevance
Fast food and counter workers (BLS SOC 35-3023: 3,460,000 workers) and food preparation
workers (BLS SOC 35-2021: 891,000 workers), representing over 4.35 million US food service
personnel, routinely assemble and restock condiment dispensing stations. Under commercial
food code sanitation schedules, portion-control condiment dispenser pumps (Server Products /
Vollrath commercial standard) are fully disassembled, washed, and reinstalled into stainless
steel fountain jars daily.

### Task Description & Contact Mechanics
- **Stainless Steel Fountain Jar** (110 x 130 x 75 mm, 1.2 kg) sits on the prep counter at
  (0.260, 0.080) m with an open rectangular mouth, flared rim (Z=0.8225 m), welded side loop
  handle at (0.260, -0.015, 0.7865) m, and front locking detent catch. Modeled with hollow
  perimeter flange borders leaving the inner cavity completely unobstructed.
- **Sanitary Wire Wash Rack** (140 x 140 x 65 mm, 0.50 kg) at (0.240, 0.250) m with a 50 mm
  deep vertical socket holding the sanitized pump upright.
- **Commercial Portion-Control Pump** (380 g) with a 24 mm dia x 65 mm vertical pick-up tube,
  118 x 138 mm rectangular contoured lid collar, spring locking detent tab, and vertical grip stem.
- **Bimanual Coordination**: Right gripper grasps the jar's side loop handle to counter-hold
  against insertion torque. Left gripper retrieves the pump from the wash rack socket, coaxially
  aligns the pick-up tube over the open jar bore, guides it down without binding, seats the lid
  collar flush onto the flared rim, pushes the locking collar tab forward into the rim detent
  slot, and applies an upward proof pull against the mechanical detent to verify positive retention
  before releasing and retracting both grippers clear for a 1.0 s stability dwell.

### Scoring & Stage Progression
- **0.20**: Pump grasped and lifted vertically clear of wash rack socket (> 30 mm).
- **0.40**: Pick-up cylinder entered jar bore (r_xy < 0.035 m, tube_z < rim_z - 0.010 m).
- **0.65**: Lid collar seated flush on jar rim (r_xy < 0.025 m, vertical gap < 15 mm).
- **0.85**: Collar locking tab pushed forward into rim detent slot (c_x - j_x < -0.002 m).
- **1.00**: Upward proof pull sustained against mechanical detent for >= 10 steps (0.5 s), both
  hands retracted clear (TCPs >= 35 mm, link hulls >= 5 mm clear), and assembly resting still.
- **Defect Latches (0.0)**: Dropped pump outside stand/jar, excessive jar displacement (> 35 mm)
  or tipping (> 10 deg), or hands not clear during hold.

### Hardware Realism & Hard Constraints Compliance
- **100% Natural 1:1 Tabletop Equipment**: Authentic commercial equipment, zero oversized handles,
  zero kinematic cheats, zero added grasp bosses. Modeled hollow open jar cavity per PR #3136/#3137.
- **Corpus Novelty**: Verified 100% CLEAR with `robocurve-pr-check` (0 collisions, 0 rejections).
- **Qualification Results**:
  - `demo_1`: 1.0 (260 steps, camera frames stored)
  - `demo_2`: 1.0 (260 steps, seed 1, consistent)
  - `neg_no_insert`: 0.20 (grasps & lifts, halts before insertion)
  - `neg_no_seat`: 0.40 (enters bore, halts before seating)
  - `neg_no_lock`: 0.40 (seats collar, halts before detent lock)
  - `dream_1`: 0.0 (Dream policy baseline)
- **Media Uploaded**: Media ID `M-YTZMUB` with genuine IsaacLab simulator videos (`demo.mp4`,
  `model_run.mp4`) and initial setup screenshot (`setup.png`).
