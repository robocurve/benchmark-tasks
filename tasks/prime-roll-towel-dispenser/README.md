# Prime a commercial roll-towel dispenser after a refill

A fresh roll is already seated in an open wall-mounted dispenser. Position its loose leader at the roller nip and work the return lever until an intact, straight tail hangs from the outlet. The task ends at the feed check while the housing is open for inspection.

Restocking public washrooms is a core duty of janitors and cleaners. O*NET 37-2011.00 lists supplying restrooms and BLS OEWS counted 2,209,760 US jobs in this occupation in May 2025. Employment is reported for the occupation. The San Jamar Element loading sheet documents placing the paper in between the two rollers and operating the lever to feed it through before shutting the cover.

- [O*NET 37-2011.00](https://www.onetonline.org/link/summary/37-2011.00)
- [BLS May 2025 employment release](https://www.bls.gov/news.release/pdf/ocwage.pdf)
- [San Jamar Element paper loading instructions](https://cdnimg.webstaurantstore.com/documents/pdf/101838_elementinstruction.pdf)
- [Manufacturer's dimensional sheet](https://www.webstaurantstore.com/documents/specsheets/element-sell-sheet.pdf)

The [accepted twin-roll tissue task](https://tasks.robocurve.org/board?task=TK-22CFQJ) relocates a usable roll, installs a refill and selects the correct outlet. Its tissue tail starts threaded. The [accepted soap refill](https://tasks.robocurve.org/board?task=TK-Z3X5F5) works a latch and seats a rigid refill. This task tests the alignment and feed of a wide, loose paper web into a roller pair. An empty lever stroke leaves the outlet empty.

## Scene and approximation

The housing is a full-width unbranded reconstruction. See task.json for the reset, dimensions and directions and task.py for every collider. The thick mounting panel is secured to a stand behind the housing. The cradle and roll start fixed. The return lever is a native hinge with limits from −38 to 0 degrees, spring 1.5 N m/rad, damping 0.05 N m s/rad, mass 80 g and estimated dry drag 0.016 N m. The roller pair has fixed collision geometry.

The paper model represents the free edge by two sample points 150 mm apart and 40 mm up from the tip. A gripper can acquire a point only on closing around the thin web, aligned with its normal. Both edges have to enter the nip together, with at most 8 mm of height difference. Once seated, the modeled roller transmission advances the paper 84 mm per radian of downward lever rotation beyond 2 degrees of backlash. A stroke must return within 1 degree of the top before it can advance further. The roll does not unwind continuously under its own weight. The paper shape is visible but simplified, with no force feedback, buckling, roller deformation, measured transmission ratio or material tearing calibration. A pair of grippers pulling the samples out of their allowable separation produces an invalid outcome. The grasp and feed checks were exercised in the recorded simulation. The feed ratio and tear threshold are estimates.

The ABS housing is 318 × 324 × 198 mm with 8 mm sides at y = ±155 mm. An 18 × 410 × 410 mm rigid mount is centered at (557, 0, 952.5) mm. The open ABS cover is 195 × 310 × 6 mm with center (247, 0, 813) mm. The cellulose roll has a cardboard core 36 mm in diameter and the shaft spans 218 mm. The two nominal 39.6 mm diameter rubber rollers run along y. The right-hand ABS lever has a 75 mm arm and a 32 mm diameter rounded end. The feed outlet is centered at x = 428 mm, with its lower lip at z = 821 mm. Place the fixed assembly and arms as shown in the setup frame.

Use a secured dispenser with the lid open and a fresh, square paper edge. Center the roll on the cradle and pull a loose leader down to the starting point in task.json. Check seating from the open front by looking for both sides of the leading edge inside the nip, with their height difference at most 8 mm. For partial credit at 0.5, measure at least 15 mm of the connected paper extending below the lip. For full credit, use the ruler to confirm at least 50 mm along the paper below the lip, within 3 degrees of released lever rest and both fingertips clear of the paper and lever for 0.5 seconds. Record the outlet in a low side view when judging a physical trial. A torn or detached leader earns zero. Reset the lever to rest and use a fresh intact leader between trials.

## Inspect commands

Use the official YAM/Arena simulation environment. Source the installed Arena environment and run from the task directory. The demonstration plays fixed waypoint targets through yam_scripted, with no live object poses or scoring feedback. The evaluation uses overhead and two wrist RGB views and standard robot joint and gripper readings. The scorer's geometric samples stay within the simulation.

```bash
source "$ARENA_ENV"
inspect-robots list embodiments
inspect-robots list policies
python -m unittest test_feed
bash run_demo.sh 200 smoke
bash run_demo.sh 1200 demo
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false \
  -E "task=$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 400 \
  --store-frames --log-dir model_run --scorer task_score \
  --instruction 'Start the new roll in the open paper-towel dispenser. Feed a straight, unbroken tail through the outlet and leave the lever released.'
```

Set `ARENA_ENV` to the installed Arena environment script before running the commands. The paid policy uses the Dream SDK's standard credential environment variable. The run limit is 1,200 steps. The submitted Dream-1 evidence is a 400-step bounded run with no seating (0.0). The scripted run scored 1.0 in 722 steps (36.1 simulated seconds; approximately 2 minutes on an NVIDIA L40S including setup and recording). The tail extended 64.5 mm below the outlet and the scored release persisted for 10 control steps.

Evidence uses seed 0, no object jitter, 20 Hz and 320 × 176 policy frames from the three cameras. Native camera frames are 640 × 360. The top view shows the starting paper and hand placement, the left wrist views the nip and outlet, and the right wrist shows lever travel. The lowered paper is partly obscured by the front lip in the overhead view. A reset after the demonstration restored the paper, counters, joint positions, root poses and velocities. The fresh RGB frames reproduced the initial scene with small rendering differences. The gripper drive effort setting is 40 N per finger, the scripted opening command is 74.9 mm, and the stock start opening is 93.9 mm. I2RT specifies a 95 mm throw for the YAM linear gripper.

The recorded environment used Inspect Robots 0.60.0, the Dream adapter 0.1.0, YAM Arena revision `0059d02`, and Isaac Lab-Arena revision `c7edf3a9d5`.

The external YAM robot USD and meshes are MIT licensed and used through the official [YAM Isaac Lab-Arena adapter](https://github.com/kairosiann/yam-isaac-lab-arena), which carries the I2RT license notice. They are installed separately. The procedural dispenser geometry and simulation code use MIT. The local copy of mechkit.py comes from the MIT-licensed brake-pads-caliper example in the benchmark repository.
