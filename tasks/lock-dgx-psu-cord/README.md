# Seat and lock the DGX H100/H200 PSU-end power cord

**Impact on AI timelines.** NVIDIA's H100/H200 service procedure calls for seating the system-end connector while its gray ring is unlocked, then turning that ring independently to lock the seated cord. This prototype puts an inactive rear-panel section face up on a bench so the stock top-down YAM demo can reach the collar. The separate ring and keyed opening carry the manipulation. [NVIDIA PSU replacement manual](https://docs.nvidia.com/dgx/dgxh100-service-manual/psu-replacement.html#locking-power-cords) · [DGX H100/H200 user guide](https://docs.nvidia.com/dgx/dgxh100-user-guide/dgxh100-user-guide.pdf)

## Place the parts

The task uses the standard station with tabletop height 747.5 mm. All positions here are relative to the tabletop datum. Positive X is forward and positive Y is left.

| Part | Location and build |
| --- | --- |
| Rear-panel slice | Center at X 300, Y 10 mm. The 222 × 154 mm bench plate carries a C20-style recess at Z 36 mm. Two metallic PSU faces and stationary neighboring plugs are at X ±75 mm from center. The recess and three visual blades reference the three-contact C20 layout. |
| Target connector | Starts at X 222, Y −168, seat datum Z 23.3 mm, unlocked. The C19-style nose is 28 × 20 × 18 mm. The tapered black backshell is at most 36 × 30 mm. The gray band is Ø37 × 18 mm before the ribs and retention tabs. |
| Pickup cradle | Clamp a 52 × 44 × 5 mm HDPE or printed PETG base to the bench. The four 7 mm walls form a 30 × 22 mm open pocket and rise 15 mm from the floor. The nose rests in the pocket and the connector lifts straight out. For a measured cable, size the pocket to leave 1 mm clearance on each side of its nose. |
| Cable | The plug has an axial strain relief followed by 284 mm of articulated jacket. The last section starts just above the table. The shipped DGX cord is 1,200 mm long end to end. |

## Check against DGX hardware

| Item | Evidence and remaining measurement |
| --- | --- |
| Connector family and action | NVIDIA specifies C19/C20 and shows the tapered backshell, ribbed gray collar and lock indicators. The simulation uses a black axial backshell, separate gray collar and white/orange witness marks. [NVIDIA cord specification and images](https://docs.nvidia.com/dgx/dgxh100-user-guide/introduction-to-dgxh100.html#dgx-h100-h200-locking-power-cord-specification) |
| Mating face | The modeled 28 × 20 mm nose uses the nominal face envelope from a commercial C19 drawing. The 18 mm insertion depth, 0.9 mm per-side clearance at the lower throat and contact geometry are model values. Measure the de-energized DGX plug and matching inlet for actual insertion depth, fit and contact positions. [Quail C19 drawing](https://www.quail.com/content/Quail/pdf/DimSpecs/IEC-60320-C19_site.pdf) |
| Collar and rear panel | The collar diameter and width, 36° throw, cleats, detent and seating forces are estimates. Record the collar with calipers, the rotation from unlock to lock, and inlet center spacing and neighboring clearance from a retired DGX or matching service parts. The bench orientation and short simulated cable change access and drag. [NVIDIA rear panel](https://docs.nvidia.com/dgx/dgxh100-user-guide/introduction-to-dgxh100.html#rear-panel-modules) |

Use a physically isolated OEM cord and matching PSU inlet for the dimensional comparison. This folder is a simulation pilot pending that measurement and a loaded Isaac trial.

## Scoring and reset

- **0.25** once the correctly aligned plug enters while unlocked. The code checks depth from the socket face and orientation as a seating predicate.
- **0.60** once seated within the 2.5 mm final gap tolerance, with the unlocked-entry sequence satisfied.
- **1.00** when seated and at least 31° of collar rotation, with neither hand touching the plug, collar or cable and one second of stable hold.
- **0.00** for a locked entry, forcing through the stop plane, or pulling the plug out while a confirmed lock remains engaged.

Use an overhead or side frame to see the flange gap and the ring index. The ring is a real revolute body driven by hand contacts. A small detent torque and under-cleat retention force approximate the OEM retention. Each reset restores the authored plug/collar pose and clears the sequence and hold state.

## Run

From this task folder, run `python task.py --ik` to check scripted waypoint reach. Use the task-local trial commands below for simulation. The run limit is 1,200 steps at 20 Hz. The older unsupported setup tipped before pickup. In the revised scene check the overhead recording and console against the starting XY (−78, −178) mm relative to the socket through 2.5 seconds. For the scripted pickup, look for less than 2 mm of lateral drift, less than 6° tilt, and a clean vertical lift before the arm travels toward the socket.

The cable uses five bounded alternating-axis hinges, shape-derived inertia and implicit damping. The revised demo stages the lift and slows the transfer. Watch `lead_mps` during pickup and carry. The full revised Lambda run reached 1.00 with a stable released lock.

The folder has `task.py`, `plan.py`, the lab's `scenekit.py` and `touch.py`, `task.json`, `setup.usda` and three generated USD assets for the bay, cord and cradle.

**Overlap screen.** Compare a single-piece bayonet cap and a retained DAC connector against seating a keyed plug followed by turning its independent collar. The face-up setup must preserve a meaningful reach or retention constraint. Check the live board before submission. [Bayonet cap #34](https://github.com/robocurve/benchmark-tasks/pull/34) · [DAC #2554](https://github.com/robocurve/benchmark-tasks/pull/2554) · [DataCenterBench setups](https://robocurve.org/datacenterbench/setups)

## V6 handoff

The carrier pauses after insertion, opens fully at the seat, raises the open pads vertically, then withdraws to +X away from the cable bend and parks. The second arm waits until that retreat is complete. A bounded resistance to upward motion models the inlet spring contacts while the keyed nose remains mated. Check `gap`, `tilt`, and `ring` during this transition. The full v6 contact sequence needs a Lambda trial.

## v7 changes

The scripted carrier aligns the held plug against its measured inlet error, with a 7 mm entry bevel leading to the keyed throat. The right hand retains the plug unless the seated-unlocked condition is met. The left hand waits for the carrier to finish withdrawing. Inspect `mouth_error_mm`, `gap_mm`, `guided_xy_mm` and `ready_to_release` in the console log. The bench face geometry and connector tolerances still need comparison with OEM parts.

## Lambda insertion tuning

The revised bench connector shortens the taper below the parallel grasp flats. The keyed inlet uses a 14 mm lead-in and a lower throat with 0.9 mm nominal clearance per side across the two face dimensions. The socket and collar use a 0.27 mm PhysX contact offset. Collar cleat clearance retains the mechanical capture feature. During insertion the scripted carrier trims X, Y, Z, yaw and tilt using the simulated plug pose. The console reports mouth error, flange gap, insertion depth, tilt, yaw and cable-link speed.

## Run the trials in YAM Arena

From this task folder, activate the existing Arena environment and load the instruction and limit from the task specification. Use a clean reset, seed 0 and zero object jitter. Both trials run at 20 Hz with the `task_score` scorer.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
```

### Scripted demo

The recorded demo used the graphical display and 640×352 frames. Set `-E headless=true` to run without a display.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E object_jitter=0 -E headless=false -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/demo
```

### Dream-1

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E object_jitter=0 -E headless=true -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/model_run
```

Read the final `task_score` under `metrics` in `inspect-robots inspect <saved-eval.json>`.

The scripted demo scored **1.0** in the saved evaluation `adhoc_af7c6178.json`.

Dream-1 scored **0.0** in the saved evaluation `adhoc_253294b0.json`.
