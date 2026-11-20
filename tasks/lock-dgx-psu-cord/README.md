# Seat and lock a rotary C19 cord from a rear service panel

Local simulation prototype for a de-energized rack slice. Three neighboring positions stand in for part of the six-PSU lower row. The inlet face is vertical and all plug axes are horizontal. The loose target plug starts on a staging support, approaches from the rear, and exits toward the operator. Two occupied bays constrain lateral access.

## Hardware evidence

| Detail | Revision and status |
| --- | --- |
| Rear arrangement | NVIDIA's DGX H100/H200 guide documents six PSU slots. Its rear photograph shows one bottom row and horizontal mating. The 79 mm center spacing is a prototype estimate from the proportions of this image, with 482.3 mm documented maximum width as a reference. |
| Installed operation | The H100/H200 service guide documents unlocked insertion and a gray collar with a closed-padlock indication. The rear-facing cable exit is inferred from the system connector view. The cable's installed bend and reach are unknown. |
| Geometry and forces | The guide identifies C19/C20, but a matching OEM part pair, internal contact geometry, usable seating datum, rotary travel and operating resistance could not be established. socket dimensions, retention surrogate, resistance, shortened lead, staging support and radial manipulation pad are provisional. |
| Scoring references | The position of the physical socket floor defines the simulation seating witness. The visible collar index aligns with the modeled locked detent. Their numerical tolerances are simulator resolution allowances. |

[NVIDIA rear-panel and cord reference](https://docs.nvidia.com/dgx/dgxh100-user-guide/introduction-to-dgxh100.html#rear-panel-modules) · [NVIDIA PSU service sequence](https://docs.nvidia.com/dgx/dgxh100-service-manual/psu-replacement.html)

## Simulation scoring

Keyed entry is worth 0.25. The authored fully seated witness is worth 0.60 after unlocked entry. Full credit requires the lock indication and aligned seat witness continuously while both hands are clear for one second, with at most 0.8 mm of plug drift and 2° of body or collar rotation during that interval. Premature lock indication, forced insertion or loss of seating after confirmed lock invalidates the trial. The deliberate controls preset the cord geometry. The backout control uses a fixture drive to rotate the seated collar and then imposes an artificial cable load with the hands clear.

Run from the task folder after activating the installed Arena environment. Use `FAILSAFE_CASE` to select a negative control (partial-turned, unlocked, backout). The cap is 1,200 steps.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
inspect-robots run --policy yam_scripted -P hold_s=2 --embodiment yam_arena --scorer task_score \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=256 -E cam_height=144 --seed 0 --instruction "$(jq -er .instruction task.json)" \
  --max-steps "$steps" --no-prompt --store-frames --log-dir logs/demo
```

Run `python task.py --ik` for the scorer and waypoint regressions.

## Commercial reference and physical trial

Use a [SSEMCO 620890 Dual-Lock rotary C19 cord](https://ssemco.com/product/d-lock-c20-c19-power-cord/) and [Schurter 4798.9000 inlet](https://www.schurter.com/en/datasheet/4798). SSEMCO specifies standard C20 compatibility for the rotary C19 end. The datasheet provides a front flange and cutout envelope for the inlet. NVIDIA provides the service context and access arrangement. The task uses the commercial pair and makes no claim about proprietary DGX retention geometry or forces. The illustrated hardware has not been independently fit-checked here. [Panel source and fabrication](fabrication/README.md)

On the disconnected bench, mark the fully seated connector and inlet housing with adjacent tape witness lines. Establish the seated mark with the rotary switch open and the connector fully home. Orient the panel vertically and keep both neighboring cords installed. Once the rotary switch shows locked, apply a smooth 2 N axial pull at the backshell with a spring scale for one second. Release and confirm the ring is still locked and the witness lines remained aligned. The 2 N proof pull is the benchmark acceptance criterion, not a supplier rating. The simulation uses an authored socket stop, a 36° detent design input and nominal seat retention force, none of which are measurements of the commercial pair. No third-party CAD or photographs are bundled.


## Revised simulation evidence

The scripted rollout scored **1.00**. The gripper contacted the off-axis pad, rotated the balanced collar to its mechanical stop, then disengaged. The final collar angle was 36.17° and the plug reference was −0.35 mm from the authored face datum. The partial-seat control scored **0.00**, seated-unlocked scored **0.60**, and backout after indicated lock scored **0.00**. Additional scorer regressions cover locked entry, incomplete rotation, insufficient hands-clear time, pose drift, and the neighboring-plug keep-out envelope. These are simulation results.
