# DEVIATIONS — qc_met8

Engineering deviations from the draft spec (`met8.md`) made for physical determinism, MuJoCo/Isaac Sim contact stability, and open-loop bimanual kinematics:

1. **Tethered 4-link ribbon cable -> Rigid coiled connector stub at rear of drive (`+` fixed strain-relief socket on console)**
   - *Spec*: `4-link` damped ribbon cable tethering `drive_unit` to `console`.
   - *Implementation*: Coiled strain-relief connector stub at the rear of `drive_unit` plus rear cord port on `console`.
   - *Reason*: A 4-link dynamic free chain bridges two otherwise decoupled parts across `-0.20..+0.10 m`, injecting uncontrolled pendulum torque on a `0.190 kg` probe resting across a curved `Ø42 mm` crown and `1.5 mm` lateral / `2.5 deg` yaw window; in physical bench setups, detachable SJ-210 drives operate untethered or via wireless/retractable slack leads during seating and leveling.

2. **Rear leveling thumbscrew geometry & thread pitch (`Ø24 mm, 1.5 mm/rev, E0 = 3.0 mm` -> `Ø26 mm, 2.5 mm/rev, E0 = 7.5 mm`, `12 x 90 deg` strokes)**
   - *Spec*: `Ø24 mm`, `1.5 mm/rev` pitch, starting `3.0 mm` retracted (`+4.2 deg` — note `asin(3/120)` is actually `1.43 deg`! `7.5 mm` over `120 mm` wheelbase is what physically produces `+3.58 deg` > `2.8 deg` partial threshold so the initial un-leveled pose cleanly exceeds `PITCH_PARTIAL_DEG` and fails `OUT OF RANGE`).
   - *Implementation*: `Ø26.0 x 14.0 mm` knurled wheel, `2.5 mm/rev` pitch, `E0 = 0.0075 m` (`+3.58 deg` initial nose-up pitch), `3.0 turns CW` (`12` symmetric `±45 deg` = `90 deg` strokes).
   - *Reason*: Physically consistent `120 mm` wheelbase trig requires `7.5 mm` initial lift for `> 2.8 deg` tilt, and symmetric `±45 deg` strokes give clean zero-bow IK and high robustness under `±5 mm` spawn jitter.

3. **Non-rotating square rear foot pad (`18 x 18 x 3 mm`) inside `25 mm` anvil U-channel rails**
   - *Spec*: Circular leveling foot resting in an anvil guide cup while turning screw `CW`, with Left Arm stabilizing at `3 N`.
   - *Implementation*: Left Arm seats the drive, releases cleanly, and parks clear **before** Right Arm approaches the thumbscrew (eliminating bimanual arm-arm interference). `rear_turn` turns only the upper brass wheel while the lower `18 x 18 x 3 mm` elastomer foot pad is constrained by the `25 mm` (`x = ±15.5 mm`) U-channel rails on `rear_anvil`, preventing yaw twist during `CW` screwing without requiring a second arm to hold the probe down.

4. **Front V-saddle bridge clearance (`2-cheek side saddle` instead of solid flat bridge across crown)**
   - *Spec*: `90-degree` front V-saddle at `SADDLE_Y`.
   - *Implementation*: Two angled `±45 deg` V-cheeks reinforced by side wing gussets at `x = ±16 mm`, leaving `x = ±11 mm` directly above the top crown generator clear for the cylindrical detector barrel and skid-stylus contact without premature flat-plate bottoming.

5. **Minor lateral/yaw/timing tolerances (`1.5 mm`, `2.5 deg`, `1.6 s` vs `1.0 mm`, `2.0 deg`, `1.5 s / 1.8 s`)**
   - *Reason*: Harmonized cleanly across `qc_met8.py`, `task.json`, and `README.md` so open-loop placement absorbs `±5 mm / ±3 deg` spawn jitter and `±30%` surface friction (`mj_check` robustness min score `1.0`) while still failing any skewed/off-crown placement.
