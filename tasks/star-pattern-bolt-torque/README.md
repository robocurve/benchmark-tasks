# Task 1: Tighten a cold plate's four bolts in a star pattern through three torque passes

A bimanual robot task where two I2RT YAM arms tighten a cold plate fixture's four bolt heads across three sequential passes in a cross (star) pattern without warping the plate or exceeding torque stages.

## What it tests

Cold plates, heat sinks, and gasketed interfaces on AI hardware must be tightened incrementally in a cross (star) pattern so the plate seats uniformly flat against the thermal interface material. Uneven tightening warps the cold plate, creating microscopic gaps, thermal throttling, and hardware destruction.

This task tests:
1. **Bimanual coordination under geometric reach constraints**: The plate fixture is positioned such that the left arm reaches only the two left bolts (`L_near`, `L_far`) and the right arm reaches only the two right bolts (`R_near`, `R_far`), with distance from shoulder base to opposite far bolt exceeding 0.50 m. Neither arm can complete the task alone.
2. **Strict multi-pass stateful procedure**: 12 total bolt presses organized into three passes of 4 bolts each. Every pass must execute a diagonal pair sequence ($p_2 = \text{partner}(p_1)$, $p_4 = \text{partner}(p_3)$), never pressing adjacent bolts consecutively within a pair.
3. **Dynamic level-maintenance invariant**: After every press, the difference between the most tightened bolt and least tightened bolt cannot exceed 1 ($\max(\text{stage}) - \min(\text{stage}) \le 1$). Over-tightening any single bolt immediately latches `WARP` (score 0.0).
4. **No over-torque**: Pressing any bolt already at stage 3 immediately latches `WARP`.

## Files

| File | Purpose |
|---|---|
| `task.py` | The `yam_arena` task specification file (scene, fixture object, visual markers, hooks). |
| `geometry.py` | Complete station geometry, coordinates, clearances, and dimensions in pure Python. |
| `logic.py` | Pure Python hidden-state machine: stage tracking, edge detection, star order verification, level invariant. |
| `scoring.py` | Stateful scorer class evaluating partial credit ladder, hold timing, and zero-gates. |
| `demo_plan.py` | Scripted bimanual demonstration with `DEMO_*` tuning knobs and ablation modes. |
| `parts.py` | Custom rigid bodies with prismatic sliders and visual marker teleports (`part`, `Slider`, `write_pose`). |
| `check_reach.py` | Offline inverse kinematics verification across all waypoints and yaw configurations. |
| `check_collisions.py` | Offline bounding-hull collision verification between both arms and all scene obstacles. |
| `run.sh` | Simulator runner script supporting `demo`, `settle`, `adjacent`, `one_arm`, `no_star`, and `dream`. |
| `task.json` | Benchmark submission metadata. |
| `VM_NOTES.md` | Runbook for GPU VM execution and verification. |
| `tests/` | Comprehensive unit tests for geometry, logic, scoring, modes, glue, and task JSON. |

## How to run

```bash
./run.sh demo       # Scripted demo (both arms, reaches 1.0)
./run.sh settle     # Stability check at rest (scores 0.0)
./run.sh adjacent   # Presses neighbouring corners in a row -> WARP (scores 0.0)
./run.sh one_arm    # Left arm presses only its 2 bolts twice -> WARP by LEVEL (scores 0.0)
./run.sh no_star    # Correct stage levels but bad within-pass order -> WARP (scores 0.0)
./run.sh dream      # Dream-1 baseline policy
```

## Scoring ladder

Partial credit is awarded as progress is validated; partial tiers latch permanently unless a zero-gate trips:
- **0.25**: Any button depressed $\ge 2.0$ mm or any bolt reaches stage $\ge 1$.
- **0.50**: Pass 1 complete (all four bolts at stage $\ge 1$) with no `WARP`.
- **0.75**: Pass 2 complete (all four bolts at stage $\ge 2$) with no `WARP`.
- **1.00**: All four bolts reach stage 3, status lamp turns green, no `WARP`, both grippers open, and all objects still ($\le 5$ mm/s linear, $\le 0.2$ rad/s angular) held continuously for 1.0 s (20 control steps).

**Zero-gates (latched score 0.0 permanently)**:
- `WARP` latched by:
  - LEVEL violation: $\max(\text{stage}) - \min(\text{stage}) > 1$ after any press.
  - STAR ORDER violation: Non-diagonal pair sequence or repeated bolt within a pass.
  - OVER-TORQUE: Any press on a bolt already at stage 3.
- Fixture displacement $> 5.0$ mm or fixture tilt $> 5.0^\circ$.

## Layout, Seeds, and Reset

The plate fixture is centered at $x = 0.280$ m, $y = 0.000$ m on the station table ($z = 0.7475$ m).
- Base plate: 200 mm ($x$) $\times$ 260 mm ($y$) $\times$ 15 mm ($z$), dark grey, mass 6.0 kg, friction 1.0.
- Cold plate top: 140 mm ($x$) $\times$ 200 mm ($y$) $\times$ 10 mm ($z$), aluminium.
- Bolt corner pushbuttons ($36 \times 36 \times 12$ mm white caps, 8 mm travel to hard stop, 100 N/m spring, 4 g mass):
  - `L_near`: $(0.225, +0.105)$ m
  - `L_far`: $(0.335, +0.105)$ m
  - `R_near`: $(0.225, -0.105)$ m
  - `R_far`: $(0.335, -0.105)$ m
- Stage lamps: Beside each bolt, a column of three cyan lamp discs (diameter 28 mm, height 3 mm) at $y = \pm 0.055$ m indicates stage 0–3 lit from bottom to top ($x = 0.193, 0.225, 0.257$ for near bolts; $x = 0.303, 0.335, 0.367$ for far bolts).
- Large status lamp: Diameter 50 mm, height 15 mm centered at $(0.280, 0.000)$ m: off initially, red on `WARP`, green when `TORQUED`.

Reset: Return all four button sliders to rest ($q = 0.0$); all 12 stage lamps turn off, and status lamp turns off. No parts or consumables are consumed.

## Runtime and Run Limit

- Scripted demo length: 12 presses $\times$ 2.9 s $+$ ready (0.5 s) $+$ hold (1.5 s) = 36.8 s $\times$ 1.8 time-scale stretch = 66.2 s (1,325 steps at 20 Hz).
- Run limit: `2,400 steps (2 minutes at 20 Hz)`, providing 45% spare margin over demo execution.

## Real-world build

The physical setup can be constructed from standard off-the-shelf items:
1. Base plate: $200 \times 260 \times 15$ mm mild steel or weighted aluminium plate ($\ge 5$ kg) with rubber feet.
2. Cold plate top: $140 \times 200 \times 10$ mm aluminium plate bolted to the base.
3. Four corner bolt switches: Standard tactile pushbuttons or spring-loaded plunger buttons with 36 mm square white caps, 8 mm stroke to hard end stop, light return spring ($k \approx 100$ N/m, $\approx 0.8$ N bottoming force).
4. Stage indicators: Four vertical 3-LED bargraphs or three 28 mm cyan indicator lamps per bolt.
5. Status lamp: One 50 mm bicolor (red/green) dome lamp or stack light.
6. Controller: Low-cost microcontroller (e.g. Raspberry Pi Pico or Arduino) executing the exact state machine in `logic.py`.

## Review notes

1. **Importance**: Data center AI clusters and robotics compute pods utilize liquid cold plates clamped over dense GPU/TPU packages. Torque sequence errors cause bowing, thermal interface pump-out, thermal throttling, and die cracking. Autonomous maintenance robots must understand and execute multi-pass star torque sequences. Differs from HawAII ("ground end first" 2-step clamp) and MJAK (single cam lever) by requiring a full 4-point cross pattern with a dynamic level-maintenance invariant over three sequential passes.
2. **Construct validity**:
   - Shortcut 1 (One arm tightens only its side): Tripped by both STAR ORDER (second press is adjacent, not diagonal) and LEVEL ($\max - \min = 2 > 1$). Latches `WARP` $\to$ score 0.0.
   - Shortcut 2 (Tighten in circle: L_near $\to$ L_far $\to$ R_far $\to$ R_near): Adjacent press within pair trips STAR ORDER $\to$ latches `WARP` $\to$ score 0.0.
   - Shortcut 3 (Repeated presses on one bolt to rack up counts): Pressing beyond stage 1 trips LEVEL; pressing past stage 3 trips OVER-TORQUE $\to$ latches `WARP` $\to$ score 0.0.
   - Shortcut 4 (Displace or push fixture aside): Zero-gate trips if fixture shifts $> 5.0$ mm or tilts $> 5.0^\circ \to$ score 0.0.
3. **Scoring**: Every threshold is numeric: button trigger $\le -6.4$ mm, re-arm $\ge -3.5$ mm, fixture shift $\le 5.0$ mm, still velocities $\le 5$ mm/s and $\le 0.2$ rad/s, hold duration $1.0$ s (20 steps). Fuzz testing with 100 random noise configurations confirms robust, bounded evaluation.
4. **Clarity**: Setup text specifies exact millimeter dimensions, masses, materials, colours, and 3D coordinates. Instruction is 2 sentences naming buttons by colour and position.
5. **Feasibility**: All waypoints achieve joint margin $\ge 0.317$ rad (far exceeding the 0.02 rad requirement). IK error $< 0.1$ mm. Command depth (14 mm below rest) uses hard stop bottoming, avoiding dependence on soft arm PD gains.

## Predicted vs Measured Mode Outcomes

| Mode | Predicted Score | Logic / Gate Outcome | Steps |
|---|---|---|---|
| `demo` | 1.00 | All 3 passes complete in star order, lamp green, grippers open, still | 1,325 / 1,800 |
| `settle` | 0.00 | Arms hold ready, no buttons pressed | 400 |
| `adjacent` | 0.00 | Press L_near (0.25), then L_far (adjacent) $\to$ `WARP` latches | 500 |
| `one_arm` | 0.00 | Left arm presses only its 2 bolts $\to$ `WARP` latches | 800 |
| `no_star` | 0.00 | Circular order (L_near, R_near, L_far, R_far) $\to$ `WARP` latches | 1,800 |
| `dream` | Baseline | Dream-1 policy trial | 2,400 |

## Credits

`parts.py` follows the multi-part rigid body and slider technique first developed in the repository's `tasks/bearing-press/partkit.py`.
