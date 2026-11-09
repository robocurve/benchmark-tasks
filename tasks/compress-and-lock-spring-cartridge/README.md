# Compress and Lock Spring Cartridge (`compress-and-lock-spring-cartridge`)

A spring-loaded cartridge (battery tray, magazine, tool retainer) must be compressed fully into its housing with one arm and locked with an orthogonal sliding bar using the other arm. If the plunger is released before the bar is across, the return spring throws it back.

## Instruction
"Compress the spring plunger all the way in with one hand and slide the lock bar across with the other, then let go so the cartridge stays compressed."

## Category
Impact on jobs

## Context
Automated packaging, ammunition feeding, battery module replacement, and manufacturing fixturing frequently require pre-tensioned spring cartridges to be compressed and locked. A single manipulator cannot simultaneously maintain compression against return spring forces while engaging the orthogonal retainer latch, necessitating coordinated bimanual execution.

## Physical Layout
| Object | Dimensions (mm) | Start Position (m) | Kinematics / Mass / Spring | Predicted Role |
|---|---|---|---|---|
| Base Fixture | 300 x 460 x 20 | `(0.370, +0.040, 0.7575)` | 8.0 kg, friction 1.0, fixed | Rigid support base |
| Cartridge Housing | 70 x 70 x 45 | `(0.485, +0.180, 0.7900)` | Rigid body feature on base (front face at x = 0.450 m) | Plunger guide sleeve |
| Plunger Slider | Knob: 30 x 45 x 35 | `(0.340, +0.180, 0.7900)` | Axis +X, travel 45 mm, 50 N/m, 0.05 kg | Spring-loaded plunger (orange knob) |
| Lock Bar Slider | Handle: 45 x 30 x 35 | `(0.340, -0.100, 0.7900)` | Axis +Y, travel 70 mm, friction hold, 0.05 kg | Orthogonal lock bar (blue handle) |
| Status Lamp | 25 x 25 x 4 disc | `(0.485, +0.180, 0.8145)` | Non-colliding marker atop housing | Green when locked & compressed |

## Scoring Ladder
| Score | Condition |
|---|---|
| 0.25 | Plunger compressed >= 38 mm along +X held for >= 1.0 s |
| 0.50 | Lock bar across >= 55 mm along +Y while plunger compressed (`ever_locked` latched per lesson 8) |
| 0.75 | Plunger released (no arm contact) and stays compressed >= 38 mm for >= 2.0 s |
| 1.00 | Both arms clear of fixture and status lamp green held for >= 1.0 s (20 steps at 20 Hz) |

## Zero-Gates
- **Physical Stop Box**: The lock bar cannot travel past 15 mm unless the plunger is compressed >= 38 mm (a physical stop feature on the plunger blocks the bar until the notch is aligned). Attempting to slide the bar early stops at 15 mm and fails to reach the 55 mm locking threshold.
- No penalty zero-gate is required because premature bar push is physically impossible.

## Timing Margin Table
| Requirement | Specification | Scripted Arm Achieved | Safety Margin |
|---|---|---|---|
| Plunger compress hold | >= 1.0 s hold | 1.5 s hold | +0.5 s (+50%) |
| Lock bar stroke | >= 55 mm travel | 60 mm travel | +5 mm (+9%) |
| Plunger release ease-off | Eased off over 1.5 s | 1.5 s smooth ease-off | Meets specification |
| Release retention hold | >= 2.0 s hold | 2.5 s hold | +0.5 s (+25%) |
| Arm clearance settle | >= 1.0 s hold (20 steps) | 1.5 s (30 steps) | +0.5 s (+50%) |

## Spring Force & Compliance Margins (Lesson 10)
- Spring rate: 50 N/m. At full 45 mm travel, spring force is $50 \times 0.045 = 2.25\text{ N} \le 2.5\text{ N}$.
- Arm Cartesian compliance along X is ~3.1 mm/N. At 2.25 N force, expected arm deflection is $2.25 \times 3.1 \approx 7.0\text{ mm}$.
- Command overshoot: +20 mm commanded past travel limit ensures plunger reliably reaches full stroke (> 40 mm) despite soft arm controller.
- Arm-arm clearance: Left arm at $y = +0.180\text{ m}$, Right arm at $y = -0.040\text{ m}$ (locked). Minimum lateral separation is $220\text{ mm} \gg 30\text{ mm}$ minimum.

## Measured in the Simulator
- Reach: worst position error 0.08 mm, worst rotation error 0.02 deg
- Min joint margin: 0.347 rad (limit >= 0.02 rad)
- Arm-arm clearance: 69.94 mm (limit >= 30 mm)
- Min scene clearance: 9.64 mm (limit >= 1.0 mm)
- Collisions: 0 collisions detected, all clearances satisfied
- GPU runs (20 Hz): demo 1.00, Dream-1 0.00, settle 0.00, and every ablation matched its prediction (table below)

| Mode | Predicted | Measured |
|---|---|---|
| `demo` | 1.00 | 1.00 |
| `order_bar_first_arm` | 1.00 | 1.00 |
| `one_arm` | 0.25 | 0.25 |
| `release_early` | 0.25 | 0.25 |
| `bar_first` | 0.00 | 0.00 |
| `no_release` | 0.50 | 0.50 |
| `settle` | 0.00 | 0.00 |
| `dream` | 0.00 | 0.00 |

Change made after the first GPU run: the cartridge housing was moved 70 mm back along +x (front face now at x = 0.450 m).
With it 15 mm in front of the palm, the plunger stalled at 25.6 mm of 38 mm no matter how far the arm was commanded.
The push now overshoots 20 mm past the 45 mm stroke (`DEMO_PLUNGER_OVERSHOOT`) and holds 1.5 s.

## Ablation Predictions
- `demo`: 1.00 (left arm compresses plunger, right arm slides bar across, left releases, arms clear)
- `order_bar_first_arm`: 1.00 (right arm reaches standby at bar first, left compresses plunger, right locks bar, left releases)
- `one_arm`: 0.25 (left arm compresses plunger, right arm stays idle / single arm cannot do both)
- `release_early`: 0.25 (left arm compresses plunger, releases before lock bar is across; spring returns plunger to 0)
- `bar_first`: 0.00 (right arm pushes bar before plunger compressed -> bar blocked at 15 mm by stop box)
- `no_release`: 0.50 (left arm compresses, right arm locks bar, but left never releases plunger)
- `settle`: 0.00 (robot idles at home)
- `dream`: 0.00 (baseline)

## Fidelity Notes
The plunger and lock bar are modelled as physical prismatic joints on the heavy fixture. The plunger includes an 50 N/m spring return and viscous damping. The lock bar has friction hold. The physical stop box and notch are modelled as geometric colliders with stop logic in Python. The status lamp is teleported via `write_pose`. Real-world basis: mechanical spring cartridges and battery trays with slide locks.
