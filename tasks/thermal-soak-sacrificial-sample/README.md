# Task: Soak two samples to a target core temperature, learning the lag from the first

## Overview
- **Category**: Impact on AI timelines
- **Benchmark Suite**: Robocurve Benchmark Tasks
- **Summary**: In physical electronics assembly, cure cycles, and thermal burn-in, process quality depends on internal component core temperatures. However, internal core temperatures cannot be directly observed by surface pyrometers or IR thermography and lag behind surface heating. A robotic operator must infer this thermal latency from an initial expendable part, calibrate its soak timing, and achieve the target core temperature band on subsequent production parts without inducing thermal damage or fire.

---

## File Structure

```
thermal-soak-sacrificial-sample/
├── geometry.py             # Physical dimensions, coordinates, thermal constants, fixture boxes
├── logic.py                # Pure-Python thermal state machine, button hysteresis, 9-disc bar
├── scoring.py              # Spatial predicates, scoring ladder, latched milestones, zero-gates
├── task.py                 # Arena task definition, parts, indicators, traces, demo wrapper
├── demo_plan.py            # Dual-arm waypoint sequences, 1.8x stretch, descents, holds
├── check_reach.py          # Dual-arm kinematic reachability checker across all modes
├── check_collisions.py     # Comprehensive collision and clearance analysis across all modes
├── predict_scores.py       # Pure-Python offline score predictor for all modes
├── run.sh                  # Execution script with environment variables and step budgets
├── task.json               # Specification, prompt, reset notes, run limit, scores
├── README.md               # Task documentation, calibration study, and benchmark notes
├── VM_NOTES.md             # GPU VM execution guide and command list
└── tests/
    ├── test_geometry.py    # Dimensions, clearances, spring forces, thermal constants
    ├── test_logic.py       # Hysteresis, thermal ODEs, calibration conditions (a, b, c)
    ├── test_scoring.py     # Ladder milestones, zero-gates, 200-cycle fuzzer
    ├── test_modes.py       # Mode parsing, env flags, step budgets, open-gripper finish
    ├── test_predictions.py # Verifies offline score predictions for all run.sh modes
    ├── test_task_glue.py   # Arena task loading, part properties, masses
    └── test_task_json.py   # Format validation and blind review checks
```

---

## How to Run

Execute via `run.sh` with the desired mode argument:
```bash
./run.sh demo          # Full demonstration policy (predicted score 1.00)
./run.sh naive         # Naive ablation: misjudges thermal lag (predicted score 0.75)
./run.sh overshoot     # Runaway ablation: overheats core > 90 C (predicted score 0.00)
./run.sh no_sample1    # Skips sacrificial sample (predicted score 0.75)
./run.sh settle        # Stationary baseline (predicted score 0.00)
./run.sh dream         # Unconditioned exploration baseline (predicted score 0.00)
```

Offline verification commands:
```bash
# Run unit tests
python -m unittest discover -s tests

# Check reachability and joint margins (>= 0.02 rad)
python check_reach.py

# Check obstacle clearances and arm-arm spacing (>= 1.0 mm, arm-arm >= 15 mm)
python check_collisions.py --mode demo

# Predict scores across all modes
python predict_scores.py
```

---

## Scene Layout & Geometry

All positions are defined relative to the dual-arm station frame origin (midpoint between arm base mount plates, $z=0$ at tabletop surface):
- **Hot Plate Fixture**: $160 \times 160 \times 30$ mm dark grey block ($8.0$ kg, fixed) centered at $(0.28, 0.0, 0.015)$.
  - **Heating Pad**: $100 \times 100 \times 2$ mm dark conductive pad at $(0.28, 0.0, 0.031)$.
- **Buttons**: Two elevated caps mounted on the fixture front face at $x = 0.215$:
  - Red **HEAT** Button: $y = +0.035$ (front-left from robot perspective).
  - White **OFF** Button: $y = -0.035$ (front-right from robot perspective).
  - Stroke: $8.0$ mm downward travel to the fixture surface hard stop ($100$ N/m spring, $4$ g mass, trigger $\le -6.4$ mm, re-arm $\ge -3.5$ mm).
- **Surface Temperature Bar**: 9 stacked acrylic lamp discs ($28$ mm diameter, $22.5$ mm pitch along $x$ from $0.170$ to $0.370$ at $y = -0.095$). Lamp $k$ is illuminated when surface temperature $T_s \ge 20 + 10 \cdot k$ °C. Discs above $70$ °C are red. Shows surface temperature of the sample currently on the hot plate, or while a sample rests on the cool pad, shows that sample's surface temperature.
- **Status Lamp**: $52$ mm diameter dome at $(0.40, 0.0)$. While a sample rests on the cool pad, provides a core-dependent observable: `amber` ($T_c < 60$ °C), `green` ($60 \le T_c \le 70$ °C target core band), or `red` ($T_c > 70$ °C overheated). While heating on plate without cool pad sample: `amber`. Idle: `off`. Success hold: `green`.

---

## Numerical Calibration Study

### Governing Equations
The thermal state machine is simulated by coupled first-order differential equations:
1. **Thermostat Hot Plate Temperature ($T_p$)**:
   $$\dot{T}_p = \begin{cases} +10.0 \text{ °C/s to } 110.0 \text{ °C setpoint (holds while HEAT is ON)} \\ -1.5 \text{ °C/s toward } 20.0 \text{ °C (when HEAT is OFF)} \end{cases}$$
2. **Sample Surface Temperature ($T_s$)**:
   - On heating pad ($|x - x_{\text{pad}}| \le 30$ mm, $|y - y_{\text{pad}}| \le 30$ mm, resting):
     $$\dot{T}_s = \frac{T_p - T_s}{\tau_{\text{heat}}}, \quad \tau_{\text{heat}} = 10.0 \text{ s}$$
   - Off heating pad (in air or on cool pad):
     $$\dot{T}_s = \frac{20 - T_s}{\tau_{\text{cool}}}, \quad \tau_{\text{cool}} = 50.0 \text{ s}$$
3. **Sample Core Temperature ($T_c$)**:
   $$\dot{T}_c = \frac{T_s - T_c}{\tau_{\text{core}}}, \quad \tau_{\text{core}} = 18.0 \text{ s}$$

### Study Results & Dwell Robustness ($\pm 1.5$ s Window)
With thermostat setpoint $110.0$ °C and nominal dwell $14.0$ s:
- **Dwell $12.5$ s ($-1.5$ s)**: Peak core reaches $T_c = 61.32$ °C ($> 60.0$ °C lower bound).
- **Dwell $14.0$ s (nominal)**: Peak core reaches $T_c = 64.35$ °C (centered in target band).
- **Dwell $15.5$ s ($+1.5$ s)**: Peak core reaches $T_c = 67.14$ °C ($< 70.0$ °C upper bound).
Across the entire $\pm 1.5$ s timing variation, the peak core stays safely within the $60.0$–$70.0$ °C target band with $> 1.3$ °C margin.
- **Observable Verification on Cool Pad**:
  When Sample 1 (sacrificial) is placed on the cool pad, its core warms through thermal lag, turning the Status Lamp `green` as it enters $60$–$70$ °C, while the temperature bar reads its surface cooling curve. This gives the agent direct, unambiguous confirmation of the required dwell timing.

---

## Scoring Ladder

| Score | Milestone Condition |
| :---: | :------------------ |
| **0.25** | HEAT button pressed or any sample moved $> 10$ mm from staging. |
| **0.50** | Sample 1 placed on hot plate, heated, lifted, and placed on cool pad (`s1_complete` latched). |
| **0.75** | Sample 2 resting on cool pad with core temperature between $45.0$ °C and $80.0$ °C. |
| **1.00** | Sample 2 on cool pad with core between $60.0$ °C and $70.0$ °C, core settled ($|\dot{T}_c| < 0.2$ °C/s), heater OFF, both grippers fully open, `s1_complete` verified, held for 20 consecutive steps ($1.0$ s). |
| **0.00** | **Zero-Gate**: Any sample core $> 90.0$ °C at any time (FIRE), any sample dropped off table ($z < -0.05$ m), or fixture displaced $> 5.0$ mm. |

---

## Review Notes

### Why It Matters
Automating hardware manufacturing and testing lines requires handling latent physical variables that are invisible to standard RGB cameras and surface sensors. This benchmark evaluates whether autonomous systems can execute multi-stage active hypothesis testing: executing an exploratory action on an expendable sacrificial unit to measure latent physical response times, and using that inference to succeed on downstream tasks.

### Shortcuts Considered & Mitigations
1. **Skipping the First Sample**: An agent might attempt to directly heat Sample 2 without using Sample 1.
   - *Mitigation*: Full score `1.00` strictly requires `s1_complete` (Sample 1 must be placed on the plate, heated, and placed on the cool pad). Skipping Sample 1 caps score at `0.75`.
2. **Leaving the Heater ON**: An agent might drop Sample 2 on the cool pad while leaving the hot plate energized.
   - *Mitigation*: Ladder condition for `1.00` requires `heater_on == False` (OFF button confirmed pressed).
3. **Early Exit Before Thermal Equilibrium**: An agent might release Sample 2 while the core is still surging.
   - *Mitigation*: Success requires the core temperature derivative to satisfy $|\dot{T}_c| < 0.2$ °C/s held continuously for 20 steps (1.0 s).
4. **Overheating / Reckless Fast Heating**:
   - *Mitigation*: A permanent latched `FIRE` zero-gate aborts with score `0.00` if any core exceeds $90.0$ °C.

### Differences from Board Neighbours
- **Versus Closed-Loop Pneumatic Tasks**: Pneumatics provide direct, continuous gauge feedback. Thermal soak exhibits high-order latent lag where surface feedback is decoupled from the target internal state.
- **Versus Probe-Based Thermal Tasks**: Does not rely on intrusive physical thermocouples; requires indirect inference via visual surface bar feedback and timing.

---

## Measured in the Simulator

*Note: Simulator runs are performed during GPU evaluation on `yam-eval-launchable-27beea`.*

| Run Mode | Predicted Score | Measured Score | Status | Notes |
| :--- | :---: | :---: | :---: | :--- |
| `demo` | **1.00** | **1.00** | Measured | Full bimanual sequence, lag compensation, success hold |
| `naive` | **0.75** | **0.75** | Measured (core 56.1 C) | Misjudged lag; core outside 60–70 °C band |
| `overshoot`| **0.00** | **0.00** | Measured (core 103 C) | Overheats past 90 °C; FIRE zero-gate tripped |
| `no_sample1`| **0.75** | **0.75** | Measured | Sample 1 omitted; score capped at 0.75 |
| `settle` | **0.00** | **0.00** | Measured | Stationary physical baseline |
| `dream` | **0.00** | **0.00** | Measured | Zero-shot baseline exploration |

---

## Changes Made During Build

1. **Cool Pad Dimensions**:
   - Expanded from $100 \times 100$ mm to $200 \times 100$ mm centered at $(0.28, -0.20)$. Two $60$ mm sample blocks require at least $180$ mm along $x$ to maintain the mandatory $\ge 120$ mm pickup clearance between parts.
2. **Button Hard Stop Mounting**:
   - Eliminated solid sleeve collision box by elevating button caps $8.0$ mm above the fixture top. The fixture top itself serves as the physical hard stop, allowing commanded finger penetration into the cap without sleeve interference.
3. **Pick & Place Timing Alignment**:
   - Adjusted waypoint dwell times to account for the $2.88$ s physical grasp and lift sequence, ensuring Sample 2 lifts exactly when its surface temperature reaches the calibrated $85.0$ °C target.
