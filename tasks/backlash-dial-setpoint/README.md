# Task 4: Backlash Dial Setpoint

Calibrate two backlash dials to their target setpoints and lock them.

Category: **Impact on AI timelines**  
Benchmark: **Robocurve** (I2RT YAM bimanual arm arena)

---

## 1. What it tests

Precision optomechanical stages, calibration knobs, and lead-screw tuners on AI hardware test benches and robotic sensor mounts exhibit mechanical backlash: when direction reverses, mechanical clearances (gear lash, screw play) take several millimeters of knob travel before the physical lead mechanism moves and the optical encoder or dial indicator updates.

An autonomous service robot must:
1. **Infer and respect hidden mechanical hysteresis**: realize that turning or pushing from the wrong direction introduces a 6 mm deadband and 6 mm following lag that corrupts dial calibration.
2. **Execute directional backlash compensation**: approach a target setpoint from the proper calibration direction (taking up backlash via deliberate overshoot and direction reversal).
3. **Bimanual simultaneous calibration & latching**: coordinate two arms across two separated dials, locking each dial once the displayed value is verified inside the setpoint tolerance band.

---

## 2. Layout & Dimensions

All coordinates are in meters in the station world frame ($x$ forward, $y$ left, $z$ up). Table top height is $z = 0.7475$ m (`TOP`).

```
                              [+x forward]
                                   ^
    Left Lamp Bar (x=0.380, y=+0.150)    Right Lamp Bar (x=0.380, y=-0.150)
            [10 Cyan Lamps + White Target]        [10 Cyan Lamps + White Target]
                         |                                     |
               Left Rail Stop (x=0.340)              Right Rail Stop (x=0.340)
                         |                                     |
    [Left Knob (Orange)] |                 [Right Knob (Teal)] |
    x in [0.220, 0.320]  |                 x in [0.220, 0.320] |
    y = +0.150           |                 y = -0.150          |
                         |                                     |
    Left LOCK Button     |                 Right LOCK Button   |
    (x=0.270, y=+0.075)  |                 (x=0.270, y=-0.075) |
                         |                                     |
               Left Rail Near (x=0.200)              Right Rail Near (x=0.200)

                      <-------- 300 mm -------->
                                [y=0.0]
              Left Arm Base (0, +0.305)       Right Arm Base (0, -0.305)
```

- **Base Plate**: $320 \times 440 \times 10$ mm dark grey plate ($5.0$ kg, friction $1.0$), centered at $(0.280, 0.0, 0.7525)$ m.
- **Two Parallel Rails (Tracks)**: 300 mm apart ($y = +0.150$ m left, $y = -0.150$ m right), each $140$ mm outer length ($x \in [0.200, 0.340]$ m), $36$ mm inner channel width, $20$ mm tall side walls ($6$ mm thick), $5$ mm end stop walls (sliding travel $100$ mm, inner length $130$ mm).
- **Track Floor Markings**: Alternating light grey and blue $20$ mm bands along channel floors ($0\text{--}20$, $20\text{--}40$, $40\text{--}60$, $60\text{--}80$, $80\text{--}100$ mm).
- **Knobs**: $30 \times 30 \times 14$ mm, mass $80$ g, Coulomb friction $0.5$. Left knob is orange; Right knob is teal.
  - Left start setting: $s = 30$ mm ($x = 0.250$ m). Target setpoint: $65$ mm ($x = 0.285$ m).
  - Right start setting: $s = 70$ mm ($x = 0.290$ m). Target setpoint: $40$ mm ($x = 0.260$ m).
- **LOCK Buttons**: Yellow caps $36 \times 36 \times 12$ mm, mass $20$ g, in fixed dark grey pocket rings ($44 \times 44 \times 16$ mm, $3$ mm walls, $1$ mm clearance per side). Spring rate $100$ N/m, damping $4$ N s/m, travel $8$ mm to hard stop. Located beside tracks at $(0.270, +0.075)$ m (left) and $(0.270, -0.075)$ m (right).
- **10-Lamp Bars**: Dark grey housings ($40 \times 120 \times 20$ mm) above each rail at $x = 0.380$ m ($y = +0.150$ m and $y = -0.150$ m). Each carries a 10-lamp cyan bar graph ($10$ mm per lamp, $0$ to $100$ mm), a white target marker lamp at the setpoint ($65$ mm left, $40$ mm right), and a green locked indicator dome.

---

## 3. Backlash Logic

The physical knob position along the rail is $x \in [0, 100]$ mm. The displayed position $y$ satisfies:
- Forward motion ($dx > 0$): $y = x$ (displayed position tracks true position).
- After direction reversal ($dx < 0$): the display holds constant until the knob has moved $6.0$ mm further in reverse.
- Once deadband ($6.0$ mm) is traversed: display follows with a $6.0$ mm lag ($y = x + 6.0$ mm while moving downward).
- Reversing upward again: display holds constant until knob moves $6.0$ mm upward, then resumes $y = x$.
- **Target Band**: $setpoint \pm 2.5$ mm displayed. The white marker lamp lights when displayed position is in band.
- **LOCK Mechanism**: Pressing the yellow LOCK button (cap passes $\le -6.4$ mm, $80\%$ of $8$ mm travel) latches the dial's currently displayed value. Pressing it again unlocks the dial.
- **Lock Limits**: One lock outside band is allowed per dial (can unlock with a second press); a third lock attempt on either dial latches permanent $0.0$ (zero-gate).

---

## 4. Scoring Ladder

The scorer evaluates each control step ($20$ Hz):
- **0.25**: A knob moved $\ge 5$ mm from start position.
- **0.50**: One dial locked in band ($|y - setpoint| \le 2.5$ mm).
- **0.75**: Both dials locked in band ($|y - setpoint| \le 2.5$ mm).
- **1.00**: Both dials locked in band, both grippers open (`FINGER_OPEN`), and both knobs still (speed $\le 5$ mm/s) for 20 consecutive steps ($1.0$ s).

**Zero-Gates**:
- Base plate shifted $> 5$ mm $\to 0.0$.
- Any knob derailed laterally $> 10$ mm from channel centerline or lifted $> 15$ mm above wall top $\to 0.0$.
- Third lock attempt on either dial $\to 0.0$.
- Second lock outside band on either dial $\to 0.0$.

---

## 5. Mode Predictions and Ablations

| Mode | Left Arm Actions | Right Arm Actions | Predicted Score | Result Reason |
|---|---|---|---|---|
| `demo` | Push 30 $\to$ 65 mm, LOCK | Push 70 $\to$ 28 mm (overshoot down 12 mm), Push 28 $\to$ 40 mm (take up backlash), LOCK | **1.00** | Full calibration, both locked in band, still 1.0 s |
| `naive` | Push 30 $\to$ 65 mm, LOCK | Push 70 $\to$ 40 mm (single downward push, no reversal), LOCK | **0.50** | Right dial ends at $y = 46$ mm (outside $40 \pm 2.5$ mm band), left dial locked in band |
| `no_lock` | Push 30 $\to$ 65 mm | Push 70 $\to$ 28 mm, Push 28 $\to$ 40 mm | **0.25** | Dials positioned in band, but neither arm presses LOCK |
| `settle` | Hold at home (8 s) | Hold at home (8 s) | **0.00** | Knobs unmoved at start positions |
| `dream` | Baseline policy | Baseline policy | **0.00** | Cannot infer 6 mm backlash and sequential locking |

---

## 6. Real-World Build

A physical implementation requires:
- **Base Plate**: $320 \times 440 \times 10$ mm cast aluminum or tooling plate, anodized dark grey.
- **Rails**: Twin linear guideways or precision ground U-channels with adjustable Delrin friction gibs ($\mu \approx 0.5$, normal force adjusted so sliders require $\sim 0.4$ N push force).
- **Dials / Knobs**: $30 \times 30 \times 14$ mm aluminum blocks with knurled gripping faces and low-profile slide carriages.
- **Backlash Mechanism**: Lead-screw or rack-and-pinion transmission connecting slider to optical encoder with $6.0 \pm 0.1$ mm mechanical backlash (machined tooth backlash or loose dog clutch).
- **LOCK Buttons**: Standard industrial $36$ mm square illuminated momentary pushbuttons with $8$ mm stroke, internal return spring $100$ N/m, and tactile detent at bottom of stroke.
- **Readout**: Linear 10-LED bargraph display modules with high-intensity cyan LEDs ($10$ mm pitch) and independent white setpoint marker LEDs.
- **Controller**: Embedded microcontroller reading quadrature encoder pulses through a hysteresis filter, driving the LED bargraph and locking state on pushbutton edge. Safe, low-voltage ($5$ V), low-force ($< 2$ N) operation throughout.

---

## 7. Review Notes & Construct Validity

- **Why it matters (AI Timelines)**: Automated hardware commissioning, laser optical alignment, test fixtures, and spectrometer tuning rely on precision adjustment dials with mechanical backlash. Calibration procedures require learning directional hysteresis and approaching setpoints from the engaged direction.
- **Construct validity**:
  - Guessing / single push fails (`naive` scores $0.50$): approaching from the wrong direction leaves the indicator $6$ mm off, landing outside the $\pm 2.5$ mm tolerance band.
  - Positioning without locking scores $0.25$ (`no_lock`).
  - Blind trial-and-error locking is penalized by the 3-attempt lock budget zero-gate.
- **Board differences**:
  - HawAII tasks: static clamping, no hysteresis state machine.
  - MJAK tasks: mechanical toggle clamp, no sensor feedback.
  - Robocurve Task G (`coupled-knob-peak-tuning`): 2D coupled Gaussian ridge with numerical reading budget, no direction-dependent mechanical backlash.
  - Task 4 uniquely models directional mechanical backlash hysteresis and deliberate overshoot take-up.

---

## 8. Verified Offline Checks

- Reachability IK: Max position error $0.08$ mm, max rotation error $0.01^\circ$, min joint margin $0.482$ rad ($> 24\times$ the $0.02$ rad requirement).
- Collision clearances: All unintended clearances $\ge 1.0$ mm, intended contacts within compliance stroke, arm-arm clearance $\ge 12.0$ mm.
- Demo duration: $660$ steps ($33.0$ s at $20$ Hz, $1.8\times$ time scale), well below the $2,400$ step run limit.
- Blind review: Zero occurrences of identifying team words; review criteria verified.

---

## 7. Measured in the simulator

| Mode | Predicted | Measured |
|---|---|---|
| `demo` | 1.00 | **1.00** (709 steps; left dial locked at 65.6 mm, right dial locked at 40.5 mm, both inside the 2.5 mm band) |
| `naive` | 0.50 | **0.50** |
| `no_lock` | 0.25 | **0.25** |
| `settle` | 0.00 | **0.00** |
| `dream` | 0.00 | **0.00** (2,400 steps) |

### Changes made after the first simulator runs

1. The code-driven "locked" lamp parts were cylinders only and the scene would not build; they are now thin square markers.
2. The first demo stalled the left dial at 50 mm and it slid back about 8 mm when the pad lifted (locked outside the band, score 0.5). The left dial is now pushed slower (20 mm/s) with a 6 mm commanded overshoot; the right dial uses 6.5 mm. Per-side settings are `DEMO_PUSH_SPEED_L_MM_S`, `DEMO_PUSH_OVERSHOOT_L_MM` and `DEMO_PUSH_OVERSHOOT_R_MM`.
