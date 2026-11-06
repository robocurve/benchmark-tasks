# Permute Keyed Modules Using a Buffer Socket

**Category:** Impact on AI timelines  
**Run limit:** 2,500 steps (2.1 minutes at 20 Hz)

---

## What It Tests

In AI compute clusters and datacenter switch gear, keyed accelerator modules, transceivers, and telemetry blades must frequently be swapped or re-seated to resolve mis-racking errors. Datacenter rack enclosures provide zero free table workspace: unseated modules cannot be rested unsupported without risk of damage or connector contamination. Servicing technicians or autonomous robots must use an empty spare (buffer) socket to perform an in-place permutation sequence:
1. Pull the mis-racked module from Socket 1 and insert it into the spare buffer socket (Socket 4).
2. Pull the module from Socket 2 and insert it directly into its matching destination (Socket 1).
3. Pull the buffered module from Socket 4 and insert it into its matching destination (Socket 2).
4. Verify the cluster status lamp transitions from RED to GREEN, and press the controller box COMMIT button to finalize configuration.

This requires coordinated bimanual execution (right arm performs precision keyed unplugging, transfer, and keyed insertions along narrow 70 mm pitch channels; left arm monitors state and executes the COMMIT button press) combined with state-driven confirmation logic.

---

## Instruction

> "Move the mis-racked modules between sockets—using the empty fourth socket as a buffer—so each module's coloured band matches its socket lamp disc, then press the white COMMIT button on the left controller box."

---

## Station Layout and Hardware

The station features two I2RT YAM arms at table top height $z = 0.7475\text{ m}$:
- **Rigid Base Fixture (8.0 kg, friction 1.0):** An aluminium-and-steel bus strip rigidly joined via a tie plate to an operator controller box.
  - **Four Keyed Sockets in a Row (pitch 70 mm):**
    - Sockets 1 to 4 mouths at $x = 0.240\text{ m}$, located along $y$ at $-0.260\text{ m}$, $-0.190\text{ m}$, $-0.120\text{ m}$, and $-0.050\text{ m}$.
    - Socket cavity: $28\text{ mm}$ width $\times 16\text{ mm}$ height $\times 24\text{ mm}$ depth, with a top keyway slot ($6\text{ mm}$ width $\times 3\text{ mm}$ height) providing $1.0\text{ mm}$ lateral clearance per side.
    - Sockets 1, 2, and 3 feature stationary $20\text{ mm}$ diameter coloured lamp discs on top:
      * Socket 1: RED disc `(0.90, 0.10, 0.10)`
      * Socket 2: GREEN disc `(0.10, 0.85, 0.20)`
      * Socket 3: BLUE disc `(0.15, 0.35, 0.90)`
    - Socket 4: Empty buffer socket (no lamp disc).
    - Each socket cavity includes a spring plunger detent on its $+y$ wall ($12,000\text{ N/m}$, $1.5\text{ mm}$ travel) that latches into the module's detent groove.
- **Three Keyed Plug Modules (80 g each, friction 0.8):**
  - Dimensions: $45\text{ mm}$ width $\times 18\text{ mm}$ depth $\times 28\text{ mm}$ height main body, with a $28\times 16\times 18\text{ mm}$ steel nose piece, key rib, and retention detent groove.
  - Three distinct coloured central bands:
    * `module_green`: Green band `(0.10, 0.85, 0.20)`
    * `module_red`: Red band `(0.90, 0.10, 0.10)`
    * `module_blue`: Blue band `(0.15, 0.35, 0.90)`
  - Initial configuration (swapped mis-racking):
    * Socket 1 holds `module_green` (mismatch: requires red)
    * Socket 2 holds `module_red` (mismatch: requires green)
    * Socket 3 holds `module_blue` (matching: requires blue)
    * Socket 4 is empty (free buffer)
- **Controller Box (left arm side):**
  - Located at $x = 0.280\text{ m}$, $y = +0.150\text{ m}$, $40\text{ mm}$ tall.
  - **COMMIT Pushbutton:** $36\times 36\times 12\text{ mm}$ white button cap with $8\text{ mm}$ travel to mechanical stop, $100\text{ N/m}$ spring stiffness ($0.8\text{ N}$ full travel). Counted at $\le -6.4\text{ mm}$ displacement (80% travel) with re-arm at $\ge -3.5\text{ mm}$.
  - **STATUS Lamp:** $50\text{ mm}$ diameter lamp beside the COMMIT button:
    * **GREEN:** Rendered when all three coloured sockets (1, 2, 3) hold their matching module (insertion depth $\ge 16\text{ mm}$, lateral offset $\le 1.5\text{ mm}$, tilt $\le 10^\circ$) AND no other module is displaced $> 5\text{ mm}$ or being held.
    * **RED:** Rendered otherwise.

---

## Scoring Ladder and Zero-Gates

Evaluated at 20 Hz ($dt = 0.05\text{ s}$):

| Score | Condition |
|---|---|
| **0.00** | Initial resting state, `settle` mode, or zero-gate tripped. |
| **0.25** | Any module pulled out of its starting socket (nose insertion depth $< 16\text{ mm}$). |
| **0.50** | At least one mis-racked module placed in its matching socket (Red in Socket 1 or Green in Socket 2). |
| **0.75** | Both mis-racked modules placed in their matching sockets (two matching). |
| **1.00** | All three coloured sockets hold matching modules, COMMIT pressed while STATUS is green, both grippers open, and all bodies still ($\le 5\text{ mm/s}$ lin, $\le 0.2\text{ rad/s}$ ang) for $1.0\text{ s}$ (20 steps). |

**Zero-Gates (latches 0.00):**
- Fixture displacement $> 5\text{ mm}$ from start position.
- Fixture tilt $> 5^\circ$.
- Any module dropped $> 20\text{ cm}$ below the table top.

---

## Review Notes

### Importance and Timelines Relevance
Hardware maintenance in dense AI accelerator racks requires navigating strict physical slot constraints. Transceiver modules and blade cards cannot be placed haphazardly on rack structures or cabling. Permuting items across an occupied array using a dedicated buffer slot is a canonical planning and manipulation primitive for robot-assisted datacenter maintenance.

### Construct Validity and Shortcuts Prevented
- **Blind Commit:** Pressing the COMMIT button without completing the permutation leaves STATUS lamp RED; COMMIT only validates when STATUS is GREEN.
- **Ignoring the Buffer:** Trying to swap two keyed modules directly without using the buffer fails physically because two modules cannot share a single socket cavity. Ejecting a module onto the table scores at most 0.50 and leaves STATUS lamp RED.
- **Premature Halting:** Stopping after completing the moves without pressing COMMIT reaches partial credit 0.75, but cannot reach 1.00.

### Differences from Board Neighbours
Unlike single-peg insertion tasks (e.g. `keyed-plug-loose-socket`) or diagnostics readout swaps (`blink-code-module-swap`), this task requires multi-stage sequential permutation, state verification via indicator lights, and bimanual division of labour between module handling and controller commitment.

---

## Real-World Build

A physical reproduction of this benchmark requires:
1. **Bus Strip:** Extruded 6061 aluminium C-channel with four CNC-machined keyed socket inserts spaced at 70 mm pitch, fitted with COTS spring ball plungers (M6 thread, 1.5 mm stroke).
2. **Keyed Modules:** CNC Delrin housings with 3D printed coloured identification collars (PETG: red, green, blue) and stainless steel noses with key ribs.
3. **Controller Box:** 3D printed or sheet-metal enclosure housing an industrial 36 mm square pushbutton with light return spring ($100\text{ N/m}$, 8 mm travel) and a dual-colour high-output LED indicator dome (50 mm diffuse polycarbonate lens).

---

## Files in This Directory

- `geometry.py`: All physical dimensions, coordinates, clearances, spring rates, and colours.
- `logic.py`: Pure Python hidden-state machine managing socket occupancy, matching logic, and STATUS lamp state.
- `scoring.py`: Stateful 20 Hz scorer implementing the partial credit ladder, latching, and zero-gates.
- `demo_plan.py`: Scripted bimanual waypoint generator for demo and ablation modes.
- `task.py`: Yam-arena entry point (`SCENE`, `OBJECTS`, `success()`, `score()`, `demo()`).
- `parts.py`: Rigid bodies and kinematic markers.
- `check_reach.py`: Kinematic reachability verifier (IK along every waypoint).
- `check_collisions.py`: Sweep collision checker verifying clearances $\ge 1.0\text{ mm}$ and arm-arm clearance $\ge 12\text{ mm}$.
- `run.sh`: Command-line launcher supporting all evaluation modes.
- `predict_scores.py`: Offline score replay verifying predicted scores for all modes.
- `tests/`: Full unit test suite covering geometry, logic, scoring, modes, glue, task.json, and predictions.
- `task.json`: Task metadata and competition specification.
- `VM_NOTES.md`: GPU VM execution guide.

---

## How to Run

```bash
# Run unit tests
python3 -m unittest discover -s tests

# Check kinematic reachability
python3 check_reach.py

# Check collision clearances
python3 check_collisions.py

# Run offline score predictions
python3 predict_scores.py

# Run in simulator (on GPU VM)
./run.sh demo
./run.sh no_commit
./run.sh no_buffer
./run.sh settle
./run.sh dream
```

---

## Predicted and Measured Scores

Measured scores come from the simulator runs listed in the last column.

| Mode | Predicted Score | Predicted Success | Description |
|---|---|---|---|
| `demo` | **1.00** | **True** | Full 3-move permutation using buffer, COMMIT pressed, 1.0 s still hold. |
| `no_commit` | **0.75** | **False** | Completes all 3 moves but skips pressing COMMIT button. |
| `no_buffer` | **0.50** | **False** | Direct swap attempt without buffer; leaves Green on table, Red in Socket 1. |
| `settle` | **0.00** | **False** | Arms hold at home for 8 s; parts rest untouched. |
| `dream` | **0.00** | **False** | Dream-1 zero-shot baseline. |

---

## Measured in the simulator

| Mode | Predicted | Measured |
|---|---|---|
| `demo` | 1.00 | **1.00** (1,047 steps; red, green and blue seated in sockets 1, 2, 3 after the three moves, COMMIT counted, lamp green) |
| `no_commit` | 0.75 | **0.75** |
| `no_buffer` | 0.50 | **0.50** |
| `settle` | 0.00 | **0.00** (nothing moved) |
| `dream` | 0.00 | **0.00** (3,000 steps) |

### Changes made after the first simulator runs

1. The first full run scored 0.25: the modules came to rest about 17 mm short of seating and about 1.5 mm low, wedged on the socket mouth. The insertion height offset (`DEMO_INSERT_DZ`) is now 2.0 mm (was 0.6 mm) and the insertion overshoot (`DEMO_OVERSHOOT`) 5 mm (was 2.5 mm); with those two values all three moves seat.
2. The `no_buffer` ablation first parked the pulled module at x = 0.38 m, past the sockets, so the hand pushed the whole bus strip 30 mm and tripped the fixture-moved gate (score 0.0). The module is now parked on the robot side of the strip, 80 mm outboard of socket 1.
