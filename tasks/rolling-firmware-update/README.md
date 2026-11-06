# Task 2: Rolling Firmware Update

**Title:** Roll a firmware update across two redundant units without dropping the load  
**Category:** Impact on AI timelines  

---

## 1. Context and Importance

Modern AI data centres, accelerator clusters, and autonomous robot fleets rely on redundant high-availability units (e.g. primary and secondary power supplies, network controllers, redundant compute modules, and dual-redundant motor controllers). To update firmware without causing system outages, technicians and service robots follow a strict rolling update protocol:
1. The standby unit is updated first while the active unit carries the full operational load.
2. Firmware update requires opening a physical service interlock door (spring slider) and holding it open while triggering the recessed update button.
3. If the active unit is taken offline for update, or if the load is switched to a unit that is offline, an immediate system outage occurs.
4. Once the standby unit is updated and online, the load is switched over, and the second unit is updated following the identical interlock protocol.

Automating this procedure requires physical manipulation (holding a 2 N spring door with one arm while pressing a recessed button with the other arm), strict temporal invariants, and state-dependent sequencing.

---

## 2. Package Structure

| File | Description |
|---|---|
| `task.json` | Program metadata, setup description, scoring rubric, keywords, and run limit. |
| `task.py` | Yam-arena task environment definition (`SCENE`, `OBJECTS`, `score`, `success`, `demo`). |
| `geometry.py` | Complete physical dimensions, coordinates, pocket clearances, spring constants, and colours. |
| `logic.py` | Pure-Python state machine tracking unit versions, update timers, active unit, and OUTAGE latch. |
| `scoring.py` | Stateful scorer evaluating the 4-tier partial credit ladder and zero-gates with 1.0 s success hold. |
| `demo_plan.py` | Scripted bimanual waypoint generator for `demo` and all ablation modes with 1.8x time stretch. |
| `parts.py` | Compound rigid body and spring-loaded slider articulation helper with non-colliding display markers. |
| `check_reach.py` | Offline IK reachability solver checking joint margins and tolerances across all waypoints. |
| `check_collisions.py` | Offline signed-distance collision checker evaluating arm-arm and arm-fixture clearances. |
| `run.sh` | Bash script executing `demo`, ablation modes, `settle`, and `dream`. |
| `VM_NOTES.md` | Runbook and diagnostic guide for GPU evaluation. |
| `tests/` | Comprehensive unit tests covering geometry, logic, scoring, glue, and task.json rules. |

---

## 3. Physical Hardware & Scene Layout

The scene is fixed on the station table (`z = 0.7475` m) with `object_jitter = 0`:

### Rigid Fixture
- **Base Plate:** 220 x 320 x 15 mm, dark grey, mass 8.0 kg, friction 1.0 (free-standing, zero-gate on > 5 mm movement).
- **Unit Panels A & B:** 120 x 90 x 55 mm grey panels mounted at `y = +0.090` m (Unit A, left) and `y = -0.090` m (Unit B, right).
- **Shared Central Panel:** 180 x 60 x 40 mm dark grey panel along the table midline (`y = 0.0`).

### On Each Unit Panel (A & B):
1. **Service Door (Prismatic Spring Slider):**
   - Slides along X with 25 mm travel and a hard stop.
   - Restored by a 2.0 N spring (spring rate `k = 80.0` N/m, damping `10.0` N s/m, slider mass `25` g).
   - Prominent orange handle (24 x 20 x 20 mm) at `y = -0.025` m (A) and `y = +0.025` m (B) on native arm side.
   - Cover plate (32 x 44 x 4 mm steel) that conceals the recessed button pocket at rest and uncovers it when deflected.
   - Deflection threshold: `>= 18.0 mm` required to start or advance update (clearing robot finger pad footprint by >= 3.3 mm).
2. **Recessed UPDATE Button:**
   - 38 x 38 x 12 mm yellow cap inside a 42 x 48 mm pocket cavity (outer pocket width 54 mm).
   - Cap top rests 2 mm below panel surface (recessed under door).
   - 8 mm vertical travel to hard stop, 100 N/m spring, 4 g mass.
   - Counted press at `<= -6.4 mm`, re-armed at `>= -3.5 mm`.
3. **ACTIVE Lamp:** 40 mm diameter disc, lit blue when that unit carries the load.
4. **VERSION Lamp:** 40 mm diameter disc: grey for v0 (old firmware), amber while updating, bright white for v1 (new firmware).
5. **4-Lamp Progress Bar:** Four 28 mm diameter cyan discs along `y = +/- 0.135` m, lighting progressively at 1 s, 2 s, 3 s, and 4 s of continuous door hold.

### On Shared Central Panel:
1. **SWITCH Button:** 38 x 38 x 12 mm white cap, 8 mm travel to hard stop, 100 N/m spring, 4 g mass, located at `x = 0.220, y = 0.0`.
2. **LOAD Lamp:** 60 mm diameter disc at `x = 0.320, y = 0.0`. Lit bright green when load is served; turns bright red permanently on OUTAGE.

---

## 4. Bimanual Interlock & Construct Validity

A single arm cannot cheat this task:
- The distance between the door handle and the recessed button on each unit is `188.3 mm` at rest and `194.5 mm` during hold.
- The YAM gripper has a maximum finger opening of `94 mm`.
- A closed-finger push is required to deflect the door against its 2 N spring and hold it against the hard stop, while a simultaneous vertical press is required into the 42 mm pocket.
- Therefore, one arm must hold the door open while the other arm presses UPDATE.
- Link4-to-link4 clearances between arms are maintained `>= 95 mm` across all bimanual operations.

---

## 5. Scoring Ladder

The task evaluates a 4-tier partial credit ladder with 1.0 s (20 control steps at 20 Hz) success hold:

- **0.25 (Door deflected or button pressed):** Either service door deflected `>= 18.0 mm` or any button pressed (`<= -6.4 mm`).
- **0.50 (Either Unit updated):** Either Unit A or Unit B reaches v1 (held open continuously for 4.0 s) with no outage.
- **0.75 (Updated unit carries active load):** A unit at v1 carries the active load, with no outage.
- **1.00 (Both units updated, load served, clean finish):**
  * Both Unit A and Unit B updated to v1.
  * No outage occurred (LOAD lamp remains green).
  * Both service doors fully released (`deflection < 2.0 mm`).
  * Both grippers open.
  * All objects still (fixture linear velocity `<= 5 mm/s`, angular `<= 0.2 rad/s`).
  * Held continuously for 1.0 s (20 steps).

### Zero-Gates:
- **OUTAGE:** The active unit goes offline (updating) at any time, or SWITCH is triggered to an offline unit. Latches permanently, turning LOAD lamp red and scoring 0.0.
- **Fixture Displacement:** Base fixture moves `> 5.0 mm` or tilts `> 5.0 deg` from start pose (scores 0.0).

---

## 6. Execution Modes & Predicted Scores

| Mode | Command | Predicted | Measured (simulator) | Description |
|---|---|---|---|---|
| `demo` | `./run.sh demo` | 1.00 | **1.00** (757 steps) | Full bimanual rolling update (Update B -> Switch -> Update A -> Park). |
| `no_switch` | `./run.sh no_switch` | 0.00 | **0.00** (outage gate) | Attempts to update active Unit A without switching -> OUTAGE latched. |
| `release_early` | `./run.sh release_early` | 0.25 | **0.25** | Releases door B after 2.0 s (< 4.0 s) -> update cancels, B stays v0. |
| `door_only` | `./run.sh door_only` | 0.25 | **0.25** | Holds service door open >= 18 mm for 5 s without pressing UPDATE. |
| `settle` | `./run.sh settle` | 0.00 | **0.00** | Arms hold still at home; tests resting stability. |
| `dream` | `./run.sh dream` | 0.00 | **0.00** (2,400 steps) | Dream-1 zero-shot baseline. |

**Plan Length:** 757 steps (measured) with 1.8x stretch (~38.5 s).  
**Run Limit:** `"2,400 steps (2 minutes at 20 Hz)"`.

---

## 7. Real-World Build

A physical implementation can be constructed using:
- **Fixture:** Aluminium base plate (220 x 320 x 15 mm) with machined acetal/POM unit panels.
- **Service Doors:** Linear ball-bearing sliders with 25 mm stroke, 2.0 N stainless steel extension spring (`k = 80 N/m`), and anodized orange knurled handle.
- **Buttons:** Low-actuation tactile limit switches (0.8 N actuation force) fitted with 38 mm CNC Delrin caps and hard internal stops at 8 mm stroke.
- **Lamps:** Industrial 40 mm (blue, amber/white) and 60 mm (green/red) panel LEDs driven by an embedded microcontroller running the arbitration logic.

---

## 8. Changes made after the first simulator runs

1. Door return spring lowered to 2 N (80 N/m): the arm's lateral stiffness (about 165 N/m) could not hold a stiffer door open.
2. Door handles are pushed with a closed fist (`FINGER_CLOSED`), approached from the outer face with a 12 mm commanded overshoot.
3. First full run: unit B updated, but the SWITCH button only reached -5.2 mm (needs -6.4 mm), so the load never moved and updating A caused an outage (score 0.0). The finger pads were landing on the button's 12 mm guard ring. The ring is now 4 mm tall and the press is commanded 8 mm deeper (`DEMO_SWITCH_EXTRA_DZ`); the switch now reaches the end stop (-8.0 mm).
