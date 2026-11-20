# Take a guest check from a marble slide rack: lift and angle off its ball wedge, neighbour undisturbed

Restaurant cooks (O*NET 35-2014.00, 1.4M US workers), waiters and waitresses (O*NET 35-3031.00, 2.3M US workers), and dining room and cafeteria attendants (O*NET 35-9011.00, 0.5M US workers) pull paper order checks off anodized aluminum marble slide racks mounted along kitchen pass rails throughout every shift. Industry surveys (including Toast's Restaurant Success Report, placing kitchen display system adoption near 37% among surveyed operators, with many full-service restaurants still running paper kitchen printers or hybrid pass rails) indicate that well over 1.5M US food-service workers handle paper pass-rail checks regularly, comfortably exceeding the 500k worker threshold. The task tests breaking a self-energizing gravity ball-wedge clutch: internal 16 mm glass marbles rest on a 16-degree inclined ramp and pinch each heavyweight board check against the vertical rear wall, so a naive straight-down pull drags the rolling marbles deeper into the narrowing wedge and jams the check (tearing paper in real kitchens). To free the target green check without disturbing the active tan check hanging 31 mm beside it, the robot must grasp the green check's outer edge, lift straight up to roll the marbles up the ramp, pivot its lower edge forward within the 14 mm bottom slot, draw it down-and-forward clear of the rack, rotate it flat, and lay it inside a clipless black check tray.

## Parts, real dimensions, and materials

All positions are in cm from the midpoint between the arm bases (`x` forward, `y` left, `z` up from table `TOP = 74.75 cm`). Every active part is built at full real-world scale using commercial food-service products:

| Part | Commercial product / source | Real dimensions & material | Simulated dimensions & mass | Start position (cm) |
|---|---|---|---|---|
| Slide check rack | San Jamar CK6512A 12-inch Slide Check Rack (anodized aluminum channel, screw-on black ABS end caps, 16 mm / 5/8 in glass marbles) | 30.5 cm long x 5.1 cm tall x 1.9 cm deep; aluminum body, ABS caps, polished glass marbles (~5.5 g each) | 30.5 cm long x 5.1 cm tall x 2.6 cm deep (29.5 cm extrusion + two 0.5 cm end caps), 1.4 cm bottom slot, 16-deg internal ramp; fixed (`mass=None`, `friction=0.24`); four free active 16 mm aqua marbles (5.5 g, `friction=0.24`) at `y = +9, +5, -3, -7 cm` | `x 30.9..33.5`, `y -15.25..+15.25`, `z +19.5..+24.6`; active marbles `(32.5, y, +21.23)` |
| Kitchen pass shelf stand | Commercial stainless pass shelf apron (or printed/fabricated via `fabricate.py`) | Shelf 10.5 x 32 x 1.5 cm, front apron 1.0 x 30.5 x 5.1 cm, two 2.2 cm diameter rear legs (21.9 cm tall), weighted base 7 x 32 x 1.2 cm; 304 stainless steel / dark grey steel base | Same dimensions, rigid fixed part of `pass_rack` (all supports behind checks, clear of grasp area) | Apron `(34.0, 0, +22.05)`, shelf `(39.75, 0, +23.85)`, legs `(43.0, +-14.0)`, base `(42.5, 0, +0.6)` |
| Green guest check (target) | National Checking Company (NCCO) G3632 Medium 1-Part Cardboard Guest Check, green | 8.9 x 17.1 cm (3.5 x 6.75 in), ~0.25-0.30 mm heavyweight board stock (~4 g) | 0.2 x 8.9 x 17.1 cm rigid plate with look-only dark order lines, 12 g, `friction=0.36` | `(33.4, +7.0, +13.55)` (top 2.6 cm inside rack, bottom +5.0 cm) |
| Tan guest check (neighbour) | National Checking Company (NCCO) T3624-SP Medium 1-Part Guest Check, tan (or 8.9 x 17.1 cm tan board stock) | 8.9 x 17.1 cm (3.5 x 6.75 in), single-copy stock | 0.2 x 8.9 x 17.1 cm rigid plate with look-only dark order lines, 12 g, `friction=0.36` | `(33.4, -5.0, +13.55)` (3.1 cm gap right of green check) |
| Black check tray | Winco WDM018-301 Ardesia Hiroto Black Melamine Rectangular Tray (flat, clipless; fits the 17.1 cm check flat inside its rim with no clip obstruction) | 19.7 x 12.7 cm (7.75 x 5.0 in) clipless matte black melamine with low raised rim | 19.7 (x) x 12.7 (y) x 1.4 cm (8 mm floor at `TOP + 8 mm`, 6 mm raised rim to `TOP + 14 mm`, 4 mm walls -> 18.9 x 11.9 cm inner floor); fixed (`mass=None`, `friction=0.50`) | Centre `(18.0, +12.5, +0.0)`; inner floor `x 8.55..27.45`, `y +6.55..+18.45` |

## What the scripted demo does

The demo reads the live pose of the green guest check from the simulator, along with its joint encoders and gripper opening. It never teleports objects, writes joints, or edits the simulation. The right arm stays parked at READY with its gripper open throughout the entire episode.

1. **Settle and side-edge approach:** After a 1.2 s settle window, the left arm opens to 28 mm, moves outside the green check's left edge (`y = +15.5 cm`) with fingers pointing along `-y` (pitched 8 degrees below horizontal so all wrist joints keep comfortable margins, pads closing along `x` across the check faces), compensates wrist sag with `trim_to_measured`, slides straight in to `y = +9.0 cm` (24.5 mm onto the check, 75 mm clear of the tan check guard), and closes fully (`FINGER_CLOSED`).
2. **Unseat the gravity wedge:** The left arm lifts straight up `+8 mm` along `+z` at constant `x` so upward friction rolls both left glass marbles up the 16-degree ramp, unloading the wedge pinch.
3. **Pivot forward within the slot:** Keeping the check's top-back corner along `x = 33.5 cm` (easing down 7 mm to 1 mm above its start height), the left arm pivots the check's lower edge forward (`-x`) by `+19 degrees` about world `+y` so the front face clears the 14 mm bottom slot lip by 2.6 mm and the ramp-to-check opening angle widens to 35 degrees (`tan(17.5 deg) = 0.315 > mu = 0.24`).
4. **Angled withdrawal:** The left arm draws smoothly down-and-forward along `(-sin 19 deg, 0, -cos 19 deg)` by 40 mm at 0.02 m/s, freeing the green check's top below the rack mouth while both left marbles settle gently back onto the bottom stop and the tan check remains undisturbed.
5. **Rotate flat and centre over tray:** After clearing forward away from the rack, the left arm pitches 22 degrees below horizontal and rolls `-90 degrees` so the check lies flat along `x` while keeping the wrist and gripper carriage well above the 14 mm tray rim, reads the live held check centre over the tray to cancel any grasp slip, and lowers to release height (`z = TOP + 3.6 cm`, check far edge ~6 mm above the tray floor).
6. **Release and park clear:** The left arm opens symmetrically to 36 mm so the check settles flat inside the 18.9 x 11.9 cm inner floor, rises 10 cm while opening fully, parks at READY (`home()`), and holds parked clear for the 8.0 s `SUCCESS_HOLD_S` window.

## Fidelity note

Real trials use the off-the-shelf 19 mm deep San Jamar CK6512A check rack and ~0.3 mm heavyweight NCCO board guest checks (G3632 green / T3624-SP tan) with the clipless Winco WDM018-301 black melamine tray. Simulation uses 2.0 mm rigid plates (12 g) and a 26 mm channel depth (14 mm bottom slot) solely for rigid-body collision stability while preserving the self-locking straight-down jam versus upward-lift and forward-angled release physics. Four active 16 mm glass marbles (two per check, 40 mm apart) use smooth native cylinder colliders (`axis="y"`, `sides=0`) inside look-only sphere visuals and recessed invisible lateral keeper ribs (`x = 31.1..32.2 cm`, clear of the check slot) representing the continuous row of marbles. Real cardstock creases or tears if yanked straight down, whereas simulated plates jam rigidly. The tray floor is modeled at 8 mm thickness (6 mm rim, 14 mm total height) so thin rigid plates do not tunnel on drop. Because the simulator scores the final step without opening grippers, `score()` explicitly enforces finger release (`mechkit.released` >= 30 mm) and rejects airborne placements (> 10 mm above any support).

## Files

- `take_a_guest_check_from_a_marble_slide_check_rack.py`: scene, scorer (`score()`, `success()`), `ARM_KEEP_OUT`, and `demo_script(ctx)` for `yam_arena`.
- `task.json`: task metadata and scoring specification.
- `fabricate.py`: standard-library script that writes ASCII STL files (mm) for the kitchen pass shelf stand mounting the off-the-shelf CK6512A at `z = 195 mm`.
- `mechkit.py`, `belimits.py`, `closedloop.py`: procedural geometry, footprint/tilt bounds, and closed-loop 6-DoF demo helper modules.

## Run a trial

From this folder in an environment configured for `yam_arena`:

```bash
INSTR="Take the green guest check out of the check rack and lay it flat on the black tray, leaving the tan check hanging."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=take_a_guest_check_from_a_marble_slide_check_rack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/check-rack-dream

# Scripted demo (full score): left arm extracts the green check and lays it on the black tray; right arm stays at READY
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=take_a_guest_check_from_a_marble_slide_check_rack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/check-rack-demo
```

- **Seed and layout:** `-E object_jitter=0` is required so the rack, checks, marbles, and tray start at their exact marked coordinates.
- **Runtime per trial:** the scripted demo completes placement at about 26 s, parks at READY by 32 s, and reaches `task_score = 1.0` after the 8.0 s `SUCCESS_HOLD_S` hold at about 35 s (702 steps at 20 Hz; full generator length 775 steps / 38.8 s). Dream-1 runs up to 2,400 steps (120 s simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 120 s for the demo and about 360 s for a full Dream-1 trial.

## Scoring

Evaluated after the initial 1.0 s settle window using simple physical tools (a ruler, a protractor, end-of-trial overhead/side photos, and 20 Hz video):

| Term | Condition | Real-world measurement |
|---|---|---|
| `D` (tan dropped, latched) | Tan check top `< TOP + 195 mm` (below rack mouth) for `>= 0.5 s` | 20 Hz video / side view |
| `V` (tan disturbed, latched) | Without `D`, tan check shifted `>= 25 mm` from start mark or tilted `>= 15 deg` from vertical for `>= 0.5 s` | Ruler from start tape mark / protractor on video |
| `F` (fault, latched) | Green check bottom `< TOP - 20 mm` (off table) or any marble falls out of the rack (centre `< TOP + 170 mm`) for `>= 0.2 s` | Video / visual check on table |
| `E` (extracted, latched) | Green check top `<= TOP + 190 mm` (at least 5 mm below rack mouth) for `>= 0.5 s` with no `D` or `V` up to that point | Side/front video |
| `Released` (last 1.0 s) | Every finger pad point of both arms `>= 30 mm` from the green check | Video / ruler after opening grippers |
| `Still` (last 1.0 s) | Green check linear speed `< 10 mm/s` and angular speed `< 0.5 rad/s` | Last 1.0 s of video |
| `H` (home on tray) | Green check face tilt `<= 10 deg` from horizontal, bottom within 3 mm of tray floor (`TOP + 8 mm`, rejecting any check perched on the 14 mm rim), whole outline inside inner floor (`x 85.5..274.5 mm`, `y 65.5..184.5 mm`) | Top-down photo + ruler + protractor |

| Score | Condition (evaluated in order) |
|---|---|
| **0.0** | `D` (tan dropped), `F` (green off table or any marble fell out of rack), or not extracted (`not E`, e.g., jammed or idle) |
| **1.0** | `H`, `E`, no `V`/`D`/`F`, `Released`, and `Still` (sustained over the last 1.0 s) |
| **0.5** | `E`, green check lying flat on table or tray (face tilt `<= 15 deg`, bottom `<= TOP + 18 mm`, not airborne), no `V`/`D`/`F`, `Released`, and `Still` |
| **0.25** | `E` with no `V`/`D`/`F` otherwise (e.g., extracted cleanly while still held, on edge, or moving); OR `H` with `V` (tan nudged without dropping), no `D`/`F`, `Released`, and `Still` |

## Real-world build, assembly, reset, and cleanliness

- **Parts list and sources:**
  1. San Jamar CK6512A 12-inch anodized aluminum slide check rack (~$15, restaurant supply distributors such as WebstaurantStore / KaTom).
  2. National Checking Company (NCCO) G3632 green medium 1-part board guest checks (3.5 x 6.75 in) and NCCO T3624-SP tan medium guest checks (3.5 x 6.75 in, or 10 pt tan cardstock cut to 89 x 171 mm).
  3. Winco WDM018-301 black melamine rectangular tray (7.75 x 5.0 in, clipless, ~$5, commercial food-service suppliers).
  4. Rear-supported pass shelf stand mounting the CK6512A 195 mm above the tabletop: either screw the CK6512A to a standard 305 mm stainless pass-through shelf apron or generate 3D-printable STL parts (`pass_shelf_apron.stl`, two `pass_stand_leg.stl`, `pass_stand_base.stl`) via `python3 fabricate.py --out stl_out`, fasten with M4 screws, and secure the base foot at `x = 42.5 cm` with double-sided tape or a rear clamp.
- **Force budget:** Lifting and angling a single board check off glass marbles requires `< 3 N` per finger (well under the YAM's ~40 N per finger limit); pulling straight down jams the marbles safely against the rack without damaging the grippers.
- **Reset and cleanliness:** Mark the two check centre lines on the rack apron (`y = +7.0 cm` and `y = -5.0 cm`, 31 mm gap between edges) and the 26 mm insertion line on the checks. Between trials, push both checks straight up 26 mm against the back wall at their marks and confirm the marbles rest evenly against each check; replace any creased or torn paper check in seconds. All parts are dry paperboard, anodized aluminum, stainless steel, glass, and melamine, keeping the shared rig completely clean.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
