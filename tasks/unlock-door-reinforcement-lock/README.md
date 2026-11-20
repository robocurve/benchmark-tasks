# Unlock a door reinforcement lock: slide the sprung block off its stop, then swing it 90 degrees clear

Millions of apartments, hotels, and homes mount a satin-nickel door reinforcement lock (Prime-Line U 10827) above the deadbolt on inward-swinging entry doors for kick-in and child security. When locked, the solid extruded aluminum stop block overlaps the closed door face with only a 3 mm door gap and its catch lug hooks behind an anti-swing stop lip, so pulling straight back toward the room or attempting to swing it open immediately jams at about 11 degrees. To unlock it, a hand must pinch the block's recessed top and bottom finger grooves, slide the spring-loaded block laterally away from the jamb along the door face against its return spring to clear the stop, sustain that retraction along the rotating block axis while sweeping a 90-degree circular arc around the vertical hinge pin clear of the door edge, let the return spring seat the block into the open rest pocket against the jamb, and release clear. Unlike single-axis slide bolts, child-proof drawer catches, or simple hinged gates, this tests simultaneous spring retraction and 90-degree hinged arc tracking on a single rigid body under mechanical interlock.

## Parts (full scale, commercial product)

Positions are in cm from the midpoint between the arm bases (x forward, y left, heights above the oak table top at z = 74.75 cm).

| Part | Commercial product, real size and material | Position |
|---|---|---|
| Door reinforcement lock | **Prime-Line U 10827 Defender Security Door Reinforcement Lock** (Satin Nickel anodized extruded aluminum, ~170 g packaged, ~$16 to $22 USD at Home Depot #202258432, Lowe's, Walmart, and Amazon; identical geometry in U 10826 brass, U 10828 bronze, U 10829 white). Manufacturer-published dimensions: 3.0 in (76.2 mm) stop height, 1-3/8 in (34.9 mm; modelled 35.0 mm) stop width, 1.0 in (25.4 mm; modelled 25.0 mm) projection past door face, 2-3/16 in (55.6 mm) mounting plate depth, 800 lb static force rating. Modelled mechanism estimates (verify with calipers and luggage/fish scale on the procured unit before live trials): 15.0 mm slide travel, 400 N/m return spring (6.0 N at 15 mm full travel, 5.7 N at 14.2 mm working pull), 66.2 mm recessed finger-groove span (5.0 mm deep top/bottom grooves, open across the front 16 mm with 3.0 mm rear y-end shoulders leaving a 29.0 mm pocket around the 15.5 mm pad tips), 120 g hollowed aluminum moving block (~50 g fixed jamb plate and pin) | Mounted across the 3 mm reveal seam at z = 12.5 cm; vertical hinge pin at x 36.3, y +1.4, z 12.5 cm; locked stop block spans x 34.7..37.2, y +4.25..+7.75 cm (3 mm clear of closed door face) |
| Closed door slab | Benchtop offcut of standard 1-3/4 in (4.45 cm) solid-core exterior door slab, painted matte slate blue, 4.45 x 16.0 x 22.0 cm, fixed upright (look-only raised panel moulding and warm brass deadbolt rose + thumb-turn at y +11.0, z 4.8 cm) | Door interior face at x 37.5 cm; spans x 37.5..41.95, y +3.0..+19.0, z 0..22.0 cm |
| Door jamb and casing | Painted warm-white solid lumber jamb post (standard 2x4 stock trimmed to 6.0 x 10.7 x 22.0 cm), 0.3 cm dark reveal seam (y +2.7..+3.0 cm), proud 0.6 x 7.4 x 22.0 cm white colonial casing (y -8.0..-0.6 cm, 4 mm clear of the open stop block), and 8.0 x 27.0 x 1.6 cm rear clamping foot | Jamb interior face at x 37.5 cm; spans x 37.5..43.5, y -8.0..+2.7, z 0..22.0 cm; casing x 36.9..37.5 cm |

### Mechanism details (simulation)

| Body / joint | Type and parameters | Clearances / travel |
|---|---|---|
| `door_lock` base | Fixed rigid fixture (`mass=None`, friction 0.60) carrying door slab, jamb, casing, rear foot, 55.6 x 25.0 x 76.2 mm satin-nickel jamb flange, 14 x 13 x 40 mm dark-steel bridge, and externalized dark-steel anti-swing retainer arc (`tube`, r_in 6.0 mm, r_out 25.5 mm, h 40 mm, 24 sides, 111..159 deg) | Retainer arc stands 4.0 mm below/above yoke arms, 3.0 mm inside stop block at s = 0 |
| `hinge` | Vertical z-axis revolute joint at local (-12, -36, 125) mm, `parent=None`, stops 0.0..90.0 deg (`0..pi/2 rad`), Coulomb hold 0.03 N m (vertical axis, zero gravity load, no creep), damping 0.005, mass 0.020 kg, look-only 10 mm dia x 74 mm steel pin | Free to swing 0..90 deg only when `slide_block` clears the retainer arc; jams at 11.2 deg at s = 0 |
| `slide_block` | Local y-axis prismatic slide, `parent="hinge"`, stops 0.0..15.0 mm, return spring 400.0 N/m (6.0 N at 15 mm), damping 8.0 N s/m, hold 0.0, mass 0.120 kg, friction 0.95. Carries top/bottom yoke arms (18 x 33 x 12 mm at z +/-30 mm), steel catch lug (5.4 x 6.0 x 48 mm at local y +18.5 mm), beveled extruded aluminum stop block core (25 x 35 x 66.2 mm), and four 3.0 x 5.0 mm rear y-end shoulders | Rest (0 deg, 0 mm) and open pocket (90 deg, 0 mm): >= 3.0 mm min self-gap. At s = 14.0 mm across 0..90 deg: >= 3.0 mm self-gap and >= 4.0 mm radial clearance over retainer arc. Releasing mid-arc (15..75 deg) rests catch lug cleanly on retainer arc outer face at s = 9.75..10.0 mm |

## What the scripted demo does

The scripted demo reads the live `hinge` and `slide_block` joint angles of `door_lock` from the simulator, alongside its joint encoders and gripper opening, to sustain 13 to 14.5 mm spring retraction around the 90-degree arc; it never teleports objects or writes joint states.

1. **Pre-grasp and groove pinch (right arm only; left arm holds READY open):** opens the right gripper to full `FINGER_OPEN` (93.9 mm, 8.85 mm per side over the 76.2 mm block), pitches the approach horizontal (`pitch = +90 deg`) and rolls `roll = -90 deg` so the pads oppose vertically along world z while the wrist camera stays on the `-x` robot side. Approaches pre-grasp at `(30.6, +6.0, +12.5)` cm at <= 0.085 m/s, trims joint-PD gravity sag against measured encoders, advances straight along `+x` to TCP `(35.8, +6.0, +12.5)` cm (pad tips 4 mm short of the door face), trims encoders again on the block, closes to `FINGER_CLOSED`, and verifies the measured pad gap sits inside the 66.2 mm recessed finger grooves (61 to 71 mm, retrying once if needed).
2. **Lateral spring retraction:** slides `+14.2 mm` along `+y` against the 400 N/m return spring (5.7 N) using closed-loop feedback on live `slide_block` travel until `s >= 13.5 mm` (catch lug >= 3.5 mm clear outside the 25.5 mm retainer arc).
3. **Coupled 90-degree outward swing:** sweeps the TCP along the circular arc around the vertical hinge pin `(36.3, +1.4)` cm from 0 to 90 deg over 9.0 s while synchronously yawing the gripper from 0 to +90 deg CCW (pads remaining vertical along z), servoing on live `(hinge, slide_block)` so retraction stays near 14.2 mm without snagging.
4. **Seat into open pocket, release, and park clear:** once `hinge >= 89.0 deg`, holds yaw +90 deg and walks TCP back along `+x` (local `-y`) so the return spring seats `slide_block` into the open rest pocket (`s <= 1.0 mm`), opens smoothly to `FINGER_OPEN`, backs 60 mm along `-y`, lifts clear, parks at READY (`arm.home(duration=4.5)`), and holds still for 10 s so the final frames show both arms parked clear and the lock seated open against the jamb. All tool-point speeds remain under 0.12 m/s.

## Fidelity

Chained rigid `hinge -> slide_block` joints (400 N/m return spring, 6.0 N at 15 mm; 0.03 N m Coulomb hold on the vertical hinge, zero gravity load so no creep) and the externalized dark-steel retainer arc (`a0 = 111 deg`, `a1 = 159 deg`, `r_out = 25.5 mm`) model the Prime-Line U 10827's internal spring-loaded anti-swing stop lip and open rest pocket in an externally visible, natively colliding 2-link geometry (in `mechkit`, a chained child collides natively with BASE while passing through its parent pin). Three-millimetre rest clearances ease PhysX contact settling. The stationary full-thickness 1-3/4 in door-edge mock-up avoids miniaturizing a swinging door leaf; panel moulding, deadbolt thumb-turn, reveal strip, and screw heads are look-only. On a real Prime-Line U 10827, the stop block's internal cavity houses the spring; our 120 g block mass matches the real ~170 g lock minus its ~50 g jamb plate and pin. The simulator scores the final step without opening the grippers, while a real trial opens them first: `released()` requires every finger pad of both arms to be at least 30 mm clear of `door_lock`'s bounding box, combined with 1.0 s continuous joint stillness, so a block held open by the robot scores 0.0.

## Files

- `unlock_door_reinforcement_lock.py`: the `yam_arena` task file (scene, chained `DOOR_LOCK` mechanism, `score()`, `success()`, `demo_script()`).
- `mechkit.py`: procedural geometry, chained joints, release check, and hooks.
- `closedloop.py`: closed-loop 6-DoF scripted demo controller.
- `belimits.py`: shared geometry helpers.
- `fabricate.py`: optional standard-library ASCII STL generator for bench setup and top-camera protractor scoring aids (writes nothing on import).
- `task.json`: task metadata.

## Run a trial

From this folder on a machine configured with `yam_arena`:

```bash
INSTR="Unlock the door reinforcement lock: slide the block off its stop, swing it fully clear of the door, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=unlock_door_reinforcement_lock.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir yam-eval/door-lock-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=unlock_door_reinforcement_lock.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir yam-eval/door-lock-demo
```

- **Seed and layout:** `-E object_jitter=0` is required so the fixed door-and-jamb fixture sits at the exact calibrated station pose (`x = 37.5 cm, y = +5.0 cm`). `-E control_hz=20` matches the 20 Hz control rate.
- **Runtime per trial:** the scripted demo finishes its unlock, release, and park at READY in about 39.7 s of simulated time (~794 steps) and holds READY for 10 s (994 steps / 49.7 s total in dry_run), so `SUCCESS_HOLD_S = 8.0` ends the episode at about 41 s (~820 steps) with both arms parked well clear. Dream-1 trials run up to 2,400 steps (120 s simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 140 s for the demo and about 360 s for a full Dream-1 trial.
- **Score:** `SCORE_OVER = "end"`. `task_score` reports 1.0 when `success()` holds for 8.0 s, otherwise `score()` at the end of the run.

## Scoring

Checked every step (in real trials: overhead photo with a protractor and mm rule mounted on the jamb plate, plus 20 fps video; open both grippers before measuring). Let `theta` be the hinge angle (0 deg locked across the door face, 90 deg swung fully open against the jamb) and `s` be lateral slide retraction (0 mm seated, 15 mm full travel):

1. **Forced-interlock zero condition (latched whole-run 0.0):**
   - In simulation: `catch_lug` penetrates `retainer_arc` by more than 2.0 mm (`gap < -2.0 mm`), or `theta` enters 15.0 to 75.0 deg while `s < 9.0 mm`.
   - In real trials: the lock block, hinge, or jamb stop is visibly bent, forced past its stop, or damaged.
2. **Milestone C (latched over the run, required for all nonzero tiers):**
   - In simulation: `theta >= 20.0 deg` reached at any step with no forced-interlock fault (physically impossible without retracting the lug past the retainer arc).
   - In real trials: the stop block visually clears the 20 deg protractor mark undamaged (physically impossible on a Prime-Line U 10827 without retracting past its internal anti-swing stop).
3. **End gate (required for all nonzero tiers):**
   - Both grippers released (`mechkit.released(door_lock, clearance=0.030)`, every finger pad >= 30 mm clear of `door_lock`'s bounding box) AND the lock still (`|dtheta/dt| < 2.0 deg/s` and `|ds/dt| < 3.0 mm/s`) continuously for the final 1.0 s.
4. **Tiers (when no forced fault, Milestone C latched, and end gate holds):**
   - **1.00:** `theta >= 84.0 deg` (clear of the closed door opening sweep) AND `s <= 3.0 mm` (return spring seated in the open rest pocket).
   - **0.50:** `theta >= 78.0 deg` and not 1.00 (e.g., opened to 78..84 deg or left unseated with `s > 3.0 mm`).
   - **0.25:** Milestone C latched and neither above (e.g., cleanly retracted past 20 deg and released mid-arc so the lug rests against the outside of the stop or returns closed).
   - **0.00:** otherwise (do nothing, jam on the stop at <= 12 deg without sliding, retract without swinging past 20 deg, forced interlock, still held, or still moving).

Possible scores: 0.0, 0.25, 0.5, 1.0.

## Real-world build and reset

- **Parts list (all off-the-shelf; zero custom 3D-printed mechanism parts):**
  1. One **Prime-Line U 10827 Defender Security Door Reinforcement Lock**, Satin Nickel anodized solid extruded aluminum (~US$16 to $22 at Home Depot #202258432, Lowe's, Walmart, or Amazon; same geometry in U 10826 / U 10828 / U 10829). Includes mounting screws.
  2. Standard lumber benchtop door-and-jamb L-mock-up: one 1-3/4 in x 6-1/4 in x 8-5/8 in (4.45 x 16.0 x 22.0 cm) exterior door slab offcut painted matte slate blue; one 2x4 jamb block trimmed to 6.0 x 10.7 x 22.0 cm painted warm white; one 0.6 x 7.4 x 22.0 cm white wood casing strip; one 8.0 x 27.0 x 1.6 cm wood/steel base plate; four wood screws and C-clamps or bolts to secure rigidly to the station workbench. Optional decorative brass deadbolt thumb-turn mounted below the lock at z = 4.8 cm for visual parity.
  3. Scoring tools: small plastic protractor (or printed `jamb_protractor_plate.stl`) taped flat atop the jamb flange centered on the hinge pin, adhesive mm scale bar along the jamb face to verify `s <= 3.0 mm` open-pocket seating, and overhead 20 fps camera.
- **Optional 3D-printed setup and scoring aids:** run `python3 fabricate.py` to generate watertight ASCII STL files in `./stl/`: `jamb_protractor_plate.stl` (notched 0/20/78/84/90 deg top protractor), `reveal_spacer_jig.stl` (3.0 mm reveal gap + 125 mm height jig), and `block_clearance_gauge.stl` (76.2 x 35.0 x 25.0 mm envelope gauge with 66.2 mm groove span).
- **Assembly and pre-trial hardware verification:** screw the blue door offcut and white jamb post upright onto the rear base plate separated by a 3.0 mm reveal gap (using `reveal_spacer_jig.stl`), attach the white casing flush on the jamb face 3.3 cm left of the reveal (`y <= -0.6 cm`), and mount the store-bought Prime-Line U 10827 with its supplied screws centered at z = 12.5 cm so the closed block sits with ~3 mm door clearance. Clamp the mock-up rigidly to the workbench with the door face at x = 37.5 cm and the reveal-jamb junction at y = +5.0 cm. Before live trials, measure the procured U 10827's actual slide travel, groove-to-groove span (calipers), and spring pull force (luggage/fish scale) to record hardware baseline values and confirm the pull force stays well under the YAM's ~40 N per finger limit (expected ~5 to 12 N).
- **Forces:** sliding the U 10827 block laterally against its internal return spring requires roughly 5 to 12 N (modelled 5.7 to 6.0 N at 400 N/m), and gripping the grooved aluminum block requires ~15 to 25 N normal pinch, well within the YAM gripper's ~40 N per finger capacity.
- **Reset and cleanliness (<5 s, nothing consumed):** swing the aluminum stop block 90 deg back across the closed door face until the internal return spring snaps it behind the locked stop. Verify the clamped jamb mock-up has not shifted from its taped table mark. All surfaces are dry painted wood and anodized aluminum, requiring no consumables or cleanup beyond wiping dust from the rubber gripper pads.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
