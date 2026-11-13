# Take an ear temperature on a manikin

**Instruction:** "Take the manikin's left-ear temperature: seat the probe snugly in the canal, press Measure, keep it
still until the beep, then lay the thermometer down."

A tympanic ear thermometer reads the infrared glow of the eardrum, so the covered probe has to sit snugly in the ear
canal, aimed along it toward the opposite temple, and stay there without moving while it measures. The Braun
ThermoScan PRO 6000 directions for use say: place the probe snugly in the ear canal and direct it toward the opposite
temple; keep it steady; press and release MEASURE; a long beep and a steady ExacTemp light end the reading; if the
probe moves, the thermometer shows POS (position error) and needs a new cover. Nursing assistants (O\*NET 31-1131.00),
medical assistants (31-9092.00) and LPNs (29-2061.00) do this at every vital-signs round.

Here one gripper holds the thermometer by its base end, seats the cone in the canal of an ear-examination manikin and
holds it still, while the other gripper presses MEASURE on the display face. The press pushes on the thermometer that
the first hand is keeping steady.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=take_ear_temperature_on_manikin.py \
    --instruction "Take the manikin's left-ear temperature: seat the probe snugly in the canal, press Measure, keep it still until the beep, then lay the thermometer down." \
    --policy yam_scripted --scorer task_score --max-steps 1800
```

- Scene: `station`. Control at 20 Hz. Run limit: 1,800 steps (90 s). The scripted demo reaches success at about
  step 570 (28.5 s). A trial takes about 10 to 13 minutes on a shared GPU.
- No object jitter. The manikin is fixed and the thermometer starts in one fixed place, so the seed changes nothing.
- `--scorer task_score` gives 1.0 on success and otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_take_ear_temperature_on_manikin.py` (generated scorer tests: start, goal, partial and
  cheat layouts, and the demo's reach).

## Scoring

The scorer reads the thermometer's pose and its MEASURE button travel. The hold and release checks read the robot's
joint state to find each gripper's centre point.

| Rule | Value |
|---|---|
| Seated | Cover tip at least 2 mm past the canal entrance plane (a snug cover reaches 3.5 mm); tip within 3 mm of the canal axis; probe axis within 20 deg of the canal axis. |
| Held | A gripper's centre point inside the thermometer's bounding box, with its pads at least 10 % closed. |
| Measurement starts | MEASURE pressed (button travel 0.7 mm or more) while seated and held. |
| Reading taken | From the button's release (travel under 0.3 mm for two control steps, 0.1 s), 3.0 s seated and held, with the cover tip moving less than 1.5 mm and the probe axis turning less than 3 deg from their pose at release. The PRO 6000's measurement time is 2-3 s. |
| POS | Unseated, let go or moved more during the measurement: no reading for the rest of the run (the real one needs a new cover). |
| Room air | MEASURE pressed while the probe is not seated: the cover is used up, no reading for the rest of the run. |
| Laid down | After the reading: lowest point within 5 mm of the table top, cover tip at least 20 mm out of the canal, both grippers' centres at least 30 mm from the thermometer's bounding box, still (under 0.01 m/s and 0.5 rad/s) for 1 s. |
| Tiers | 1.0 reading and laid down; 0.75 reading, not laid down; 0.5 a measurement started on a seated probe but no reading; 0.25 seated for 1 s at some time, no measurement started on it; else 0. |
| Sticky zero | Cover tip ever more than 12 mm inside the canal (the MW12's painful-insertion zone), or any part of the thermometer ever more than 20 mm below the table top. |
| Success | 1.0, held for 1 s (`SUCCESS_HOLD_S`). |

The following all score 0: doing nothing, laying the thermometer down without a reading, a room-air press followed by
anything, a probe resting at the canal mouth, a probe pushed into the skin beside the canal, and a dropped
thermometer.

**Measuring it in a real trial:**
1. The thermometer's own display decides the reading. A temperature counts, and so does LO: the manikin is at room
   temperature. POS or an error message does not count.
2. Run the MW12 in self-learning mode. It beeps when an instrument tip passes the bony part of the canal, which is the
   sticky depth zero.
3. Judge seating, the press (a single beep) and the hold until the long beep from video.
4. After the robot lets go, photograph the thermometer lying on the table.

## Layout and dimensions (metres, world frame; arm bases at (0, +/-0.305, 0.750); table top 0.7475)

| Object | Type | Material | Size | Mass | Pose |
|---|---|---|---|---|---|
| Ear-examination manikin (Kyoto Kagaku MW12 EAR Examination Simulator II) | fixed, exact triangle-mesh colliders | soft light skin-tone resin head on a charcoal bust | 420 x 220 x 382 mm (catalogue 42 x 22 x 39 cm) | 1.75 kg (fixed: two G-clamps on the base) | base centred at (0.2685, -0.010, 0.7475), facing +y |
| Tympanic ear thermometer (Braun ThermoScan PRO 6000, Welch Allyn/Hillrom 06000-200) | free articulation: body plus a sprung MEASURE button | white ABS; polypropylene cover | 152 x 44 x 33 mm | 0.146 kg (100 g plus two AA cells; button 2 g) | base end at (0.090, -0.200), lying on its back, axis 16.5 mm above the table, probe toward +x |

**Manikin** (head frame = world axes: x = the patient's right, y = the face, z = up).

| Part | Geometry |
|---|---|
| Head | Ellipsoid 152 mm wide, 192 mm front to back, 230 mm tall, centre (0.2685, 0.006, 1.0145); slightly narrower jaw. Moulded face as on the MW12: brow ridge, closed eyelids, nose, lips and chin. |
| Left ear canal | Entrance an ellipse 9 mm tall x 7.5 mm front to back, centred at (0.195, 0.000, 1.0025) (255 mm above the table). It sits at the bottom of a concha bowl 21 mm across and 2.5 mm deep, and runs 25 mm into the head along (0.951, 0.255, 0.174): inward, 15 deg toward the face and 10 deg up, toward the right temple. |
| Auricles | Helix rim (a 5 mm tube on a C-shaped path 34 x 62 mm, open toward the face) with its centre 16 mm behind and 6 mm above the canal; a 2 mm web between the rim and an arc 19 mm round the canal; lobule 16 x 18 mm. The rim stands 1-4 mm proud of the skull. The right ear matches, with no canal. |
| Neck | Round, 108 mm across. |
| Shoulders | Charcoal bust, 420 mm wide and 220 mm deep at the base, 105 mm high at the shoulder points, rounding up into the neck at 138 mm. |

**Thermometer** (frame T: x along the body from the base end toward the probe; y across the 44 mm width; z out of
the display face; origin on the body axis at the base end).

| Part | T extent / value |
|---|---|
| Body | Rounded rectangle 44 x 33 mm (10 mm corner radius), x 0..127 mm. It domes to 36 x 28 mm at 133 mm and to 22 x 20 mm at 137 mm. |
| Probe | Base at (137, 0, 0). Axis bent 20 deg toward the back: (cos 20, 0, -sin 20). Blue collar 16 mm across and 3 mm long, then the covered cone, 12 mm across at its base, 6 mm at its tip and 13.5 mm long. The tip is 16.5 mm from the probe base. |
| MEASURE button | 18 mm round, centre x = 118 mm on the display face, 1.5 mm proud, sitting in a green ready ring 20.5 mm across. It slides along -z with 1.0 mm of travel. Spring 300 N/m preloaded 5 mm: 1.5 N to start, 1.8 N bottomed. Damping 0.5 N s/m. |
| Display window | 28 x 26 mm, centre x = 93 mm. |
| Other keys | C/F and Mem at x = 70 mm (y -/+9 mm); timer button 14 mm across at x = 47 mm. |
| Friction | 0.6 / 0.5 (static / dynamic) for the thermometer, cover and manikin skin. |

How the dimensions were set:
- **MW12.** Size and mass are from the Kyoto Kagaku catalogue (42 x 22 x 39 cm, 1.75 kg) and manual (about
  42 x 21 x 38 cm). The manual also confirms the tilting neck, the soft auricle and the painful-insertion warning:
  a yellow lamp and intermittent beep once the instrument tip passes the bony part of the canal.
- **Ear canal.** Its height, entrance size and direction are adult anthropometric estimates (tragion about 130 mm
  below the vertex). Measure them on your unit.
- **PRO 6000.** Dimensions, mass, the 2-3 s measurement time, the button set and the measuring sequence are from the
  PRO 6000 directions for use and specifications (Hillrom DIR 80024839, section 15). The probe bend, the dome and
  the button positions are scaled from the manufacturer's product photographs; measure them on your unit.
- **Forces.** The MEASURE button force (1.5-1.8 N) is a rubber-dome estimate. Calibrate it by pressing the button
  with a 5 N push gauge until it clicks.

## Real-world parts list

| Item | Product | Notes |
|---|---|---|
| Ear-examination manikin | Kyoto Kagaku MW12 EAR Examination Simulator II (11391-000), normal exam ears L1/R1 | Clamp the base to the table with two G-clamps over a rubber mat. Power on, self-learning mode. |
| Tympanic thermometer | Braun ThermoScan PRO 6000, Welch Allyn/Hillrom 06000-200 (small cradle model), two AA alkaline cells | Lay it on its back on the table at the tape marks. |
| Probe covers | Braun ThermoScan PRO 6000 probe covers 06000-005 | One new cover per trial, fitted before the trial. |
| Table | Robot station table | Tape marks for the thermometer's base end and the manikin's base. |

Why the thermometer starts on the table, not in its charging station: the station holds the thermometer upright in a
89 mm slot, and lifting the 152 mm thermometer clear of it needs the gripper centre about 0.31 m above the table.
That is beyond the arms' reach for a straight-down grip (about 0.20-0.22 m).

## Scripted demo (`demo()`, open loop, from the start layout and time only)

**Right gripper.** It holds the thermometer by its base end throughout.
1. Come down from behind the base end, wrist tilted 30 deg toward the probe.
2. Pinch the 44 mm sides 9-22 mm from the base, 5 mm past contact per pad.
3. Lift, then turn the thermometer so the probe points along the canal axis with the display up, 30 mm out from the
   ear.
4. Advance along the axis to seat the cover. The command goes 4 mm past the snug depth, because the hand rides
   compliant. It is also raised 10 mm: the held thermometer rides 10 mm low at this tilted pose (measured on the
   GPU). The demo is open loop and uses only the start layout and time.
5. Hold still through the measurement.
6. Withdraw along the canal, lower, lay the thermometer on its back where it lay, let go and rise.

**Left gripper.** Fingers closed.
1. Wait clear of the head.
2. Move to 22 mm off the MEASURE button, approaching 20 deg off the button's axis (the reachable press at 0.27 m
   above the table).
3. Press 6 mm past the bottomed button for 0.3 s, then back off to release.

Both arms then hold still for 3.5 s.

Offline checks: IK error under 0.5 mm and 0.1 deg along the whole plan; the two gripper centres stay at least 102 mm
apart (at the press).

## Assets and licences

All geometry is generated in code from primitives and procedural meshes (`yam_parts.py`, `yam_joints.py`,
`yam_mesh.py`, part of this task's own kit). No third-party meshes, scans, textures or fonts are used.

## Sources

- Hillrom / Welch Allyn, *Braun ThermoScan PRO 6000 Ear thermometer: Directions for use* (DIR 80024839), sections 3
  (product description), 9 (temperature measurement), 12 (errors: POS) and 15 (specifications: 152 x 44 x 33 mm,
  100 g without batteries, measurement time 2-3 s).
- Hillrom / Welch Allyn, *ThermoScan PRO 6000 Charging station: Directions for use* (89.2 x 103.7 x 153.0 mm, 150 g).
- Kyoto Kagaku, *MW12 EAR Examination Simulator II* catalogue (42 x 22 x 39 cm, 1.75 kg) and instruction manual
  (painful-insertion warning, tilting neck, soft auricle).
- O\*NET OnLine 31-1131.00, 31-9092.00 and 29-2061.00 task statements and 2025 employment.
