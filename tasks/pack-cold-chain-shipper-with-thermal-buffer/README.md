# Pack a cold-chain pharmaceutical shipper with a thermal buffer separating the refrigerant brick

## Why this is a real job

- **Occupation**: **Hand packers and packagers** (BLS Standard Occupational Classification 53-7064, with over 650,000 workers in the United States).
- **Core activity**: Packaging temperature-sensitive biologics, vaccines, and liquid pharmaceuticals into qualified insulated thermal transport shippers at distribution hubs and specialty mail-order pharmacies.
- **What is real and what is simplified** (sizes are chosen so the YAM gripper, which opens 94 mm, can really pinch every part):
  - *Refrigerant gel brick*:
    - Real: hard-shell gel bricks of about 6 x 3 x 1 in are an everyday size; mass about 0.3 kg.
    - Simulation: 152 mm (Y) x 76 mm (X) x 25 mm (Z), mass 0.33 kg (about 295 cm3 of gel plus shell). Modelled as one rigid box; the real shell flexes and the gel sloshes.
  - *Corrugated thermal buffer barrier pad*:
    - Real: a die-cut double-wall corrugated pad, a few millimetres to about 15 mm thick, mass a few tens of grams.
    - Simulation: 150 mm (Y) x 74 mm (X) x 15 mm (Z), mass 0.04 kg, one rigid box. A real pad is crushable and would deform under the pad tips; here it is stiff.
  - *Medication carton*:
    - Real: a small carton for vials or pens is about 130 x 70 x 25 mm, mass about 0.18 kg.
    - Simulation: 130 mm (Y) x 70 mm (X) x 25 mm (Z), mass 0.18 kg, one rigid box with flat label decals on its top face.
  - *Insulated shipper*:
    - Real: small foam or rigid insulated shippers exist in about this size; a real one is usually tighter around the contents.
    - Simulation (simplified): cavity 172 mm (Y) x 150 mm (X) x 70 mm deep, 10 mm walls and floor, rigid. The cavity is deliberately wider than a snug shipper so the two gripper fingers fit beside a part when it is set down and when they leave.
  - *Shipper lid*:
    - Real: a foam or corrugated lid the size of the shipper's footprint, a few millimetres to a few centimetres thick, usually closed with tape, a snap or a latch.
    - Simulation (simplified): one rigid slab 170 mm (X) x 192 mm (Y) x 8 mm, mass 0.12 kg, resting on the 10 mm rim all round, with a 40 mm (X) x 30 mm (Y) x 22 mm grip tab on top. A real lid is thicker, compressible and often hinged or taped; there is no hinge here and no seal.
  - *Spring slide latches (the closing mechanism)*:
    - Real: spring latches or snap hooks on a case close by a hook that is pushed aside as the lid is pressed down and springs back over the lid's edge.
    - Simulation (simplified): two latches, one outside each 192 mm end of the shipper. Each is a rigid post with a 14 mm round head (a rod along X, 30 mm wide) on a prismatic joint with a spring drive: 0.5 N/mm (500 N/m), 5 N s/m damping, 8 mm travel outward, 10 g slider mass, no preload. At rest the head hangs 4 mm over the lid's end edge and 1.5 mm above a seated lid. The lid's lower end edge rides over the round head, pushing the slider outward by 4 mm, and the spring returns it over the lid. The latch holds the lid by contact alone; a real snap hook's geometry, flex and wear are not modelled. The round head replaces a sloped hook face, because the parts are built from boxes and cylinders only.
    - Friction: the shipper and the lid use friction 0.3 each, so the lid's edge slides over the round head instead of sticking to it (with this rod size the edge meets the circle at a slope of about 0.47 at the start of the push, so it would stick if the combined friction were above about 0.47).
  - *Friction*: the brick, buffer pad and carton use friction 1.0 (rubber-like pads on plastic and cardboard); the shipper and lid use 0.3 (see the latches). This is a simplification; real cardboard against the pads can be lower or higher.
  - *Grasping rules measured on the YAM arms*:
    - Fingers open 94 mm; a part must be at most 80 mm across the pinched side (at least 7 mm per side) because arms land 3 to 6 mm off the commanded point.
    - The pad tips are 13 mm below the commanded TCP point, so the commanded TCP z is the pad-tip z plus 0.013 m.
    - The finger bodies are about 12 mm thick and 24 mm wide near the pads, but the URDF hull of each finger flares to about 24 mm thickness from 33 mm above the pad tips; the collision audit uses both models.
  - *Part orientation and gripper yaw*: every part lies with its short side along world X and its long side along Y. Fingers close along world X (left arm yaw -90 deg, right arm yaw +90 deg; they pinch identically and each yaw keeps that arm's wrist joint well inside its limit). Pinched widths: brick 76 mm, buffer pad 74 mm, carton 70 mm, lid tab 40 mm, all at most 80 mm.
  - *Pad tip height rule*: at pick the pad tips ride 4.5 mm above the staging plate (pick TCP z = 0.7695 m, pad-tip z = 0.7565 m) and engage 20.5 mm of the brick side, 10.5 mm of the buffer pad side and 20.5 mm of the carton side. On release the part is 2 mm above its seat and the pad tips are 4.5 mm above the part bottom.
  - *Lid grasp*: the arm pinches only the 40 mm tab (pad gap commanded 8 mm, well under 40 mm) with the pad tips 5 mm above the lid's top face, so 17 mm of the 22 mm tab is between the pads and the finger bodies stay 5 mm above the slab (the rule is at least 4 mm). Opened to 90 mm on release, the fingers have 25 mm of daylight per side around the tab. The lid is carried with its underside 33 mm above the latch heads (commanded TCP z 0.910 m). The other parts ride at TCP z 0.905 m, 36 mm above the heads and 60 mm above the rim (the rule is 15 mm or more, because a loaded arm tracks about 10 mm below a commanded height).
  - *Cavity free space*:
    - Along X (the finger side): (150 - 76) / 2 = 37.0 mm for the brick, (150 - 74) / 2 = 38.0 mm for the buffer pad, (150 - 70) / 2 = 40.0 mm for the carton (the fingers need at least 15 mm per side).
    - Along Y (the long side): (172 - 152) / 2 = 10.0 mm for the brick, (172 - 150) / 2 = 11.0 mm for the buffer pad, (172 - 130) / 2 = 21.0 mm for the carton (at least 4 mm required).
    - Release opens the pads to a 90 mm gap: at least 7 mm of daylight per side around every part, so the fingers lift out without dragging the part.
  - *Staging Pad Layout Decision*:
    - The shipper cavity is placed at (x = 0.280 m, y = 0.000 m); its outer front face is at x = 0.195 m.
    - Lateral pads at (x = 0.220 m, y = +/- 0.220 m); front pad at (x = 0.115 m, y = 0.000 m). The open finger hulls reach about 71 mm from the pick point, so the front pad sits 80 mm from the shipper face, leaving about 9 mm of clearance to the shipper wall. Pads are 110 mm (X) x 180 mm (Y) x 4 mm.
    - Lid plate at (x = 0.300 m, y = +0.420 m) in layout A or (x = 0.300 m, y = -0.420 m) in layout B, 176 mm x 198 mm x 4 mm: 11 mm beyond the lateral pad's outer edge. The shipper's own x range is 0.195 to 0.365 m, and an offline probe shows the YAM arms cannot reach x = 0.38 m on the centreline within 3 mm (12 mm position error), so the lid cannot stand behind the shipper; beside the lateral pads the arm on that side reaches it with at least 0.158 rad of joint margin.
  - *Hold Time Decision*:
    - A stable 3.0 s dwell (60 control steps at 20 Hz) with arms withdrawn clear to home standby confirms the parts have come to rest in the shipper.
  - *Pauses*: the soft arms stall 20 to 60 mm short of a commanded point unless the plan pauses, so every pick and place pauses about 0.6 s (before the 1.8x time scale) at hover before descending. The demo plan, with the lid, takes about 1,600 steps at 20 Hz (80 s), inside the 2600 step limit.

## Real hazard to a person

Many pharmaceuticals and vaccines are freeze-sensitive and must stay between 2 and 8 degrees Celsius while they travel. The Pan American Health Organization states that "vaccines that are sensitive to freezing should be stored at temperatures between 2°C and 8°C" and tells health workers to prepare (condition) frozen ice packs before they go into a transport container (PAHO, "Cold Chain" FAQ, https://www.paho.org/en/immunization/cold-chain, read for this submission). A frozen refrigerant brick sits well below 0 degrees Celsius, so a product pressed against it can be cooled below freezing where it touches. A patient who receives a product that has frozen and lost its effect is the harm; the packer is the person who decides, by how the shipper is loaded, whether that can happen.

Hand packers load these shippers. The safeguard this task models is a buffer layer between the frozen brick and the product carton.

**What the simulator does and does not model (read this before judging the task):**
- Not modelled: temperature, heat flow, time, condensation, the shipper's insulation, or any damage to the product. The buffer pad is not claimed to be a qualified thermal solution; its job in the task is to be a physical layer that keeps the carton off the brick.
- Modelled as a rule: if the carton rests directly on the brick (centre-to-centre height difference under 32.5 mm), the contact violation latches and caps the score at 0.25. Real freezing depends on pack temperature, contact time and the product, so this rule is a deliberate simplification of "frozen pack touching the product".
- Modelled as geometry: a rigid lid and two spring slide latches (see the build notes above). The lid counts as closed only when it sits flat on the rim and both latch heads hang over its end edges, on top of a correct stack. This is a mechanical closure, not a seal: nothing here tests air tightness, insulation or hold time, so a finished layout is not claimed to be transport-qualified. A real shipment would also be closed with the shipper's own seal or tape and its temperature performance proven by a test.

## Instruction

*"Pack the medication shipper for transport."*

The instruction is strictly goal-only (6 words): it specifies the high-level objective without disclosing component order, the presence of the thermal buffer, or temperature-freeze constraints.

## Setup and Scene

- **Shipper Base**: Qualified insulated thermal shipping container standing at x = 0.280 m, y = 0.000 m, table elevation z = 0.7475 m. Cavity interior is 172 mm (Y) x 150 mm (X) x 70 mm deep, with 10 mm walls and floor.
- **Staging Pads**:
  - *Left Staging Pad*: Centered at x = 0.220 m, y = +0.220 m.
  - *Right Staging Pad*: Centered at x = 0.220 m, y = -0.220 m.
  - *Front Staging Pad*: Centered at x = 0.115 m, y = 0.000 m.
- **Components**:
  - `gel_brick`: sub-zero hard-shell refrigerant brick (152 mm x 76 mm x 25 mm, mass 0.33 kg).
  - `buffer_pad`: corrugated kraft thermal buffer pad (150 mm x 74 mm x 15 mm, mass 0.04 kg).
  - `medication_carton`: freeze-sensitive pharmaceutical drug carton (130 mm x 70 mm x 25 mm, mass 0.18 kg).
  - `shipper_lid`: rigid lid (170 mm x 192 mm x 8 mm, mass 0.12 kg, friction 0.3) with an orange grip tab (40 mm x 30 mm x 22 mm), lying on `pad_lid`.
  - Two spring slide latches (`latch_pos_y`, `latch_neg_y`) are part of `shipper_base`: orange posts outside its two 192 mm ends with a round head each hanging 4 mm over the lid's end edge.
- **Visual Variants**:
  - *Variant A*: Left pad holds `medication_carton` carrying a blue freeze-sensitive stripe with a white cross pictogram (insulin style; no printed text is modelled). Right pad holds `gel_brick`. Front pad holds `buffer_pad`. The lid lies on the left, beside the carton's pad (y = +0.420 m).
  - *Variant B*: Right pad holds `medication_carton` carrying an amber warning stripe with a dark warning icon (heparin style; no printed text is modelled). Left pad holds `gel_brick`. Front pad holds `buffer_pad`. The lid lies on the right, beside the carton's pad (y = -0.420 m).
  The two variants differ in what the robot sees: the brick, carton and lid swap sides, and the carton label colour and icon change. Each arm uses the yaw that suits its side (left arm -90 deg, right arm +90 deg); the fingers close along world X in both.
  The robot must inspect the scene visually to identify payload carton vs refrigerant brick before packing.

## Clearances and Tolerances

| Feature | Value | Rule / Standard |
|---|---|---|
| Cavity interior dimensions | 172 mm (Y) x 150 mm (X) x 70 mm | Widened along X so the fingers fit beside the part |
| Brick free space per side (X / Y) | 37.0 mm / 10.0 mm | X >= 15 mm for the fingers, Y >= 4 mm |
| Buffer free space per side (X / Y) | 38.0 mm / 11.0 mm | X >= 15 mm for the fingers, Y >= 4 mm |
| Carton free space per side (X / Y) | 40.0 mm / 21.0 mm | X >= 15 mm for the fingers, Y >= 4 mm |
| Pinched width (brick / buffer / carton / lid tab) | 76 mm / 74 mm / 70 mm / 40 mm | At most 80 mm (gripper opens 94 mm) |
| Lid underside over the carton when seated | 5.0 mm | Lid rests on the rim, never on the layers |
| Latch head over the lid's end edge, seated | 4.0 mm (1.5 mm above the lid's top face) | Latch travel 8 mm, spring 0.5 N/mm |
| Lid seating tolerance | within 2.5 mm above the rim, within 12 mm of centre, tilt < 3 deg | A head must hang at least 0.5 mm over each end edge |
| Pad tip height at pick | 4.5 mm above the staging plate (lid: 5 mm above its top face) | Pad tips are 13 mm below the TCP |
| Finger, link and carried part clearance | at least 4.0 mm | check_collisions.py |
| Horizontal seating tolerance (XY) | ± 16.0 mm | Component center within cavity |
| Vertical seating tolerance (Z) | ± 12.0 mm | Component height inside stack |
| Maximum allowable tilt | 4.0 deg | Level horizontal seating |
| Freezing contact threshold | < 32.5 mm vertical centre separation | Carton on brick is 25 mm, carton over buffer is 40 mm |
| Quiescent hold dwell | 3.0 s (60 steps at 20 Hz) | Quiescent equilibrium with arms retracted |

## Scoring Ladder and Ablation Modes

- **0.25**: Refrigerant gel brick placed flat in the shipper base cavity (z ~ 0.770 m, tilt < 4 deg).
- **0.50**: Thermal buffer pad placed inside shipper directly over gel brick (z ~ 0.790 m, tilt < 4 deg).
- **0.75**: Medication carton placed on top of buffer pad (z ~ 0.810 m, tilt < 4 deg).
- **1.00**: All three components seated flat inside chamber in correct safe order (gel brick on bottom, buffer pad in middle, medication carton on top), tilt < 4 deg, zero direct contact between medication carton and gel brick, **the lid seated flat on the shipper rim (centre within 2.5 mm of its seated height, tilt < 3 deg, within 12 mm of the shipper centre) with both latch heads hanging at least 0.5 mm over its end edges**, arms withdrawn clear to home standby, and held for at least 3.0 s.
- **The score is the state at the current step**, never a best score: undoing a step lowers it (lifting the lid off its latches drops 1.00 to 0.75; pulling the carton out drops 0.75 to 0.50).
- **The lid never adds credit by itself.** It counts only on a correct stack (all three layers placed, in order, no latched violation). A lid shut on a stack with a layer missing scores what the layers present score; a lid closed on a carton resting on the brick is the latched violation (cap 0.25).
- **Why the lid is hard**: a lid laid on the two latch heads rests up to about 23 mm above the rim (geometry); in the `lid_unlatched` simulator run it perched tilted, one end on a head and the other on the rim, with its centre about 12 mm above the seated height. The arm must press it down through both springs (each head is pushed 4 mm outward by the lid's edge) until it sits on the rim and the heads spring back over it. Laid on the heads and let go, it does not close.
- **Latched violation**: any direct contact between the medication carton and the refrigerant gel brick (bypassing or displacing the buffer pad) latches the contact violation for the rest of the run and caps the score at 0.25. This is a rule, not a heat model (see the hazard section).

### Ablation Modes

| Mode | Predicted | Measured | Steps | Description |
|---|---|---|---|---|
| `demo` | 1.00 | 1.00 (layout B also 1.00, 1,626 steps) | 1,626 | Full packing demonstration: gel brick bottom, buffer pad middle, carton top, then the lid carried over, pressed down through both latches until it sits on the rim, arms home, 3.0 s hold |
| `no_buffer_violation` | 0.25 | 0.25 | 932 | Medication carton placed directly onto the gel brick, omitting the buffer pad (latches the contact violation) |
| `brick_only` | 0.25 | 0.25 | 604 | Gel brick placed flat in shipper; buffer pad and carton untouched |
| `buffer_without_carton` | 0.50 | 0.50 | 706 | Gel brick and buffer pad placed in shipper; medication carton untouched |
| `lid_unlatched` | 0.75 | 0.75 | 1,641 | All three layers packed, then the lid is laid on the latch heads and released without being pressed down; the latches never snap over its edge |
| `no_lid` | 0.75 | 0.75 | 1,259 | All three layers packed, the lid left on its plate |
| `settle` | 0.00 | 0.00 | 400 | Robot arms remain idle at home standby; all components untouched |
| `dream` | 0.00 | 0.00 | 2,600 | Dream-1 baseline policy with the lid and latches; hit the 2,600-step limit with no part moved |

All measured values are layout A, one run each, except the second demo entry (layout B, one run). The `demo`, `lid_unlatched` and `no_lid` rows were measured with the lid and latches in this folder; so were the other ablations and the Dream-1 run. In `lid_unlatched` the lid ended perched and tilted on one latch head, its centre about 12 mm above the seated height, with neither latch engaged.

## Rule 9: One Source of Truth Cross-Module Verification

| Constant | `geometry.py` | `task.json` | `README.md` | Verification Status |
|---|---|---|---|---|
| `TABLE_Z` | 0.7475 m | 0.7475 m | 0.7475 m | EXACT MATCH |
| Cavity Length Y | 0.172 m (172 mm) | 172.0 mm | 172 mm | EXACT MATCH |
| Cavity Width X | 0.150 m (150 mm) | 150.0 mm | 150 mm | EXACT MATCH |
| Cavity Depth Z | 0.070 m (70 mm) | 70.0 mm | 70 mm | EXACT MATCH |
| Brick Dimensions (Y x X x Z) | 152 x 76 x 25 mm | 152 x 76 x 25 mm | 152 x 76 x 25 mm | EXACT MATCH |
| Brick Mass | 0.33 kg | 0.33 kg | 0.33 kg | EXACT MATCH |
| Buffer Dimensions (Y x X x Z) | 150 x 74 x 15 mm | 150 x 74 x 15 mm | 150 x 74 x 15 mm | EXACT MATCH |
| Buffer Mass | 0.04 kg | 0.04 kg | 0.04 kg | EXACT MATCH |
| Carton Dimensions (Y x X x Z) | 130 x 70 x 25 mm | 130 x 70 x 25 mm | 130 x 70 x 25 mm | EXACT MATCH |
| Carton Mass | 0.18 kg | 0.18 kg | 0.18 kg | EXACT MATCH |
| Brick free space per side | 37.0 mm (X) / 10.0 mm (Y) | 37.0 mm / 10.0 mm | 37.0 mm / 10.0 mm | EXACT MATCH |
| Buffer free space per side | 38.0 mm (X) / 11.0 mm (Y) | 38.0 mm / 11.0 mm | 38.0 mm / 11.0 mm | EXACT MATCH |
| Carton free space per side | 40.0 mm (X) / 21.0 mm (Y) | 40.0 mm / 21.0 mm | 40.0 mm / 21.0 mm | EXACT MATCH |
| Minimum Fit Clearance (long axis) | 10.0 mm | 10.0 mm | 10.0 mm | EXACT MATCH |
| Pinch limit | 80 mm | 80.0 mm | 80 mm | EXACT MATCH |
| Shipper Location | (0.280, 0.000) | (0.280, 0.000) | (0.280, 0.000) | EXACT MATCH |
| Hold Dwell Time | 3.0 s | 3.0 s | 3.0 s | EXACT MATCH |
| Lid Dimensions (X x Y x Z) | 170 x 192 x 8 mm | 170 x 192 x 8 mm | 170 x 192 x 8 mm | EXACT MATCH |
| Lid Mass | 0.12 kg | 0.12 kg | 0.12 kg | EXACT MATCH |
| Lid grip tab (X x Y x Z) | 40 x 30 x 22 mm | 40 x 30 x 22 mm | 40 x 30 x 22 mm | EXACT MATCH |
| Latch spring / travel / head overlap | 500 N/m, 8 mm, 4 mm | 0.5 N/mm, 8 mm, 4 mm | 0.5 N/mm, 8 mm, 4 mm | EXACT MATCH |
| Lid plate centre | (0.300, +/-0.420) | (0.300, +/-0.420) | (0.300, +/-0.420) | EXACT MATCH |

## Board Check (Titles read directly with `gh pr view`)

Exact output from `gh pr view <n> --repo robocurve/benchmark-tasks --json number,title,state -q '"PR #" + (.number|tostring) + ": " + .title + " | " + .state'`:

1. `PR #1060: Add reclose-vaccine-cooler task | CLOSED`
   - *Physical difference*: PR #1060 is an insulated cooler lid reclosing task (pushing a hinged lid down to latch). Our task shares only the last step, closing a lid against spring latches (a loose rigid lid pressed onto two spring slide latches, no hinge). Before that, the robot must pack three layers in a safe order into the shipper (gel brick, thermal buffer pad, medication carton), and the lid counts only if that stack is correct and the carton never touched the brick.
2. `PR #631: Store vaccine cartons in the middle of the fridge, away from the door and the walls | CLOSED`
   - *Physical difference*: PR #631 places vaccine boxes inside a domestic refrigerator cavity based on spatial clearance margins from exterior refrigerator walls; zero refrigerant gel bricks, zero thermal buffer barrier pads, zero cold-chain transport shipper packing.
3. `PR #495: Add thermal-soak-sacrificial-sample task | CLOSED`
   - *Physical difference*: PR #495 handles metallurgical heat-treatment soak samples in a high-temperature furnace; completely different thermal domain (high heat metallurgical furnace vs sub-zero pharmaceutical cold chain), zero pharmaceutical packaging materials.

## Verification Commands

```bash
# Run unit tests
PYTHONPATH="work/refs/yam-isaac-lab-arena:work/tasks/pack-cold-chain-shipper-with-thermal-buffer" work/venv/bin/python -m unittest discover -s work/tasks/pack-cold-chain-shipper-with-thermal-buffer/tests -v

# Kinematic reachability audit
python3 check_reach.py --variant A
python3 check_reach.py --variant B

# Collision and clearance audit
python3 check_collisions.py --variant A
python3 check_collisions.py --variant B

# One simulator trial (the modes are demo, no_buffer_violation, brick_only, buffer_without_carton,
# lid_unlatched, no_lid, settle and dream); set TASK_VARIANT=B for the other layout
./run.sh demo
```

`check_collisions.py` builds every obstacle in world coordinates (table top z = 0.7475 m): shipper floor and walls, the two latch posts and heads, four pads and the four parts (the three layers and the lid with its tab: staged, carried, then seated). It checks the URDF gripper hulls and a simple finger-body model (two 12 mm x 24 mm boxes, 40 mm tall from the pad tips, at plus and minus half the pad gap) against everything that is not being carried, and the carried part against the shipper walls and the latch heads (the lid is exempt only while it rests on the rim under the heads, by design; the press is commanded 20 mm below the seat and the audit lifts that pose to the seated height because the lid stops the arm). The minimum allowed clearance is 4 mm, and 50 mm between the two arms. These are offline geometry checks only; they do not model how the real pads behave in contact.
