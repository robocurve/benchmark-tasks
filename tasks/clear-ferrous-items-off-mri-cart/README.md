# Clear the steel and electronic items off a patient cart before the MRI scanner room

A two-arm tabletop task. A tray standing in for the top of a patient transport cart holds seven loose, real-size
things. Four must not enter an MRI scanner room (bandage scissors, a penlight, a smartphone hidden under a foam pad,
a stethoscope draped over the foam items) and three may (a foam positioning pad, a foam wedge, a PET water bottle).
The robot has to put the four hazards into a bin and leave the three MR-safe items on the cart. The instruction does
not say which is which.

## Files

| File | Purpose |
|---|---|
| `clear_ferrous_items_off_mri_cart.py` | scene, scorer (`score`, `success`) and scripted demo (`demo`) |
| `task.json` | task card: instruction, setup, scoring, run limit, fidelity |
| `test_clear_ferrous_items_off_mri_cart.py` | scorer tests: start, goal, partial tiers, cheats |
| `yam_parts.py`, `yam_joints.py`, `yam_mesh.py` | our own helper modules for compound parts, hinged chains and meshes |

## Running a trial

Scripted demo (20 Hz control, zero object jitter, 3,000-step limit):

```
inspect-robots run --embodiment yam_arena -E task=clear_ferrous_items_off_mri_cart.py \
    --policy yam_scripted --scorer task_score --max-steps 3000 \
    --instruction "Make this cart safe for the MRI scanner room: put anything that must stay out into the grey bin; leave everything else on the cart."
```

Use any other policy with the same `-E task=` file and instruction. Scorer tests: `pytest test_clear_ferrous_items_off_mri_cart.py`.

- Runtime: the demo plan is 64 s of simulated time (1,274 steps) and reaches success at about 1,190 steps; a trial
  takes roughly 10 to 15 minutes of wall time on one shared GPU.
- Seeds: the scene is deterministic (object jitter 0); there is nothing random in the reset.
- The episode ends with `success` once the score has been 1.0 for 1 s.

## Scene (metres; origin at the midpoint between the arm bases on the table top; x forward, y left, z up)

| # | Object | Material | Size | Mass | Start pose |
|---|---|---|---|---|---|
| 1 | Cart-top tray (fixed) | green ABS | inside 260 (x) x 720 (y) mm, 3 mm floor and walls, rim 25 mm above the floor | fixed | inner faces x 0.140..0.400, y -0.360..0.360 |
| 2 | Bin (fixed) | grey polypropylene | outside 150 (x) x 350 (y) x 80 mm, 3 mm walls and floor (inside 144 x 344 x 77) | fixed | outer faces x -0.020..0.130, y -0.175..0.175 |
| 3 | Lister bandage scissors, 7.25 in | 420 stainless steel | 184 x 67 x 5 mm closed; finger rings 28 mm outside, 19 mm bore; blades angled 30 deg, blunt probe tip; pivot screw 9 mm | 105 g (13.5 cm3 x 7.75 g/cm3) | flat on the tray, mid-length at (0.185, 0.170), rings toward +y, blade tip toward +x |
| 4 | Penlight, 2 x AAA | anodised aluminium barrel; steel head, tail cap and pocket clip | 150 mm long; barrel 13 mm, head and tail cap 15 mm; clip 46 x 5.5 x 1.2 mm | 45 g | on the tray, centre (0.320, 0.190), lamp toward +y, clip on the +x side |
| 5 | Smartphone in a TPU case | glass, aluminium, lithium cell; black TPU case | 150.4 x 74.4 x 10.6 mm; camera island 36 x 36 x 1.6 mm with two 14 mm lenses | 200 g | screen down, centre (0.250, -0.267), camera end toward -y; under the pad |
| 6 | Stethoscope, dual-head | stainless steel chestpiece and stem; PVC tubing; aluminium ear tubes with a steel spring; soft ear tips | 0.70 m overall: chestpiece 45 mm diameter x 22 mm, stem 7 x 20 mm, tubing 10 mm x 440 mm, headset 176 mm long with 6 mm ear tubes 60 mm apart | 160 g (chestpiece 85, tubing 40, headset 35) | chestpiece at (0.281, 0.322) on the tray; tubing along the floor, up the wedge and across to the pad; headset on the pad, yoke at (0.250, -0.150), ear tips at (0.250 +/-0.027, -0.320) |
| 7 | Positioning pad | polyurethane foam, 55 kg/m3 | 190 (y) x 80 (x) x 30 mm | 25 g | centre (0.250, -0.235), lying on the phone; 12 mm of the phone shows at the -y end |
| 8 | Positioning wedge | polyurethane foam, 55 kg/m3 | 120 (y) x 80 (x) x 40 mm ramp, high end toward -y | 11 g | centre (0.250, -0.060) on the tray floor |
| 9 | Water bottle, 0.33 L, capped, full | PET, HDPE cap, water | 60 mm diameter, 165 mm tall | 350 g | upright at (0.360, 0.050) |

Friction (static/dynamic): steel, aluminium, PET, ABS and polypropylene 0.4/0.3; phone case 0.5/0.4; foam 0.8/0.7;
tubing 0.5/0.4.

Stethoscope tubing model: 22 capsule segments of 20 mm, joined by hinges that alternate between horizontal and
vertical bending, each limited to 60 deg (a 40 mm bend radius), with a weak spring (0.006 N m/rad) and damping
(0.002 N m s/rad). The chestpiece and the headset are rigid bodies hinged to the tube ends. Start bends: 15 deg
left then right in the floor run (an S of 31 mm), and 18.6 deg up then level where the tube climbs the wedge.

## Scoring (the same numbers as `score()`)

- A scissors, penlight or phone counts when its bounding-box centre is inside the bin's inner walls, above the
  floor and below the rim, both grippers' centre points are at least 30 mm from its bounding box, and no corner of
  its outline box moves 0.5 mm or more between 20 Hz control steps (under 1 cm/s), all for 1 s. Rest is judged
  from pose changes between steps, not instantaneous speeds, because resting contacts chatter.
- The stethoscope counts when the chestpiece centre, the yoke, the middle of both ear tubes and both ear tips are
  inside the bin's inner walls below the rim; any tubing outside the bin's outer walls is neither over the tray (in
  plan view, including its rim) nor within 10 mm of the table; both grippers are 30 mm clear of its bounding box;
  and no sampled point on it moves 0.5 mm between control steps, all for 1 s. Tubing arching above the rim or
  hanging over it is allowed.
- Score = 0.2 x (hazards that count), plus 0.2 when all four count and the pad, wedge and bottle are each centred
  inside the tray's inner walls, not below the tray floor, released (30 mm) and at rest (same rule) for 1 s, with the bottle
  within 20 deg of upright.
- Score 0 while a pad, wedge or bottle is in the bin (centre inside the inner walls, lowest point below the rim) or
  off the cart (centre outside the tray's inner walls with its lowest point below the rim top). The score is taken
  at the end of the trial, so an MR-safe item taken back out of the bin, or picked up and returned to the tray,
  no longer counts against it; one left in the bin or off the cart zeroes the trial.
- Doing nothing scores 0. Success is score 1.0 held for 1 s.

Measuring a real trial: after release and 1 s of stillness, photographs from above and from the side with a ruler
in view for the bin and tray checks.

## Real-world parts list

- Polymer utility tray about 720 x 260 mm with a 25 mm rim (or a rim taped onto a board), clamped to the table.
- Polypropylene storage bin about 350 x 150 x 80 mm, clamped.
- Lister bandage scissors, 7.25 in, stainless steel.
- Reusable aluminium penlight with a steel pocket clip, 2 x AAA.
- A smartphone of the 6.1 in class (147.6 x 71.6 x 7.8 mm) in a TPU case; a retired handset or a dummy display
  unit of the same size and mass.
- Dual-head stethoscope with 10 mm PVC tubing, 0.70 m overall (27 in class).
- Polyurethane-foam positioning pad cut to 190 x 80 x 30 mm and positioning wedge 120 x 80 x 40 mm.
- 0.33 L PET water bottle, full, capped.

Real-trial safety: no magnet is needed; the scissors are blunt-tipped and stay closed; use a retired phone; keep
people out of the arms' reach and an emergency stop at hand.

## Simplifications

- The tray stands in for the cart top; the cart itself is not modelled.
- No magnetic field is simulated. Whether an item is a hazard follows from its material, as listed above.
- Foam pad and wedge are rigid; the bottle is a rigid solid of its filled mass; the scissors are one rigid body and
  do not open; the headset (yoke, ear tubes, spring) is one rigid body.
- Finger rings and the wedge collide as their convex hulls; all other colliders are boxes, cylinders and spheres.
- The split of the stethoscope's 160 g between its parts is an estimate.

## Licences

Every shape in the scene is built in the task file from primitives and procedurally generated meshes; there are no
third-party models, textures or fonts. `yam_parts.py`, `yam_joints.py` and `yam_mesh.py` are our own code, submitted
under the benchmark's licence.
