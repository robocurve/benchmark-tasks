# Secure a new car battery's hold-down: hook both J-bolts blind into the tray, fit the bar and press the nuts on

A new car battery stands in its tray at the bottom of an engine bay (a well cut into the table). Two J-bolts stand in
holders at the bay's front corners, the hold-down bar lies on the table beside it, and two flange nuts lie by the bay.
The robot must hook each J-bolt into its slot in the tray, beside the battery and out of sight, lay the bar across the
battery's top with both bolts up through its slots, and push a nut down each bolt onto the bar.

Fitting a new battery is routine work for automotive service technicians and mechanics (O*NET 49-3023.00; about
825,800 US jobs, BLS 2025), and the battery must be held down before the car is driven: a loose battery shifts, can
short its terminals against the hood or body and crack its case. Hold-down kits go on in this order: hook each J-bolt
into its slot in the tray, lay the bar over the battery and the bolts, then run a nut down each bolt onto the bar
(Dorman hold-down kit instructions).

The hard part is physical. Each J-bolt is held by its top and its hook has to go 21 cm down the gap beside the
battery, through a 28 x 10 mm slot, and slide along under the tray's floor; the bolt sways and turns in the hand and
the hook has a few mm of room. Let go, a bolt is held only by its hook and leans a few degrees, so the bar then has to
go down over both leaning bolt tops at once without knocking either out of its slot, and each nut has to follow its
leaning bolt down: pushed straight down, it jams at the bolt's top.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the scene (with the bay cut into the table), `success()`, `score()` and `self_test()` |
| `dims.py` | Every size and place, with sources |
| `scenekit.py` | Loads the J-bolt, bar and nut models and builds the engine bay, tray and battery in the station scene |
| `touch.py` | Robot-to-body contacts from PhysX: whether the robot still touches a bolt, the bar or a nut |
| `assets/jbolt.usda` (4 KB) | A 12 in, 1/4 in J-bolt |
| `assets/bar.usda` (7 KB) | The hold-down bar with its two slots |
| `assets/nut.usda` (6 KB) | A 1/4-20 flange nut |
| `fabrication/tray_and_bracket.pdf` (22 KB) | Dimensioned drawing of the tray (with its hook slots), the bracket and how they fasten together, and the bolt holders |
| `fabrication/tray.stl`, `bracket.stl`, `bolt_holder.stl` (28 KB in all) | The same parts as build-ready STL files, in mm |

The J-bolt, bar and nut are modelled to the hold-down kit's dimensions below; the battery, tray, bracket, bay and
holders are built in `task.py` to the same table. No third-party assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Engine bay | hole x 13.0 to 47.0, y -14.0 to 14.0 | A hole in the table over a dark steel well, its floor 20.65 cm below the table top |
| Battery | centred at (30.0, 0.0), top 4.0 cm up | BCI Group 35, 23.0 x 17.5 x 22.5 cm, black polypropylene case, two lead posts, length along x; seated in its tray (about 17 kg). For trials, use a discharged, inert battery or a dummy of the same size and mass |
| Tray | x 18.0 to 42.0, y -10.6 to 10.6 | Black-coated steel, floor 2 mm thick, fastened with four M5 screws to a steel bracket (x 19.0 to 41.0, y -8.0 to 8.0) that carries it 1.95 cm above the well's floor; 4 cm side and end walls (drawing and STL in `fabrication/`) |
| Hook slots | x 28.6 to 31.4, \|y\| 9.0 to 10.0 | One in the tray's floor halfway along each long side of the battery, 28 x 10 mm |
| J-bolts | shanks at (14.9, 11.2) and (14.9, -11.2) | Zinc-plated steel, 1/4 in (6.35 mm) rod, 30.5 cm from the hook's underside to the top, the hook's tip 12.4 mm from the shank and 18 mm tall, 80 g; each stands hook down, tip towards +x, in its holder |
| Holders | on the well's floor at (15.5, +-11.2) | Orange plastic blocks 3 x 3 x 6 cm, each with a 20 x 7 mm slot through it |
| Bar | centred at (30.0, -25.0), flat, along x | Black-coated steel, 22.8 x 2.54 x 0.4 cm, 170 g, a 10 x 20 mm slot (long along the bar) centred 9.6 cm either side of its middle |
| Nuts | flange down at (17.0, 20.0) and (17.0, -20.0) | Zinc-plated steel 1/4-20 flange nuts, 14.3 mm tall, their 25.4 mm flange bearing on the bar as a washer, 25 g each |

Sources: workers, BLS *Occupational Outlook Handbook* and O*NET 49-3023.00 (onetonline.org/link/details/49-3023.00);
order of work, Dorman battery hold-down kit instructions; battery size, BCI Group Size chart (Group 35, max 230 x 175 x
225 mm; batterycouncil.org); battery mass, retailer listings for Group 35 flooded batteries (35-38 lb); J-bolts, 1/4-20
J-bolts sold 6 to 12 in long (Deka DW00916, Lawson 97542); bar width, 1 in (NAPA/Grote hold-down frame); nuts, 1/4-20
flange nuts 9/16 in tall with a 1 in flange (McMaster-Carr). The tray's slots, the bracket and the bar's slots are
sized to fit these 1/4 in J-bolts.

## Run a trial

From this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Secure the battery with its hold-down: hook each J-bolt into its slot in the battery tray, lay the bar across the battery's top with both bolts through its slots, then push a nut down each bolt onto the bar." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/battery-dream
```

`task_score` prints the trial's score: 1.0 on success (held for 1 s), otherwise `score()` at the end of the episode.
A trial takes about 4 minutes on one RTX 4080 (scene start-up included). Seed 0; the scene has no random placement
(`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules on 23 hand-built states and 9 size checks.

## Demo

The demo video was made with a scripted, closed-loop demo (before each move of a hook or a nut it reads where the part
really is and corrects the arm's target by the remaining error). It reaches 1.0 in about 70 s (about 1,400 steps),
3 times out of 3.
Laying the bar on with the bolts left in their holders scores 0, and so does putting the hooks down through the slots
without sliding them under the floor (the bolts fall over when let go).
