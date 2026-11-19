# Four-Point Kelvin Probe Stepped-Coupon Contact Head

Instruction: "Lower the Kelvin probe head onto the stepped coupon while releasing its counterbalanced transit clip."

Files in this folder: `task.json`, `kelvin_probe_stepped_coupon_contact.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_kelvin_probe_stepped_coupon_contact.py`, `tools/author_visuals.py`, `assets/kelvin-probe-stepped-coupon-contact/`, `README.md`, `DEVIATIONS.md`.

Environment: Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1; `yam_arena` embodiment, `station` scene, 20 Hz control.
Runtime per trial: about 3 min on one A10. The control limit is 900 steps at 20 Hz (45 s simulated time).
Seeds: 0; fixed layout with `object_jitter=0`.

## Run commands

```sh
inspect-robots run --embodiment yam_arena -E task=kelvin_probe_stepped_coupon_contact.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Lower the Kelvin probe head onto the stepped coupon while releasing its counterbalanced transit clip." --scorer task_score --max-steps 900 --seed 0
```

```sh
inspect-robots run --embodiment yam_arena -E task=kelvin_probe_stepped_coupon_contact.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Lower the Kelvin probe head onto the stepped coupon while releasing its counterbalanced transit clip." --scorer task_score --max-steps 900 --seed 0
```

## How it is modelled

Six assemblies (six dynamic rigid bodies >= 55 g plus a fixed ESD bench mat) model a 1:1 materials-characterization sheet-resistance probe fixture on a pebbled ESD mat (`bench_mat`, 0.80 friction):
- **`carrier_stage` (`720 g`)**: unbolted ceramic-and-anodized XY carrier with guide rails, an asymmetric `-Y` polarity key rail blocking 180-deg reversed coupon loading, trunnion posts at `X = 0.208 m`, and a `+Y` transit-clip dock.
- **`datum_fence` (`56 g`)**: compliant ceramic/brass V-anvil datum block on a 5 mm `+X` cushion slide (`240 N/m`, 0.04 N friction).
- **`stepped_coupon` (`68 g`)**: two-step thin-film test coupon (16 mm upper rear step, 8 mm lower front step, four copper pads, arched `-Y` caging handle lug) guided along a 44 mm `+X` linear travel (`25 N/m` return spring toward 0, 0.02 N slide friction).
- **`probe_arm` (`130 g`) & `probe_head` (`55 g`)**: counterbalanced rotary arm (`+Y` hinge, `-26 to +4 deg` stop range with `-22 deg` raised open to `0 deg` seated; `0.080 N*m` detent friction) carrying a compliant 4-pin Kelvin head (`+-4 mm` Z slider, `650 N/m`) whose rear ceramic shoulder traps behind the coupon's upper step once lowered.
- **`transit_clip` (`56 g`)**: weighted safety prop bonded to the `+Y` carrier dock via a 3.5 N breakaway bond (`mk.bond(break_force=3.5)`), physically obstructing the rotary arm above the coupon until pulled clear and seated upright inside the mat rest bay centered at `(0.292, 0.175) m`.

## End-state scoring

Judged purely from observable end-state geometry with caliper, ruler, and digital protractor; succeeds once `1.0` holds continuously for `3.0 s` (`SUCCESS_HOLD_S = 3.0`, `SCORE_OVER = "end"`, zero runtime stiffness or target overrides):
- `0.0`: carrier stage shifted > 15 mm in 3D or tilted > 8 deg, coupon reversed (`facing < 0.85`), transit clip outside the `+Y` rest bay (`+-32 mm` XY of `(0.292, 0.175) m`, `<= 12 mm` above mat top, tilt `<= 20 deg`, `>= 30 mm` from dock), or alignment error > 12 mm / head tilt > 12 deg.
- `0.4`: clip seated upright in bay and coupon forward, max alignment error <= 12 mm and head tilt <= 12 deg.
- `0.7`: max alignment error <= 8 mm and head tilt <= 9 deg (or either gripper within 45 mm of dynamic parts).
- `1.0`: clip seated in bay, max coupon/probe position error <= 6 mm, head tilt <= 6 deg, stage stable, and both grippers >= 45 mm clear of every dynamic body.
Possible scores: `0.0, 0.4, 0.7, 1.0`.

Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_kelvin_probe_stepped_coupon_contact.py`.

## Real-world build and mechanism evidence

1:1 tabletop four-point Kelvin-probe characterization bench (`$76` total BOM, `0%` miniaturization):
- Four-point Kelvin contact probe head with spring-loaded brass pins ($28): https://www.amazon.com/s?k=four+point+probe+head
- Ceramic XY slide carriage and linear guide rail hardware ($32): https://www.mcmaster.com/linear-guide-rails/
- Extension/return spring (25 N/m), counterweight brass collar, and breakaway safety transit clip hardware ($16): https://www.mcmaster.com/springs/
- ESD pebbled bench mat with clip parking bay: https://www.mcmaster.com/rubber-mats/

| Mechanism | Physical joint / spring / contact | Source and approximation |
|---|---|---|
| Stepped coupon stage | 44 mm +X prismatic slide, 25 N/m return spring toward 0, 0.02 N friction, -Y polarity key rail | https://www.mcmaster.com/linear-guide-rails/ ; rigid block, electrical sweep omitted |
| Compliant datum fence | 56 g V-anvil on 5 mm +X slide (`240 N/m`, 0.04 N friction) | machined brass/ceramic stop cushion |
| Counterbalanced probe arm | +Y rotary hinge (-26 to +4 deg stop limits, -22 to 0 deg nominal travel), `0.080 N*m` detent | https://www.mcmaster.com/springs/ ; zero runtime target override |
| Compliant 4-pin Kelvin head | +-4 mm vertical Z compliance (`650 N/m`) + rear step-retaining shoulder | turned brass pogo pins modeled rigid with single Z compliance |
| Breakaway transit clip | 56 g safety prop on 3.5 N breakaway bond blocking probe drop until placed in bay | physical prop obstruction; zero runtime stiffness override |

## Reset

Lift the probe arm to -22 deg open, reseat the weighted transit clip onto its +Y carrier dock bond, slide the stepped coupon back to 0 mm at the start stop, square the carrier stage on the ESD bench mat mark, and reset both arms to ready poses.

## License

| Asset | License |
|---|---|
| `assets/kelvin-probe-stepped-coupon-contact/` (`bench_mat.usdc`, `carrier_stage.usdc`, `datum_fence.usdc`, `stepped_coupon.usdc`, `probe_arm.usdc`, `probe_head.usdc`, `transit_clip.usdc`) and `assets/kelvin-probe-stepped-coupon-contact/textures/*.png` | Procedurally generated by the authors for this task via `tools/author_visuals.py` and the accompanying procedural PBR texture generator (label badge rendered with DejaVu Sans); original own work released under the repository's MIT licence |
| Procedural task collision geometry (`kelvin_probe_stepped_coupon_contact.py`) | Original own work released under the repository's MIT licence |
| Runtime helper Python files (`mechkit.py`, `labkit.py`, `pitch_demo.py`, `arena_assets_fix.py`) | Retained supplied source headers under the repository's MIT licence |

## Importance

A sheet-resistance sweep runs after every deposition batch: the four-point probe head has to come down on a stepped thin-film coupon, land evenly across its pads, and be lifted off again before the coupon moves to the next measurement. In a self-driving materials laboratory the batch loop never stops for a person, so this contact step is what either keeps the loop running or stalls it between every batch.

Two things make it a two-hand job rather than a placement. The carrier stage is not bolted down and shifts under a one-sided load, and the coupon is held on its slide by a light return spring, so an unlatched coupon is pulled off its datum the moment the hold is released. The weighted transit clip is a separate part again: it physically blocks the probe arm until it is lifted clear and parked, and it has to be parked rather than dropped.

This is a workflow argument, not an occupational headcount claim: the step is named for what it is, a probe contact and its safety interlock on a materials-characterization instrument, and the category is `Other` for that reason.

## Nearest tasks

Verified against the 2,720-PR corpus:
- Contrasts with **PR `#1182` (`TK-LAX9SN`, FINAL — semiconductor wafer probing without scratching)** and **PR `#1912` (`TK-9DAZLJ`, MERGED — seating a tympanic probe snugly in an ear canal)**: `#1182` moves a flat wafer under an unlatched probe head while holding it clear, and `#1912` presses a covered thermometer cone into one compliant cavity while pressing a button. Here, an autonomous SDL bench requires releasing a **3.5 N breakaway safety transit clip** (`transit_clip`) that physically blocks a rotary counterbalanced arm (`probe_arm`), holding a **spring-loaded two-step coupon** (`stepped_coupon`, `25 N/m`) aligned at its `+40 mm` V-anvil datum until the **four-pin stepped Kelvin head's rear shoulder** physically traps behind the upper step so both arms can release completely.
