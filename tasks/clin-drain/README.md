# Recharge a 400 mL spring wound-drain reservoir and seal its spout plug

A grey dimpled absorbent bench mat (`260 x 220 x 3 mm`, friction `1.4` in the model) sits fixed on the station bench centred at `(28.5, 4.0) cm` from the midpoint between the arm bases (`x` forward, `y` left). Resting upright at `(28.0, 5.0) cm` on the mat is a loose 400 mL 3-spring closed-wound suction drainage reservoir (`145 g` dry, `Ø112 mm`, `76.0 mm` expanded, `20.0 mm` compressed): white polypropylene top and bottom discs with a generic green lid print (volume, port and spout labels, no brand or CE marks), a clear collapsible film side wall with its white folds showing through, three internal stainless spring columns, a `+y` anti-reflux inlet port with short clear `10 Fr` tubing stub, a moulded `+x` flared pouring spout (`15.6 mm` ID throat, `14 mm` tall), and an open-sided plug-holder cradle extending to `(35.7, 1.0) cm`. Standing upright in the cradle on a `4.1 cm` nominal tether beside the spout is a loose `10 g` LDPE/PP pouring-spout plug (`Ø10.4 mm` tapered `12.5 mm` stem, `Ø27.0 mm` blue seating flange, `20 x 6 x 15 mm` pull-tab). Around the mat, beyond `x = 42 cm` and not touched by the demo, stand visual-only bench props (no collider, no mass, not task objects): a stainless kidney dish, a folded surgical drape, a nitrile glove box, a disinfectant spray bottle, a specimen bag and a disposable syringe (NVIDIA SimReady models, physics stripped).

Re-activating the reservoir requires both arms together: one arm presses straight down on the top disc against the internal springs until top and bottom discs meet flat (`<= 30%` expanded height, `<= 22.8 mm`) on the mat and holds that compression while the second arm picks the tethered plug by its pull-tab, carries it to the flared pouring spout, seats the stem and flange upright flush on the spout rim, and withdraws clear before the pressing arm releases. Once sealed under compression, trapped vacuum holds the unit collapsed (still `<= 30%` expanded height, `<= 22.8 mm`); full score needs this for `4 s` with both grippers clear and both parts still, then `1 s` more until the run ends as a success (`5 s` in all). Putting the plug into the spout while the reservoir is taller than `22.8 mm` fails to create vacuum and scores `0`.

## Run a trial

From this folder, on a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`):

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/clin_drain.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clin-drain-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/clin_drain.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clin-drain-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/clin-drain-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end: `1.0` once `success()` has held for `1.0 s`, otherwise `score()` at the last step. At the official `1280 x 704` review resolution (`4x` over the `320 x 176` `station` `top_cam`), every loose object projects above `20 px` (`min_px = 6` at `320 x 176`, i.e., `24 px` at `1280 x 704`).

## Scoring

Measured at rest after release from a side photo with a ruler (reservoir height vs expanded height; plug flange flush on spout):

| Score | Condition | Check on a real table |
|---|---|---|
| `1.0` | Reservoir compressed (`<= 22.8 mm` prior to sealing), plug seated upright flush on spout rim, both gripper tool-centre points outside the drain and plug bounding boxes grown by `50 mm` on every side, both parts still (`< 1 cm/s`) and the reservoir still `<= 30%` expanded height (`<= 22.8 mm`) for `4 s`, then held `1 s` more until the run ends as a success (`5 s` in all) | Side photo + ruler after `5 s` wait: unit stays flat (`<= 22.8 mm`), plug flange sits flush, both hands clear |
| `0.7` | Reservoir compressed `<= 22.8 mm` and plug seated flush while `<= 22.8 mm`, but the `4 s` clear-and-still hold is not complete (an arm still inside the `50 mm` box, a part still moving, or the reservoir back above `22.8 mm`) | Plug flush on compressed reservoir, grippers still touching or `< 4 s` settled |
| `0.3` | Reservoir `<= 22.8 mm` tall now, or pressed to that height at some moment while upright (`<= 18 deg`) and resting on the mat (base `< 12 mm` above it); plug not seated (still in the cradle, or later knocked out) | Side photo shows discs collapsed during press, spout still unsealed |
| `0.0` | Plug put into the spout while the reservoir is taller than `22.8 mm`, reservoir tipped past `18 deg`, reservoir centre more than `20 mm` beyond the mat edge, reservoir lifted more than `25 mm` above the mat at any time (even if put back), reservoir or plug bottom more than `5 mm` below the table top, or never compressed | Photo shows expanded plugged unit, tipped/fallen reservoir, or untouched start state |

Possible scores: `0, 0.3, 0.7, 1.0`. Run limit: `2,400` steps (`2 minutes` at `20 Hz`).

## Design notes

- **Real mechanism, real scale.** Full `400 mL` closed-wound suction drainage reservoir scale (`Ø112 x 76.0 mm` expanded, `20.0 mm` flat, `26.3%` compressed height) on a `260 x 220 x 3 mm` absorbent bench mat. The `-X` press point (`(-44, -5) mm`) and `+X` pouring spout (`(+62, -2) mm`) are separated by `106 mm` along `X` so both vertical YAM wrists operate simultaneously without `link4` wrist-bracket interference (`arm_arm = 0`). No colour-as-identity; materials follow the product photo (white polypropylene discs with green print, a clear film side wall, steel springs, blue valve cap and stopper).
- **One fixed layout:** `-E object_jitter=0`, no other randomness; reset unseats the plug back to the cradle at `(35.7, 1.0) cm` and returns both spring joints to `0.0` (`76.0 mm` expanded) at `(28.0, 5.0) cm`.
- **Reach & visibility:** Pure open-loop `ArmPlan` waypoints derived solely from fixed scene constants achieve `worst_ik_mm = 0.0739 / 0.0789` (left / right), `worst_ik_deg = 0.0120 / 0.0124` (`0.079 mm` closed-loop MuJoCo tracking) within every joint limit, completing cleanly in `~316` planned steps at `20 Hz` (`15.8 s`, terminating early around step `291` once the `1.0 s` success hold completes), and parking both arms with `0%` camera occlusion of the mat, reservoir, and plug.

## Fidelity

- **Internal spring assembly:** Three internal compression springs (`13.4 N` full-compression load) are modelled as two coaxial series prismatic (`slide`) joints (`bellows_mid` and `top_plate`, `480 N/m` each = `240 N/m` effective stiffness, `-28 mm` stroke each = `-56 mm` total). The maker's catalogue (page 2) states a maximum suction of `30 mmHg`, about `39 N` over this `112 mm` plate; `13.4 N` is a reduced value.
- **Vacuum lock & tether:** PhysX/MuJoCo rigid assemblies do not simulate air pressure differentials or flexible thin elastomeric straps directly; therefore (1) the `40 mm` tether is not simulated (the plug is a loose part; only the blue rim lugs show where it attaches), and (2) when the plug enters the spout upright with its flange flush on the rim, `on_step` latches the current joint displacements `(q_mid, q_top)` as position targets to model trapped vacuum (reverting to expanded `(0.0, 0.0)` immediately if the plug is unseated). No liquid or drainage fluid is simulated.

## Assets & Provenance

| Asset | Source / URL | Revision | sha256 | Licence |
|---|---|---|---|---|
| `labkit.py` | `robocurve/benchmark-tasks` `tasks/analytical-balance/labkit.py` (`yam-isaac-lab-arena`) | Modified (visual hooks: `VISUALS`, `SCENERY`, `dress()` add visual-only USD layers; physics unchanged); upstream file sha256 `710e04aa73e3ea67b1ef7ec6a7d6654f12754ed93ed1e141ff310f0e4e4a31b5` | `c2f8eb8c054955ad22dae4ba7cebff4141639b90c48850a6d7f5dc6715d2bb5b` | Upstream Robocurve task code under the program's open-source task licence (the upstream repository ships no separate licence file); our changes are released under this task's licence |
| `arena_assets_fix.py` | `robocurve/benchmark-tasks` `tasks/analytical-balance/arena_assets_fix.py` | Vendored unchanged | `4804d30037ebf6392a7146ca37868d1256182cdd82285505c21925f9227bbb4c` | Upstream Robocurve task code, as for `labkit.py` |
| `clin_drain.py` procedural labkit assembly (evacuator + plug) | Own model of a generic 400 mL 3-spring closed-wound suction drainage reservoir. Reference photo (shape and colours only): Zimmer Biomet Hemovac catalogue, page 1 (https://www.zimmerbiomet.com/content/dam/zimmer-biomet-japan/Health%20Care%20Professionals/surgical-techniques/surgical/HemoVac%20%E3%82%AB%E3%82%BF%E3%83%AD%E3%82%B0.pdf; page 2 shows the unit in use). No manufacturer drawing or listing publishes the reservoir's outer size; 112 mm diameter, 76 mm expanded and 20 mm flat are our own estimates, sized for the 400 mL capacity. Procedure sources: Lynn Skill 51 (`http://downloads.lww.com/wolterskluwer_vitalstream_com/sample-content/9781582557359_lynn/samples/sample_skills_51-54.pdf`), Mosby procedure (`https://mosby.nwciowa.edu/nursing/mosby/intermediate/assets/procedure/UsingWoundDrainageSystems-procedure.pdf`) | Package authoring | Built directly in `clin_drain.py` | Original package code |
| Absorbent bench mat (`mat`): collider in `clin_drain.py`, visual `assets/visual/mat.usda` + `assets/visual/textures/pad_albedo.jpg`, `pad_normal.jpg` | NVIDIA SimReady `consumable_absorbent_pad_stack_a01` from https://huggingface.co/datasets/nvidia/simready-assets: the top surface of the pad stack over a 260 x 220 mm window, resampled on a regular 2 mm grid with its own UVs (one downward ray per point), its relief scaled by 0.4 onto the 3 mm collider top (from 2.2 mm below, at the centre-fold crease, to 0.3 mm above) over a flat underside, by `assets_build/build_visuals.py`; albedo and normal maps resampled to 2048 px JPEG by `assets_build/make_textures.py` | `7879e0e50423290300ac20ecd6e727325f680b60` | listed below | CC-BY-4.0 (NVIDIA; `assets/visual/LICENSE-simready-absorbent-pad.txt`) |
| Bench props `assets/props/{kidney_dish,drape,glove_box,disinfectant,specimen_bag,syringe}/` (visual only) | NVIDIA SimReady packages `container_system_sterilising_c01`, `drape_surgical_a01`, `box_glove_a01` (scaled 1.27 x 1.05 x 1.15 to a 100-count retail box, ~226 x 123 x 64 mm), `disinfectant_a01`, `medical_sample_bag_a01`, `syringe_a01` from https://huggingface.co/datasets/nvidia/simready-assets, flattened with physics and MDL removed, textures <= 512 px, by `assets_build/build_props.py` | `7879e0e50423290300ac20ecd6e727325f680b60` | per-file in each folder's `LICENSE.txt` | CC-BY-4.0 (NVIDIA; `LICENSE.txt` copied into each prop folder) |
| DejaVu Sans / DejaVu Sans Bold glyphs rasterised into `assets/visual/textures/drain_top_label.png` | https://dejavu-fonts.github.io (system package `fonts-dejavu-core`) | 2.37 | n/a | DejaVu Fonts licence (Bitstream Vera derivative: free use and redistribution) |
| `assets/visual/drain_body.usda`, `drain_mid.usda`, `drain_top.usda`, `plug.usda` + `assets/visual/textures/drain_top_label.png` (reservoir and plug visuals) | Authored for this package in Blender 5.2.3 by `assets_build/build_visuals.py` + `assets_build/make_textures.py` (deterministic) to the dimensions above. Clear film side wall over white film folds, and the lid layout and green ink follow the catalogue photo; the lid carries generic markings only (400 ml, COMPACT EVACUATOR, SUCTION PORT / INSERT TUBE COMPLETELY, POURING SPOUT, caution and single-use symbols, instructions line): no brand word mark, maker's logo, address, CE mark, lot or catalogue number is printed. Model search, 2026-11-14 (index over nvidia/simready-assets, ABO, Objaverse, HSSD, Google Scanned Objects): 'wound drain reservoir', 'hemovac', 'closed wound suction evacuator', 'evacuator', 'spring evacuator bellows': no match; 'drain', 'drainage': only SimReady `drainage_a01` (a drainage bag) and `drainage_pumpSystem_a01` (a Jackson-Pratt bulb), plus oil-drain tools; 'silicone mat', 'non-slip mat', 'rubber mat', 'cutting mat', 'tray liner', 'underpad': no match; 'mat': mattresses, a bathroom mat, `MAT-49`; 'placemat': anti-fatigue floor mats; 'pad': brake pads, `pad_vital_a01` and `consumable_absorbent_pad_stack_a01`; 'absorbent pad' -> `consumable_absorbent_pad_stack_a01` (used for the mat). Props: 'surgical drape' -> `drape_surgical_a01`; 'syringe' -> `syringe_a01`; 'glove box' -> `box_glove_a01`; 'disinfectant' -> `disinfectant_a01`; 'kidney dish', 'specimen bag': no match by name, so `container_system_sterilising_c01` and `medical_sample_bag_a01` were chosen from the SimReady catalogue. | Package authoring, fix round 2 (2026-11-14) | listed below | Original package work (no third-party content) |

sha256 of the visual files:

```
9ba9550ad48438d0836ddab3da480b3b69ffa0aac7b7878b5a0039e7ab429411  assets/visual/LICENSE-simready-absorbent-pad.txt
ad5f284e366214d0bd2809f7a3a0c6a4cff6257c5325173ce7bddff29033463a  assets/visual/drain_body.usda
c86b758792929aecdce8ab2698f6f77e0e622aae41649392c899f0de3a1d8762  assets/visual/drain_mid.usda
4a08c1c65eb6779561c14fd3c8eab86aa6b4fdf9322fbd28614034f6ed92c619  assets/visual/drain_top.usda
2c41fd9864f577f468895a67057bb56c2a9320287c01811b113e08998dfacdd5  assets/visual/mat.usda
3e23366905f6496330e0432b9e71c4c39cfacd201f13caa1f329feb0f1cd6e7c  assets/visual/plug.usda
5dd26359627072c3ad3c3d761d47745bc01fd00799960eb3bd33d10439515ac8  assets/visual/textures/drain_top_label.png
6077a8ab7eaea9f8cdbf8fb66595537347a4f20e8a17fca848fff798b720082d  assets/visual/textures/pad_albedo.jpg
e00387ce81b7ff673dce1e68f2b7ebc13e4616cd3da7f3d998ff19d9e529b46b  assets/visual/textures/pad_normal.jpg
```

## Runtime and seeds

- `2,400` control steps (`2 minutes` at `20 Hz`) per trial; one fixed starting layout (`-E object_jitter=0`).
- The bimanual scripted demo compresses with the left arm, picks and seats the plug with the right arm, withdraws clear and parks both arms; the success needs both grippers clear for `4 s` plus the `1 s` hold (`316` planned steps = `15.8 s` max, ~`291` steps until early success termination).
- Measured in Isaac (`yam_scripted`, 2026-11-15, NVIDIA A10 host): the demo ends as a success after `291` steps (`292` video frames, `14.6 s` of simulated time, `task_score: 1`); Isaac start-up plus the run took about `2 min` wall clock at `320 x 176` cameras. The last `2.0 s` of the top-camera video show both arms parked off the mat.
- Dream-1 (`--policy dream`, `320 x 176`, the exact `task.json` instruction, 2026-11-15, same host type): `task_score: 0` after the full `2,400` steps (the reservoir is never pressed flat); about `10-13 min` wall clock including Isaac start-up (two runs).
- Seeds: none needed. `object_jitter=0` gives one fixed layout, and the scripted demo is open loop.
- Run log: every step of the full-score hold prints one `SUBACTION_END` JSON line per sub-action (both spring-slide joint positions in m, the plug origin in m, the 0-based step, and `run_end: true` on the step the run ends), so a successful run's log ends with the end state of each sub-action.

## Sub-action end states and visible proxy

All parts are at real size, so two end states are small or hidden in the official top camera. They are shown by a declared proxy: the `SUBACTION_END` log values at the last step, end-of-run close-up stills, and the visible mechanical result (top plate down on the mat, blue plug in the spout rim).

| Sub-action | Joint / body | Target (tolerance) | Measured at the last step (`1280 x 704` demo) | Top camera at `1280 x 704` | Close-up still (evidence set) |
|---|---|---|---|---|---|
| Compress the middle spring slide | `drain.bellows_mid` | `-28 mm` (`1.5 mm`) | `-28.0 mm` | hidden under the top plate by design (`0 x 0 px`) | `clin-drain-bellows-compressed-closeup.png` (side view: discs flat, film wall folded) |
| Compress the upper plate slide | `drain.top_plate` | `-28 mm` (`1.5 mm`) | `-28.0 mm` | upper disc about `107 x 98 px` | `clin-drain-bellows-compressed-closeup.png` |
| Seat the spout plug at the compressed-state rim | `plug` | `(342, 48, 772) mm` (`6.5 mm`) | within `2.5 mm` of the target | blue flange (`Ø27 mm`) about `24 x 22 px` | `clin-drain-plug-seated-closeup.png` (plug in the spout rim) |

The total height `76.0 - 28.0 - 28.0 = 20.0 mm` is below the `22.8 mm` scoring limit.

## Scoring and offline checks (no GPU needed)

```bash
python -m pytest -q test_clin_drain.py
python rehearsal.py
```

16 unit/shortcut/reach/occlusion/hold-height/log tests pass offline, and the standalone dynamic MuJoCo contact rehearsal (using animated gripper pad fingers with zero welds) completes at `SCORE 1.0 SUCCESS True`.
