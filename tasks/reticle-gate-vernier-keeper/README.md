# Inspect and index the keyed optical reticle carriage past the beam gate to conformance

Instruction: "Inspect and index the polarity-keyed reticle carriage to its conformance datum while holding the beam gate clear and engaging the perpendicular vernier keeper."

Files in this folder: `task.json`, `reticle_gate_vernier_keeper.py` (scene, scoring, demo),
`labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`,
`test_reticle_gate_vernier_keeper.py`, `tools/author_visuals.py`,
`assets/reticle-gate-vernier-keeper/`, `README.md`, `DEVIATIONS.md`.

Environment: Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1; `yam_arena` embodiment, `station` scene, 20 Hz control.
Runtime per trial: about 4 min on one A10. The control limit is 2400 steps at 20 Hz (120 s simulated time).
Seeds: 0; fixed layout with `object_jitter=0`.

## Run commands

```sh
inspect-robots run --embodiment yam_arena -E task=reticle_gate_vernier_keeper.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Inspect and index the polarity-keyed reticle carriage to its conformance datum while holding the beam gate clear and engaging the perpendicular vernier keeper." --scorer task_score --max-steps 2400 --seed 0
```

```sh
inspect-robots run --embodiment yam_arena -E task=reticle_gate_vernier_keeper.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Inspect and index the polarity-keyed reticle carriage to its conformance datum while holding the beam gate clear and engaging the perpendicular vernier keeper." --scorer task_score --max-steps 2400 --seed 0
```

## How it is modelled

Four independent rigid bodies (`1.25 kg` stage, `120 g` beam gate, `82 g` reticle carriage, `65 g` vernier keeper) sit on a fixed `480 x 460 mm` pebbled ESD rubber inspection mat (`bench_mat`, friction `0.90`). All handles include rigid fore/aft and bottom caging shoulders (`>= 20 mm` grip post) and all approach/retreat paths clear the stage vertically before parking cleanly.

- **X-axis crossing spring beam gate (`beam_gate`, `gate_hinge`)**: Pivots `0..52 deg` about horizontal `-X` at `(0.252, +0.035, 0.020)` m against a `0.10 N*m/rad` return spring (`0.003 N*m` friction, rest `0 deg`, zero open detent). Because its pivot axis is parallel to the reticle's `+X` translation, forward force against the closed shutter tooth creates zero opening torque (`0.000 N*m`), making 1-hand bypass physically impossible (`score == 0.0` when unheld).
- **X-axis polarity-keyed reticle carriage (`reticle_carriage`, `reticle_slide`)**: Slides `90 mm` along `+X` dovetail ways to an asymmetric 180-degree stepped datum fence while `beam_gate` is held tilted `+44 deg` overhead. Once seated (`+45 mm` east of the gate slot), easing `beam_gate` closed lowers the shutter behind the reticle datum lug.
- **Y-axis perpendicular vernier keeper (`vernier_keeper`, `keeper_slide`)**: Slides `54 mm` along `+Y`, inserting its primary brass tongue into the seated reticle's `-Y` locking notch (`Y = -108 mm`) while its overhead brass latch arm locks over the closed beam gate striker cap (`Z = 38..45 mm`) to block reopening.

## End-state scoring

Measured at trial end with a caliper/ruler and digital protractor; full success requires `1.0` continuously for `3.0 s`. `0.0`: stage 3D displacement `> 15 mm`, vertical shift `> 10 mm`, or tilt `> 8 deg`; or reticle travel `< 76 mm`. `0.4`: stage steady and reticle travel `>= 76 mm`, but reticle `< 84 mm`, vernier keeper `< 48 mm`, or beam gate angle `> 7 deg`. `0.7`: reticle `>= 84 mm`, keeper `>= 48 mm`, gate within `7 deg` of `0 deg`, and stage steady while either gripper tool point is `< 45 mm` from any scored body or fewer than two tool points are tracked. `1.0`: identical locked geometry with both grippers `>= 45 mm` clear of every scored body across the `3.0 s` hold. Possible scores 0.0, 0.4, 0.7, 1.0.

`components()` reads live rigid poses, joint displacements, and both TCP positions (`SUCCESS_HOLD_S = 3.0`, `SCORE_OVER = "end"`, zero runtime stiffness or target latch rules). Run `PYTHONDONTWRITEBYTECODE=1 python -m pytest -q -p no:cacheprovider test_reticle_gate_vernier_keeper.py`.

## Occupational relevance & real-world build

- **Category `Other`**: an automated dimensional-inspection step. The workload that motivates it is the re-indexing of an optical-comparator reticle at every lot changeover; see `## Importance`.
- **1:1 Hardware BOM (`$84` total, `0%` miniaturization)**:
  - Optical-comparator linear reticle mount stage (`168 x 82 x 92 mm` substage envelope, `$48`): https://www.amazon.com/s?k=optical+comparator+reticle+stage
  - Precision stainless dovetail linear slide & vernier guide ways (`$20`): https://www.mcmaster.com/dovetail-slides/
  - Torsion/compression return spring (`0.10 N*m/rad` return torque `<= 18 N` arm ceiling, `$8`): https://www.mcmaster.com/torsion-springs/
  - Pebbled ESD rubber inspection bench mat (`$8`): https://www.mcmaster.com/rubber-mats/

| Mechanism | Physical joint / spring / contact | Source and approximation |
|---|---|---|
| Crossing beam gate | `0..52 deg` `-X` horizontal pivot hinge, `0.10 N*m/rad` closing spring, `0.003 N*m` friction | https://www.mcmaster.com/torsion-springs/ ; tooth blocks reticle at `0 deg` with zero +X opening torque |
| Polarity reticle slide | `90 mm` `+X` dovetail guide, `0.18 N` passive detent, stepped polarity stop | https://www.mcmaster.com/dovetail-slides/ ; asymmetric datum rejects reversal |
| Vernier keeper bar | `54 mm` `+Y` perpendicular slide, `0.18 N` detent; twin brass fingers engage notch & gate | https://www.mcmaster.com/linear-motion-slides/ ; simultaneous dual-body mechanical lock |
| Unbolted bench stage | Loose `1.25 kg` stage on fixed ESD mat (`0.90` friction) | https://www.mcmaster.com/rubber-mats/ ; `15 mm / 8 deg` stability gate |

## Reset

Withdraw the perpendicular vernier keeper to `0 mm`, tilt the crossing beam gate clear, retract the keyed reticle carriage `90 mm` back to its reset stop, allow the spring gate to settle closed at `0 deg`, and square the stage at `(0.268, 0.000) m`.

## License

| Asset | License |
|---|---|
| `assets/reticle-gate-vernier-keeper/` (`stage.usdc`, `beam_gate.usdc`, `reticle_carriage.usdc`, `vernier_keeper.usdc`, `bench_mat.usdc`) and `assets/reticle-gate-vernier-keeper/textures/*.png` | Procedurally generated by the authors for this task (`tools/author_visuals.py` in Blender 5.2 and Python PIL); original own work released under the repository's MIT licence |
| Procedural task collision geometry (`reticle_gate_vernier_keeper.py`) | Original own work released under the repository's MIT licence |
| Runtime helper Python files (`mechkit.py`, `labkit.py`, `pitch_demo.py`, `arena_assets_fix.py`) | Retained supplied source headers under the repository's MIT licence |

## Importance

An optical comparator only measures what its reticle is set to. At every lot changeover the reticle carriage has to come back to
its stepped datum, the spring-return beam gate has to be eased closed behind it, and the perpendicular vernier keeper has to be
set so the carriage and the gate are locked to each other before the next part is placed on the stage. In an automated
dimensional-inspection cell that runs between setups without a person at the bench, this re-indexing is the step that either keeps
the cell inspecting or stalls it before the next lot.

The same ordered sequence - hold a spring-return gate out of the way, index a keyed carriage onto a stepped datum, close the gate
behind it, and lock both with a perpendicular keeper - is how the stage of a benchtop profile projector is re-set after a
changeover: the carriage is polarity-keyed so it cannot be indexed backwards, and the keeper is the element that carries the load
once the gate is down.

This is a workflow argument, not an occupational headcount claim: the step is named for what it is (a reticle re-index and its
two-body lock on a metrology instrument), and the category is `Other` for that reason.

## Nearest tasks

Verified against the `2,703`-PR benchmark corpus and prior rejection history:
- **PR `#38` (`TK-L264XR`, MERGED — Slide a carriage onto a linear rail until it clicks)** and **PR `#1083` (`TK-UKX5MH`, MERGED — rail carriage click)**: Both score sliding a single carriage along an unobstructed linear rail to a detent click. By contrast, `reticle-gate-vernier-keeper` enforces a **three-axis crossing-gate + X-reticle + Y-vernier mechanical interlock**: an X-axis spring-return (`0.10 N*m/rad`) optical shutter tooth blocks the reticle path at `0 deg` (`score == 0.0` if unheld, zero +X camming torque), requiring continuous left-arm lifting past `40 deg` before indexing the keyed reticle (`90 mm`) and easing closed behind the carriage prior to perpendicular `+Y` vernier dual-locking (`54 mm`).
- **PR `#223` (`TK-Y3NABC`, FINAL — spring-plunger hold-and-open)** and **PR `#1277` (`TK-HJW48D`, FINAL — Miniaturization conveyor-gate parcel push)**: `#223` repeats a repetitive spring-plunger latch on a hinged panel, while `#1277` shrinks a full-scale warehouse parcel conveyor under a miniature 10 cm gate for simple push-through. Here the 1:1 desktop optical-comparator subassembly (`0%` miniaturization) requires lifting an active crossing shutter `44 deg` clear against its closing spring, translating the keyed reticle carriage `90 mm`, lowering the gate behind it, and latching both orthogonal bodies with a perpendicular vernier key.
