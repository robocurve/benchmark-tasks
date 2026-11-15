# Seal a mouse ventilated cage and dock it in its rack bay so both air valves connect

A clear polycarbonate mouse IVC (individually ventilated cage, 391 x 199 x 160 mm) stands on a bench mat in front of
the empty bay of a stainless ventilated rack. Its filter top lies flat on the bench beside it, so the cage is open; its
260 mL water bottle stands upright in the front pocket. Two spring-closed air valves sit in the cage front; the rack
bay has two silicone air nozzles on its rear plenum.

The robot picks up the filter top and sets it on the tub so its gasket seats in register, lifts the cage by its side
handles onto the bay's runners, pushes it home until the nozzles open both valves, and moves both arms clear. The top
goes on first because the arms cannot reach over a docked cage: with the official IK a straight-down grip over the
docked cage at 10-20 cm is 224-304 mm out of reach (`test_arms_cannot_set_a_top_on_a_docked_cage`).

## Real mechanism (generic names; sources)

| Mechanism | What the real product does | Model here |
|---|---|---|
| Filter-top seal | The maker's mouse IVC cage has no latches: "perfect sealing is simply achieved by placing the top on the cage body, through the radial interference between gasket and cage top" (radial silicone gasket round the base rim). Spec sheet: https://www.somatco.com/sealsafe-plus-mouse-ivc-green-line.pdf ; maker shop page ("Latch-Free Radial Cage Sealing"): https://shop.tecniplastusa.com/products/single-sided-green-line-gm500-mouse-cage-rack-with-70-cage-places | `lid`: its own loose 120 g body. A plug skirt under the top (the gasket carrier) fits 1 mm inside the tub mouth with a chamfered lead-in (6 mm on the sides, 13 mm at the ends); set on the tub, the top rests on the flange in register. Retention is gravity plus contact friction 0.5 (declared); no latch or lever is modelled because the product has none. |
| Air valves | Cage valves with spring-loaded pistons are opened when the cage is fully inserted and close on removal (US 4,343,261; US 5,044,316: valves in the cover forced open by the shelf on insertion). Rat IVC racks: the metal air valve is opened by the penetrating silicone nozzle only when docked correctly, inlet left, outlet right (PubMed 24602544). | `cage.valve_l` / `cage.valve_r`: X slides, 8 mm travel, 100 N/m spring (0.7 N at the 7 mm docked travel) and 0.3 N seal friction (declared). Rack nozzles (r 5 mm) at y +-50 mm, tips at x 811 mm; with the cage nose on the stop (x 808 mm) each piston is pushed in 7 mm (open = >= 5 mm). Pulling the cage out lets them close. |
| Runners | Cages hang by their side flanges on the bay's runners, clear of the cell floor. | Runner ledges 17 mm wide, top 145 mm above the table; the hanging cage floor is 16 mm above the cell floor. |

Reference photos of the cage/rack class: https://www.tecniplast.it/usermedia/uk/brochures/IVC_UK.pdf ,
https://www.tecniplast.it/usermedia/us/Brochures%20July%202019/HOUSING_IVC.pdf (dimensions 391 x 199 x 160 mm, 172 mm
with bottle).

### Declared values and how a real bench measures them

| Value | Declared here | Real-bench measurement |
|---|---|---|
| Filter-top gasket drag | friction coefficient 0.5 on the plug skirt, 1 mm radial play; no extra holding force | Seat the top on an empty base, hook a 0-10 N spring scale to the grip spine and pull level along the cage axis; the peak reading before it moves is the drag. Pull straight up for the lift-off force. |
| Valve spring | 100 N/m, 0.5 N at the 5 mm open threshold, 0.7 N at the 7 mm docked travel | Press a push gauge (0-5 N) on the piston face and read the force at 5 mm and 7 mm stroke (depth gauge or ruler beside the piston). |
| Valve seal friction | 0.3 N | Same push gauge: the force to start the piston moving from rest at 0 mm stroke. |
| Register | every plug corner within 1.5 mm of its seated place | Feeler gauge between the top's rim and the base flange at the four corners. |

## Run a trial

From this folder, on a machine with the official simulator (runtime: Isaac Sim yam_arena, 20 Hz,
2,400-step limit; `object_jitter=0`, so there is no seed and every trial starts from the same layout):

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/wet_ivc1.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/wet-ivc1-dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/wet_ivc1.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/wet-ivc1-demo

# Render one MP4 per camera
for log in ~/yam-eval/wet-ivc1-*/*.json; do inspect-robots video "$log"; done
```

`score()` is evaluated every control step on the final state only; the one thing it remembers is how long the full
state has held (no order or history gate). `task_score` is 1.0 once `success()` has held for `SUCCESS_HOLD_S` = 1 s
after the 3 s hold inside `score()` (both arms parked clear for the last >= 4 s, so the top camera ends on the docked,
sealed cage), otherwise `score()` at the end (`SCORE_OVER = "end"`).

## Scoring (final state only)

| Credit | Condition | Check on a real bench |
|---|---|---|
| 0.15 | Cage hanging level on both runners: within 8 mm of runner height, tilt <= 5 degrees, >= 150 mm engaged | look under the cage base: clear of the cell floor |
| +0.20 | Filter top seated on the cage in register and level: every plug corner within 1.5 mm of its seated place | top view: top on the tub, flush with the flange; feeler gauge at the corners |
| +0.25 per valve | Valve piston pushed in >= 5 mm by its rack nozzle, cage on the runners with its nose within 3 mm of the stop | ruler on the piston; nose on the stop |
| +0.15 | All of the above, bottle upright in its pocket, cage, top and bottle at rest, both grippers >= 12 cm from cage and rack, held 3 s | top view, stopwatch |

Every total `score()` can return, and no other (enumerated from the code in `test_every_total_is_reachable_and_listed`):
0, 0.15, 0.2, 0.35, 0.4, 0.6, 0.65, 0.85, 1.0. Cage tipped over 20 degrees or off the table, or a part missing: 0.

Shortcuts (measured, `mj_check` MuJoCo): do nothing 0.0, sweep 0.0, lift-and-drop 0.0, half demo 0.2 (top on, cage
still on the bench), undo after success 0.85. Offline tests (`test_wet_ivc1.py`): top left on the bench and cage docked
0.65, top on but 3-6 mm or 2 degrees out of register 0.65, one valve only 0.6 (0.4 with the top off), cage on the bay
floor 0.2 (top only; 0 with the top off), cage 10 mm short of the stop 0.35, top pushed off after success 0.65, arm near
the cage / bottle tipped / cage moving 0.85.

## Sub-action end states and visible proxy

Each sub-action prints one `SUBACTION_END` JSON line per step during the final hold (observe-only on-step hook; it moves
nothing and writes no joint target): `lid sealed` (largest plug-corner distance from register, m), `cage on runners`
(cage body origin, m), `valve L open` / `valve R open` (cage.valve_l / valve_r, m), `arms clear` (both TCPs, m). In the
official 320 x 176 top camera the start shows the white filter top lying beside the open cage and the end shows the
cage, top on, hanging in the bay against the rear plenum with both arms parked clear; the 12 mm valve pistons are a few
pixels, and their measured travel is in the run log.

## Demo (scripted, open loop, fixed bench constants only)

1. Left arm, open fingers straight down in front of the top on the bench, pushes its front end back about 3 mm so both
   fingers square it; then pinches the grip spine at the top's centre of mass, lifts it, carries it over the cage,
   lowers the skirt into the tub mouth 6 mm behind register and releases; the chamfered skirt seats it. The right arm
   waits outboard.
2. Both arms (35 deg pitch, 15 deg outward roll) pinch the side-handle rails, lift the cage over the runner mouth and
   set its flanges on the runners at x 420 mm, then release.
3. Both arms (25 deg pitch) push the cage's rear flange until its nose meets the rear stop; the nozzles push both valve
   pistons in 7 mm.
4. Both arms back off, lift and park at (140, +-320, 150) mm for the last 5 s.

Plan 30.0 s, 559 steps of 2,400 in MuJoCo; worst IK error 0.085 mm. Robustness (mj_check): +-5 mm / +-3 degree jitter
of every loose part (5 seeds) and friction x0.7 / x1.3 all score 1.0.

## PROVENANCE

| Asset / reference | Source URL | Revision | sha256 | Licence |
|---|---|---|---|---|
| Cage, filter top, gasket, valves, bottle, rack bay, nozzles, mat: `assets/visual/{cage,lid,valve_l,valve_r,bottle,rack,mat}.usda` + `tex/*.png` | authored here: `assets/visual/src/build_visuals.py` (Blender 5.2.3) + `make_textures.py`, to the dimensions above | generated 2026-11-15 | regenerate with the two scripts | original work in this package (task licence); text rasterised with DejaVu fonts (Bitstream Vera licence, embedding allowed) |
| Bench props (visual only, no colliders): glove box, cleaning wipes, two disinfectant bottles, medical face masks, light-duty gloves: `assets/props/*/*.usdc` via `assets/visual/bench_props.usda` | https://huggingface.co/datasets/nvidia/simready-assets (`box_glove_a01`, `cleaning_wipes_a01`, `disinfectant_a01`, `disinfectant_c01`, `facemask_medical_a01`, `ppe_gloves_lightDuty_a01`), flattened by `assets/props/src/flatten_props.py` | `7879e0e50423290300ac20ecd6e727325f680b60` | e0fd2bbb..., d7876c52..., 424038eb..., 938c2726..., 046bf8fa..., 6a334eb8... (file order as listed) | CC-BY-4.0 |
| `labkit.py` | https://github.com/robocurve/benchmark-tasks (`tasks/analytical-balance`) | vendored + `visuals`/`hide_shapes`/`GLASS_TINTS`/`GLASS_IOR` hook | `54e35fe930fbddd21d0a2dd91deaccda093e11a823b21b4702908e5665cb3edc` | as upstream task |
| `arena_assets_fix.py` | https://github.com/robocurve/benchmark-tasks (`tasks/analytical-balance`) | vendored unchanged | `4804d30037ebf6392a7146ca37868d1256182cdd82285505c21925f9227bbb4c` | as upstream task |

Library search (free open libraries first; no model was bought):

| Object | Query / source | Result |
|---|---|---|
| IVC cage | nvidia/simready-assets 2,677-name index (`cage`, `mouse`, `rodent`, `ivc`, `animal`, `tub`); imagineio SimReady catalogue | no match (only a computer mouse, opaque tool cases): authored |
| IVC water bottle (square 55 mm) | SimReady `sipper`, `bottle_water`, `water_bottle` | no match (the nearest SimReady bottle is a round media bottle): authored |
| IVC rack bay, nozzles | SimReady `rack`, `shelf` | no match (warehouse shelving): authored |
| Silicone bench mat | SimReady `_mat`, `mat_` | no match: authored |
| Bench props | SimReady PPE / cleaning rows | 6 models used (table above) |

## Offline checks

```bash
python -m pytest -q test_wet_ivc1.py
```

## Measured run metadata

Runtime per trial: scripted demo 395.56 seconds; Dream-1 886.19 seconds, measured wall time on one A10. These measurements include simulator and policy overhead, not inference cost alone.
Seeds: 1415158438 in both recorded episodes; object_jitter=0 (fixed layout).
