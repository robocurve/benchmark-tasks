# Load an ASTM D638 dogbone into two spring-closed wedge grips, jogging the test stand crosshead onto it

Load the dogbone specimen into both wedge grips of the tensile test stand at the 115 mm test length, straight and flat, then step away from the stand.

## Response to the official revision feedback

The official review of this task asked for these changes (quoted from the review; "Importance" and "Feasibility" are its headings); each answer names the file.

| Official text | This version | Where |
|---|---|---|
| "Change the domain to Other and explain how mounting tensile specimens supports materials-research automation" | category is Other; the context says loading the specimen is the first step of every tensile test and that a self-driving materials lab can only automate those studies if a robot can do this step | `task.json` category, context; first paragraph below |
| "the total inspector workforce does not establish that 500,000 workers regularly perform this specific operation" | no workforce or headcount argument is made any more (the occupation anchor was removed from `task.json` and from this README) | `task.json` context |
| "Specify an affordable, reproducible real-trial setup for the stand and both grips, such as a parts list and build files or a low-cost commercial alternative" | a published parts list (about USD 314 with estimated spring, frame and shipping figures; known page prices USD 230.78 for the stage, controller, power supply and two buttons) and build files, because no commercial motorised stand was found at an affordable price (commercial stands with a jog function start near USD 2,500, so this is about one eighth of that): a motorised screw stage with a two-button jog box (Build the stage from the parts list; the scene's STOP key is a fixed visual; the build wires only the two jog buttons), and printed lever-wedge grips with their clamp blocks, a generator script, a geometry check, and an assembly and wiring page | `REAL-TRIAL-PARTS.md`, `build_files/` (`export_grip_cad.py`, `check_assembly.py`, `ASSEMBLY-AND-WIRING.md`, `stl/`), `task.json` real_trial_cost_usd |
| "The named motorized test stand and two specialist wedge grips exceed a few hundred dollars" | the named stand and the commercial wedge grips are replaced by the parts list above (a commercial grip was priced at USD 270-335 each and not used, see `REAL-TRIAL-PARTS.md`; commercial motorised stands start near USD 2,500) | `REAL-TRIAL-PARTS.md` |
| "while preserving the real specimen-loading, spring-grip and crosshead-jog actions" | all three are preserved: specimen loading (hold the lever open, slide the 3.2 mm tab in, release); the spring-closed grip (1.9 N jaw-spring bite, a declared value; a jaw closed on the tab is part of the clamped condition); and the crosshead jog: a momentary key moves the crosshead while it is held, as in the original task. On the real-trial stage the keys are two push buttons wired to a stepper controller whose manual describes a mode in which the motor "rotates when keep press buttons and stop if release button" (the controller manual's hold-to-run mode, https://m.media-amazon.com/images/I/61rXd-xe47L.pdf and https://manuals.plus/cnc/zk-smc02-cnc-stepper-motor-driver-manual.pdf; Build the stage from the parts list) | `qc_met10.py` (`components`, `jaw_on_tab`, jog keys), `task.json` setup, scoring, fidelity |
| Flag: "Include the 17 mm tab marks in the real-trial setup and reset instructions" | the setup says the dogbone carries a pen mark 17 mm from each tab end; the reset notes say to redraw it; the parts list has the marker | `task.json` setup, reset_notes, `REAL-TRIAL-PARTS.md` |

Differences between the simulation and the real trial: the jaw is a 1.9 N spring and the real wedge grip's self-tightening is not simulated (the task ends at mounting, before any pull); the scene's keys are square 24 mm simulation surrogates for the round Maple PBS111 head (head diameter unpublished: an estimate; the datasheet gives a 22 mm hole, a 29 mm bezel and a 30 mm block), the 45 mm key pitch is an estimate, and the 500 mm/min jog speed is the commercial stand's manual value, set on the build by the controller's speed pot.

Tensile testing of plastic and printed specimens (ASTM D638) is basic measurement in materials research, and mounting the specimen in the grips is the first step of every test. A Type I specimen is
165 mm long with 115 mm between the grips. Wedge grips are spring-closed and open only a few millimetres (0-6.4 mm): a grip's lever is held open while the flat tab is slid in through the grip's open-sided thru slot; the
second grip rides the stand's motorised crosshead and is jogged onto the other tab with the stand's LEFT key while its
lever is held open. A specimen gripped at an angle, off the jaw centre, or pushed against a slot end tears early at the
fillet and the result is discarded. The task is holding a spring-closed grip open while threading a 3.2 mm tab into its 6.2 mm opening,
then driving the second grip onto the other tab and clamping both straight at the test length.

## Files

- `qc_met10.py`: scene, mechanism, scoring and the scripted bimanual demo (yam_arena task file).
- `task.json`: title, category, instruction, context, setup, scoring, run limit, fidelity, scores.
- `test_qc_met10.py`: offline checks: scoring tiers, shortcuts, task.json/score() constants agree, demo reach and
  joint limits, demo purity and smoothness, top-camera projection of every scored region at the final arm poses
  (occlusion test).
- `rehearsal.py`: MuJoCo rehearsal with the real arms and finger contact. It plays the demo and adversarial policies
  and scores each with the task's own `score()`.
- `REAL-TRIAL-PARTS.md`, `build_files/`: the real-trial parts list and the printable grip (generator script, geometry check, STL files).
- `SCENE.md`: every object with its real product source, dimensions, material and position.
- `labkit.py`, `arena_assets_fix.py`: shared part library and asset-path fix (see PROVENANCE).
- `assets/qc_met10/`, `tools/`: the visual meshes, their textures and the scripts that author them.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1 run
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/qc_met10.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 900 --store-frames --log-dir logs/dream

# Scripted full-score demo (open loop, waypoints from the fixed layout)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/qc_met10.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 900 --store-frames --log-dir logs/demo

for log in logs/dream/*.json logs/demo/*.json; do inspect-robots video "$log"; done
```

`task_score` is 1.0 once `success()` has held for 3 s (`SUCCESS_HOLD_S`; the runtime counts the hold and ends the
episode); otherwise it is `score()` at the last step. `score()` is the end state's sum (1.0 is reachable at once; the
hold adds no score).
Runtime per trial: about 3-4 min wall clock on one A10 (simulator start included; 171 s demo and 232 s Dream-1 measured on the version before the jaw-closed condition was added); up to 45 s of simulated time (900 steps). The demo succeeds after about 580 steps.
Seeds: none (fixed layout, `-E object_jitter=0`, deterministic). The only loose object is the dogbone.

## Offline checks

```bash
python -m pytest -q test_qc_met10.py
python rehearsal.py <path to a MuJoCo rollout script>
```

## Scoring

Scored at the last step from the end state alone; every row is a ruler, protractor or photo check. One additive table,
one condition per row. Named constants (`qc_met10.py`, shared by `score()`, `success()`, the demo and the tests) in
brackets.

**Seated** = the tab is inside the slot between the jaws (`SEAT_Z_TOL` 3 mm: not on top of the 8 mm upper jaw), past
the 17 mm ruler mark at its jaw face (`DEPTH_MIN`; 25 mm nominal bite) and not beyond its slot end, and overhangs its
19.1 mm jaw by at most 4 mm (`SEAT_X_TOL`). **Clamped** = seated with the lever fully forward and the upper jaw closed on the tab (jaw gap at most `JAW_ON_TAB_MAX` = 3.2 mm tab + 1.5 mm = 4.7 mm; a lever let go with the jaw held open or off the tab does not count).

Each grip's rows count on their own: the -y tab earns rows 4-6 whether or not the +y tab is in. Rows 7-9 are checks of
the mounted specimen and count once both tabs are clamped (a specimen on the tray is also straight, and an empty
stand can be jogged to 115 mm).

| Row | Credit | Condition | Bench check |
|---|---|---|---|
| 1 | +0.10 | +y tab seated (`SEAT_CREDIT`) | 17 mm mark on the tab hidden inside the jaw face; tab between the jaws |
| 2 | +0.10 | lever A fully forward with the +y tab seated: within 2 mm of its forward stop (`LEVER_FWD_MAX`), let go, and the spring-closed jaw on the tab: gap at most 4.7 mm (`JAW_ON_TAB_MAX`, `LEVER_CREDIT`) | red lever against its forward stop (photo); no feeler thicker than 1.5 mm between jaw and tab |
| 3 | +0.05 | the seated +y tab's end leaves a 2 mm gap to its slot end (`STOP_GAP`, `GAP_CREDIT`) | visible gap / 2 mm feeler at the slot end |
| 4-6 | +0.10, +0.10, +0.05 | rows 1-3 for the -y tab and lever B | as rows 1-3, grip B |
| 7 | +0.15 | specimen within 2 deg of the pull axis in yaw and pitch (`SKEW_MAX_DEG`, `STRAIGHT_CREDIT`) | protractor against the bed rail |
| 8 | +0.20 | jaw faces 115 +- 5 mm apart (`GAUGE_L`, `GAUGE_TOL`; ASTM D638 distance between grips; `GAUGE_CREDIT`) | ruler between the jaw faces |
| 9 | +0.15 | both grippers at least 5 cm from the specimen, both levers and both jog keys (`HAND_CLEAR_M`, `CLEAR_CREDIT`) | photo |

Success: the full score (all nine rows) held 3 s (`SUCCESS_HOLD_S`); the trial then ends. The hold is not a row.

Possible scores: 0, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.95, 1
(every sum of the rows above under their gating; 0.9 would need both tab ends within 2 mm of their slot ends, more
than 62 mm of tab inside the grips, which leaves the jaw faces under 103 mm apart, so row 8 fails;
`test_qc_met10.py` enumerates them). Zero: no tab seated (never
placed, lying on top of a closed jaw, hanging off the jaw, knocked onto the mat), also with the crosshead jogged to
115 mm and the arms clear. Partial examples: one grip clamped, the other not reached: 0.25; both tabs in, both levers
held open: 0.30; a tab end against its slot end: 0.95; reopening a lever after success drops that lever's row and rows
7-9. Jaw gap is logged (`SUBACTION_END`) and scored through the clamped condition (at most 4.7 mm, `JAW_ON_TAB_MAX`).

## Mechanism

Each grip: a black housing 38.1 (x) x 62.2 (y) x 49.5 (z) mm with a thru slot open at the front and back, a fixed
serrated lower jaw (1.5 mm 45 deg chamfer on its entry edge at the jaw face) and a 19.1 mm upper jaw on a vertical slide
(joint `gap_l` / `gap_r`; value = jaw gap) pulled shut by its spring (`JAW_K` 600 N/m: 1.9 N bite on the 3.2 mm tab,
3.7 N held open). The red lever handle behind the housing slides 22 mm back (`lever_l` / `lever_r` of `test_stand`)
against a return spring (`LEVER_K` 300 N/m, 6.6 N held fully open): its two wedge rails run under the jaw's follower
bar and lift the jaw to a 6.2 mm gap (inside the real 0-6.4 mm range); let go, the spring returns the lever and the
jaw clamps the tab (gap 2.6-2.7 mm in MuJoCo). Grip B (-y) sits on the stand's crosshead (`crosshead`,
a slide along y that starts backed off 35 mm). The crosshead holds a target that the task's `on_step` advances at
500 mm/min while the panel's LEFT key (`key_left` of `control_panel`, a 2 mm spring key) is pressed past 1.2 mm: the
stand's momentary jog; RIGHT (`key_right`) moves the target back the same way while held. Grip B's upper jaw is its own articulation riding a
carriage driven to the same target (the lever wedge and the jaw it lifts must be in different articulations to
touch). The slot floor has a 3 mm 45 deg lead-in on its front edge. The specimen tray has two 4 mm back stops under
the tabs.

Grip force (simplified, declared): the maker publishes no spring rate or clamp force (the real grip's jaws are
"spring loaded" and the grip is "self-tightening", product page, PROVENANCE). Here the jaw spring bites the 3.2 mm
tab with 1.9 N and the lever is a one-finger push (6.6 N plus the jaw wedge); the wedge's self-tightening under test
load is not simulated, and no test load or preload is applied in this task, which ends with the specimen mounted.

## Demo

Bimanual, about 27 s. Step 1: the left arm grips the +y lever and holds it open against its spring; the right arm
pushes the dogbone along the tray by its -y end to a fixed mark, pushes it back against the tray stops (squares it),
picks it by its neck (pads on the flat neck and first fillet step, clear of the wider steps), carries it in front of
the stand (wrist turned 1.2 deg so the +y tab leads: the slot's friction turns it back onto the axis) and slides
the +y tab sideways into the open +y grip; the left arm opens its fingers and the lever's spring
snaps the jaw onto the tab while the right arm still holds the dogbone; the right arm lets go (the dogbone hangs
clamped in grip A). Step 2: the right arm grips the crosshead grip's lever and holds it open; the left arm holds the
panel's LEFT key down for 4.2 s, so the crosshead drives grip B 35 mm over the -y tab to the 115 mm jaw-face distance,
while the right arm follows the moving lever; the left arm lets go of the key (the crosshead stops), the right arm
eases the lever forward (the spring clamps the tab). Both arms park. Reach (official IK, every control step): worst
error 0.097 mm / 0.012 deg (left), 0.099 mm / 0.013 deg (right); all joints within limits (`test_qc_met10.py` asserts
`worst_ik_mm < 0.5`).

## Sub-action end states and visible proxy

The task file prints one observe-only `SUBACTION_END {json}` line per sub-action on every step of the full-score hold
(`_log_subaction_end`, called from the `on_step` hook; it reads state and never writes it), so a successful run's
last lines are the end state, with `"run_end": true` on the final step. Every sub-action holds its end state to the
final step: nothing is opened again after it closes. The end values below were copied from that run's log (the version before the jaw-closed condition was added: the end values do not depend on it) after the
run (a text-only edit of this README; the task code is the code that ran).

| sub-action (`joint_or_body`) | target / tolerance | Isaac end value, official 1280 x 704 GPU run (step 583) | visible from the top camera? |
|---|---|---|---|
| seat +y tab (`dogbone.tab_+y`, world m) | x 0.300 +- 0.004, z 0.8265 +- 0.003, depth 17-31 mm | (0.2993, 0.0864, 0.8265), depth 28.9 mm | tab 19 x 24 mm inside the grip: about 5 x 6 px at 320 x 176 (< 30 px) -> proxy |
| clamp +y grip (`test_stand.lever_l+upper_jaw_l.gap_l`) | lever <= 2 mm (forward); gap logged and scored (<= 4.7 mm) | lever 0.0 mm, gap 3.12 mm | red handle forward: visible; jaw gap edge-on, hidden by the housing |
| jog grip B over the -y tab (`test_stand.crosshead+dogbone.tab_-y`) | depth 17-31 mm | crosshead 34.6 mm, tab (0.3009, -0.0786, 0.8264), depth 20.7 mm | the grip housing moving 35 mm along the bed: about 8 px -> visible motion; depth -> proxy |
| clamp -y grip (`test_stand.lever_r+upper_jaw_r.gap_r`) | lever <= 2 mm; gap logged and scored (<= 4.7 mm) | lever 0.0 mm, gap 3.05 mm | as the +y grip |
| specimen straight at the gauge length (`dogbone+gauge`) | yaw and pitch <= 2 deg, jaw faces 115 +- 5 mm, tab ends >= 2 mm off the slot ends | yaw 0.55 deg, pitch 0.03 deg, 115.4 mm, slot-end gaps 4.1 / 12.3 mm | yes: the 165 mm ivory dogbone spans about 30-33 px at 320 x 176 |

Visible proxy (real size kept): the measured `SUBACTION_END` values in the official GPU run log,
end-of-run close-up stills of both grips (kept with the run evidence, outside this folder), and the visible mechanical
result: both red handles forward, grip B moved up to the specimen, the dogbone lying straight across both grips. Nothing was enlarged for the pixel
floor.

## Fidelity

Wedge grips at the maker's drawing size (19.1 mm jaws, 0-6.4 mm opening, thru slot) on a horizontal motorized force
test stand, at 1:1, made of labkit parts; the bench dressing behind the stand is visual-only SimReady models. The
grip's rack-and-pinion lever is modelled as a 22 mm spring-return slide whose wedge lifts the spring-closed jaw to
6.2 mm. The task ends at mounting, before any pull, so only the jaw spring holds the tab. Grip force is simplified: the jaw spring bites the tab with a declared 1.9 N (no published value; see
Mechanism), and the real grip's self-tightening wedge (jaws on an incline that bite harder as the test load rises) is
not simulated; no test load or preload is applied. The crosshead is a stiff position servo whose target moves at the
set speed (500 mm/min, inside the stand's optional 0.5-1,100 mm/min speed range; standard 13-330 mm/min) only while LEFT or RIGHT is held (the guide's momentary
KEYS setting). The modelled frame is 460 mm long (the data sheet's frame is 750 mm) with the
panel on its +y end, set flat (the guide: viewing angle adjustable). The force indicator and its load cell sit at the
fixed (+y) end; the maker mounts them on the crosshead adapter (no load is applied in this task). See DEVIATIONS.md.

## PROVENANCE

| Item | Source | Used for |
|---|---|---|
| Wedge grip | Mark-10 G1061 wedge grip, 1,200 lbF, product page https://mark-10.com/products/grips-attachments/tensile-pull/wedge-grips-g1061/ (accessed 2026-11-15: "heavy duty self-tightening grips", "The jaws ride on rollers", "spring loaded jaws open with a ... rack and pinion lever mechanism", "Thru slot for samples of unlimited width"; no spring rate or clamp force published); dimension drawing https://mark-10.com/wp-content/uploads/G1061-tensile-wedge-grip-1-DIM.jpg (sha256 ed2721c615227cb0e764b1ca5a4d4a8e7a29785e51084beeba68171995a1e8e0): A 49.5, B 62.2, C 19.1 (jaw width), D 38.1, F 0-6.4 mm (jaw opening) | housing, jaw width, opening; slot length 33 mm and the lead-ins are (unverified) |
| Test stand | Mark-10 ESM303H horizontal motorized test stand, https://mark-10.com/products/legacy-products/esm-303h/ (accessed 2026-11-15: "300 lbF (1.5 kN) @ up to 24 in/min (610 mm/min)", "Crosshead accommodates any Mark-10 indicator and Series R01 or R03 load cells", "Force gauges cannot be mounted to the ESM303H test stand"; modelled as indicator + R01 load cell, see fidelity); user's guide https://mark-10.com/downloads/product-downloads/manualESM303.pdf (sha256 60117392812ff8dc5ef40e75bea7e6930da0df3c00044564cb44078c130a85e0): p.13 "UP / RIGHT Commences movement in the up (ESM303) or right (ESM303H) direction" / "DOWN / LEFT ... left (ESM303H) direction"; p.24 KEYS "Momentary: The crosshead will move only if the button is held down"; p.16 speed "0.5 - 1,100 mm/min" (default 250); p.7 the panel bracket goes "on either the right end or the left end" of the frame; data sheet https://mark-10.com/downloads/product-downloads/DataSheetESM303H.pdf (sha256 6cd2ec857426437655b7fd5a905cde1646197fd1c76a801d908e09fee553c389): "Max. travel: 18.0 in [457 mm]", "Max. force: 300 lbF [1.5 kN]", "Throat depth: 3.0 in [76 mm]", "control panel may be mounted to the left or right end of the frame" | jog keys, momentary mode, speed, panel at the frame end, 76 mm axis; frame length modelled 460 mm (data sheet 750 mm), key size/pitch, panel tilt and 35 mm start offset (unverified) |

Sha256 N/A for the grip and stand: parametric build from catalogue dimensions; no mesh was downloaded.

| Asset files | Origin | Licence |
|---|---|---|
| `assets/qc_met10/*.usdc` (test_stand, crosshead, control_panel, lever_l, lever_r, upper_jaw_l, upper_jaw_r, dogbone, specimen_tray, bench_mat) | authored in this package by `tools/author_visuals.py` (Blender 5.2.3), converted to binary crates | original work, same licence as this package |
| `build_files/stl/grip_housing.stl`, `build_files/stl/grip_jaw.stl`, `build_files/stl/grip_crossbar.stl`, `build_files/stl/grip_carriage.stl`, `build_files/stl/grip_block.stl` | generated by `build_files/export_grip_cad.py` from this task's own dimensions (the task author wrote it; boxes and extruded polygons, no third-party mesh) | original work, same licence as this package |
| `assets/qc_met10/textures/*.png` | authored in this package by `tools/make_textures.py` (procedural PBR maps; label glyphs from DejaVu Sans) | original work, same licence as this package; DejaVu Sans glyphs under the Bitstream Vera licence |
| `assets/qc_met10/bench_props.usda`, `assets/qc_met10/props/` | NVIDIA SimReady models (keyboard, mouse + pad, vernier caliper, safety glasses) flattened by `tools/make_props.py`, details below | CC-BY-4.0 (each folder keeps its LICENSE.txt) |

- Specimen: ASTM D638-14 Type I (165 x 19 x 3.2 mm, 13 x 57 mm narrow section, 115 mm grip separation),
  https://www.astm.org/d0638-14.html
- Workforce: O*NET 51-9061.00, https://www.onetonline.org/link/details/51-9061.00 (accessed 2026-11-15: core task above, importance 87; "Employment (2025) 602,000 employees"); BLS OEWS May 2023, https://www.bls.gov/oes/2023/may/oes_nat.htm (51-9061: 584,630).
- `labkit.py` sha256 e5df1aab729d3796e6a65a03ba5e13fc61b1d5315aa84444a19cf8824a1a16fd: the shared part library (#621, MIT) plus one addition, a `visual=` USD mesh per
  part/joint that is drawn in place of its shapes (the shapes stay as invisible colliders). `arena_assets_fix.py`
  sha256 4804d30037ebf6392a7146ca37868d1256182cdd82285505c21925f9227bbb4c: unchanged copy.
- Visual meshes `assets/qc_met10/*.usdc` (binary crates of the authored layers; `bench_props.usda` stays text) + `textures/*.png`: authored in this package (`tools/author_visuals.py`,
  Blender 5.2.3; `tools/make_textures.py`) to the dimensions above; PBR finishes (brushed stainless jaws, black
  anodised grip bodies, powder-coated stand, natural LDPE specimen, ESD rubber mat). No Hugging Face
  library holds a tensile stand, wedge grip or dogbone (survey of nvidia/simready-assets and other 3D sets), so these are not third-party meshes; license:
  same as this package. Label glyphs rendered from DejaVu Sans (Bitstream Vera license).
- Bench dressing `assets/qc_met10/props/` (visual only, `bench_props` part): NVIDIA SimReady models from
  https://huggingface.co/datasets/nvidia/simready-assets at revision 7879e0e50423290300ac20ecd6e727325f680b60,
  CC-BY-4.0 (each folder keeps its LICENSE.txt). Flattened by `tools/make_props.py` (physics, joints and MDL shaders
  dropped, UsdPreviewSurface PBR kept, textures as 512 px JPEG). Source root USD sha256:
  `computer_keyboard_a01` cb12b6c9ae0c4270fe7fd26d545e323bd24c71018ac1a4c7f8bbbd5040f1dc4e,
  `computer_mouse_office_b01` 8d727da2cd8fc9ae26b34c4659ec3f17c22f04b8de881c676ad875bf6ae71ee3,
  `instrument_caliper_a01` e8df8a420cdcfe4e97a1b74832908fa6684e5783ca5e21571d6b5f91adf0974f,
  `ppe_eyeProtection_safetyGlasses_a01` 42e4069d2c4b386e2853581cc54c7da2237f6156ab99f0d785f76a9b7702a8d2.

## Real-trial setup

See REAL-TRIAL-PARTS.md: Build a motorised screw stage with a two-button jog box from the parts list, and print two lever-wedge grips (`build_files/export_grip_cad.py` regenerates `build_files/stl/*.stl`, `check_assembly.py` checks the geometry and the spring arithmetic).  The parts, the dimensions, the manual sentence, the assembly and wiring steps and the check procedure are in that file.
