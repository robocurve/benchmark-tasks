# Press culture tube snap caps to their vented first stop

A teal polypropylene 3-place rack for 17 mm tubes (`56 x 160 x 62 mm`) stands fixed on the station bench centred at `(30.0, 0.0) cm` from the midpoint between the arm bases (`x` forward, `y` left). Its wells at `y = +5.0, 0.0, -5.0 cm` hold three open 14 mL round-bottom clear polypropylene culture tubes (`17 x 100 mm`, about `3 mL` of amber LB broth, blue printed graduations, white writing patch). Beside the rack, on a fixed `110 x 110 x 2 mm` absorbent bench-protector square (white quilted top, blue polyethylene backing) centred at `(24.0, -18.0) cm`, three loose natural-white LDPE two-position snap caps for these tubes (`21.2 mm` across the top flange, `20 mm` tall, hollow skirt `19.2 mm` inside, `5 g`) stand open-side down at `(21.5, -18.0)`, `(25.5, -20.4)` and `(25.5, -15.6) cm`.

The robot caps all three tubes for an overnight aerobic culture: each cap goes square onto a tube mouth and is pressed down only to its first click, the vented stop (`5 mm`), where air still reaches the broth. Pressing on to the second click (about `9 mm`) seals the tube, which starves an aerobic culture; a cap merely set on the tube is not clicked on at all. The skill is a short, force-limited press that stops at an intermediate detent, three times, on small real-size parts.

## Run a trial

From this folder, on a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`):

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/cap1.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cap1-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/cap1.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cap1-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/cap1-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end: `1.0` once `success()` has held for `1.0 s`, otherwise `score()` at the last step.

## Scoring

Measured per tube at the end of the run with a ruler and a side photo. "Pressed depth" is how much lower the cap sits than the same cap just set on the tube (cap top `113.5 mm` above the table when just set on, `108.5 mm` at the first stop, `104.5 mm` sealed).

| Per tube | Condition | Check on a real table |
|---|---|---|
| counts | A cap stands on the tube upright (tilted `<= 20 deg`), centred within `5 mm`, pressed `4-6 mm` (the vented first stop, nominal `5 mm`) | Ruler: cap top `107.5-109.5 mm` above the table; side photo: cap square on the tube |
| does not count | Cap just set on the tube (pressed `< 4 mm`), pressed `> 6 mm` (sealed second stop at about `9 mm`), tilted `> 20 deg`, off-centre `> 5 mm`, or no cap | Cap top above `109.5 mm` or below `107.5 mm`, or visibly tilted / missing |

On real tubes the first-stop height is taken from a spare tube clicked to its first stop, and a cap counts within `1 mm` of it, so the real-world check does not depend on our estimated travel. Score = counted tubes / 3, read at the end of the run: `0` (no tube counts, or the rack or any tube tilted more than `20 deg`), `0.33` (one tube), `0.67` (two tubes, or three while any of the three caps still moves at `>= 3 cm/s` or turns at `>= 0.3 rad/s`), `1.0` (three tubes with all three caps still, `< 3 cm/s` and `< 0.3 rad/s`). A successful run ends after the `1.0` state has held for `1 s`. Over-pressing is penalised per tube: a cap pushed into the sealed stop scores nothing for that tube, exactly like an uncapped tube. Run limit: `2,400` steps (`2 minutes` at `20 Hz`).

## Real-world basis

- **Why this task (category Other: microbiology research automation).** Culture work is routine in microbiology and molecular-biology research; the task isolates one step an automated culture workstation must do, closing a snap-cap tube to a defined intermediate position (vented) and not to the sealed one. No worker count is claimed for this category. Overnight cultures are routine: "Loosely cover the culture with sterile aluminum foil or a cap that is not air tight." then "Incubate bacterial culture at 37 °C for 12–18 h in a shaking incubator." (published protocol "Inoculating a Liquid Bacterial Culture", https://www.addgene.org/protocols/inoculate-bacterial-culture/); "loosen the caps to facilitate aeration of the cultures" before shaking at 37 °C (published competent-cell protocol, https://www.sigmaaldrich.com/US/en/technical-documents/protocol/cell-culture-and-cell-culture-analysis/microbial-cell-culture/competent-cells). Real-trial note: for shared rigs use a nonliving dyed-water stand-in for the broth to reduce contamination risk.
- **Product.** The reference tube is the common sterile 14 mL round-bottom polypropylene culture tube, `17 x 100 mm`, with a polyethylene "dual-position snap cap" that sits either in a vented position or fully closed; equivalent tubes are sold with a "two-position snap cap" for "aerobic (loose) and anaerobic (tight)" culturing (https://morganvillesci.com/shop/mtc-bio-products/culture-tubes/17-x-100-mm-culture-tubes-polypropylene/; other listings, e.g. https://www.thermalscientific.com/falcon-14ml-disposable-sterile-round-bottom-tubes-w-caps-bd-352059/ and https://www.fishersci.com/shop/products/falcon-round-bottom-polypropylene-tubes-dual-position-snap-cap-sterile-w-writing-patch-nominal-volume-14ml-25-pk/1495911b). The models carry no brand marks.
- **Not the same as seating a cap.** Locking or seating tasks are done when the part is pushed home; here pushing home (the second click) is the failure, and the target is the intermediate click a few millimetres short of it.
- **Not published (our estimates; not found - gap):** neither the product listings nor the related detent-closure patent we checked (US4289248A, "Container closure assembly having intermediate positioning means", qualitative only) give numbers for the stop travel (`5 mm` vented, `9 mm` sealed; our estimates for a hand-pressed LDPE cap on a `17 mm` PP rim) or the press forces (the simulated click forces are set as described under Fidelity); the cap's outer size (`21.2 x 20 mm`) is sized to fit over the `17 mm` rim (the listings give no cap drawing).

## Sub-action end states and visible proxy

All parts are at real size, so each cap is small in the official top camera (below `30 x 30 px`). Each sub-action's end state is shown by a declared proxy: the `SUBACTION_END` log values at the last step, end-of-run close-up stills, and the visible result (a white cap on every tube mouth, none left on the pad).

| Sub-action | Body | Target (scored band) | Measured at the last step (`1280 x 704` demo) | Top camera at `1280 x 704` | Close-up still (evidence set) |
|---|---|---|---|---|---|
| Press a snap cap onto the tube at `y = +5 cm` (`tube_0`) to the vented first stop | `cap_0` | pressed `5 mm` (`4-6 mm`), centred within `5 mm` | pressed `5.0 mm`, centred within `2 mm` | white cap about `18 x 21 px` | `cap1-caps-vented-closeup.png` |
| Same for the tube at `y = 0` (`tube_1`) | `cap_1` | as above | pressed `5.0 mm`, centred within `2 mm` | about `19 x 21 px` | `cap1-caps-vented-closeup.png` |
| Same for the tube at `y = -5 cm` (`tube_2`) | `cap_2` | as above | pressed `4.9 mm`, centred within `2 mm` | about `17 x 18 px` | `cap1-caps-vented-closeup.png` |

A cap at the sealed stop would sit about `4 mm` lower (`~1 px` from the top camera), so the stop itself is read from the log and the side close-up, not from the top view.

## Design notes

- **Real size.** Tubes, caps, rack and pad are at real size; no part is enlarged. The click forces are not published; the simulated values are under Fidelity. The caps are small in the official top camera, so their end state is also shown by the proxy above.
- **Why two arms.** The task itself is one-handed work. On this fixed station the right arm reaches the pad and the tubes at `y = 0` and `-5 cm` but not the one at `y = +5 cm`, and the left arm cannot reach the pad, so the scripted demo has the right arm set one cap down on the bare bench in front of the rack at `(22.0, 0.0) cm` for the left arm to pick up. A policy may use either arm in any order; only the end state is scored.
- **Scripted demo (open loop, scene constants only).** Each cap is centred by a light squeeze, gripped by its skirt, carried over its tube, held above the rim for `0.4 s` to let it settle, lowered until its open end is `1.5 mm` below the rim and released, so it drops onto the upper detent row; the closed finger tips then come down on the cap top over `1.2 s` to `7 mm` below its resting height (the lip snaps over the upper row and the cap stops on the lower row at `5 mm`; the tips stay short of the second click) and lift away. Both arms park clear of the rack (`(14, +-25) cm`, `14 cm` above the bench). `759` planned steps (`37.95 s`) at `20 Hz`; the run ends early once all three caps have held still at the first stop for `1 s`.
- **One fixed layout:** `-E object_jitter=0`, no other randomness. Reset: pull any cap off its tube (it pulls back over both stops) and stand the three caps open-side down on the pad at the positions above.

## Fidelity

- **Two stops per tube: a contact detent.** The tube is one rigid body. Below its rim, two rows of four spring-loaded rollers (`4 mm` radius, on radial slide joints at `+x, -x, +y, -y`) stand proud of the tube wall, and the cap skirt carries a `1.4 mm` tall inner lip with a `30 deg` lead-in chamfer at its open end (narrowest radius `8.8 mm`). A cap set on the tube rests with its lip on the upper row. Pressing it pushes the lip over the upper row (rollers `0.5 mm` proud of the lip, `700 N/m` springs): the first click; the cap then stops on the lower row, `5 mm` down (vented). A harder press pushes the lip over the lower row (`0.75 mm` proud, `2000 N/m`) until the cap's inner ceiling lands on the rim, `9 mm` down (sealed). Pulling the cap up passes the rows the other way. Each roller guide has heavy damping and `0.05 N` of sliding friction so a seated cap stays still. No code moves or holds the cap or the rollers: the task's only `on_step` hook reads state for the run log. The score reads the cap's own height.
- **Forces (not product data).** Stop travel and spring rates are our estimates. Bench test of this mechanism in MuJoCo (cap alone, pushed through a `1500 N/m` spring; friction scaled `x0.77 / x1.0 / x1.43`): first click at `0.57 / 0.75 / 1.01 N`, cap then at `4.91 / 4.99 / 4.95 mm`; second click at `1.89 / 2.29 / 3.05 N`; pull-off from the first stop `0.76 / 0.90 / 1.74 N`. Control with every roller held retracted: the cap drops straight to `9.0 mm` under its own weight, no first stop. MuJoCo's soft contacts under-read these forces; with rigid contact at friction `0.5` the same geometry gives about `1 N` and `4-5 N` (hand calculation).
- **Simplified:** tubes are fixed in the rack wells; the broth is static; the elastic give of the real cap bead and tube rim is lumped into the spring-loaded rollers in the tube wall (the cap itself is one rigid body).

## Assets and provenance

| Asset | Source | sha256 | Licence |
|---|---|---|---|
| `labkit.py` | `robocurve/benchmark-tasks` `tasks/analytical-balance/labkit.py` (`yam-isaac-lab-arena`), modified: per-part visual USD models drawn over invisible colliders; upstream file sha256 `710e04aa73e3ea67b1ef7ec6a7d6654f12754ed93ed1e141ff310f0e4e4a31b5` | `74ac261d1f1631c53f7c3f78fbdab644648830d6d0676cd25616a47a2a8c12d5` | Upstream Robocurve task code under the program's open-source task licence; our changes under this task's licence |
| `arena_assets_fix.py` | `robocurve/benchmark-tasks` `tasks/analytical-balance/arena_assets_fix.py`, vendored unchanged | `4804d30037ebf6392a7146ca37868d1256182cdd82285505c21925f9227bbb4c` | As above |
| `assets/visual/tube_body.usda`, `tube_mouth.usda`, `snap_cap.usda`, `tube_rack.usda` + `textures/tube_print.png`, `textures/rack_label.png` | Authored for this package by `assets_build/make_visuals.py` (Blender 5.2 bundled `pxr`) and `assets_build/make_textures.py` to the dimensions above; printed text is graduation numbers, `mL`, `17 mm` and well numbers only (DejaVu Sans Bold, https://dejavu-fonts.github.io, free licence) | `96161a4efc3d`, `75240eea8a29`, `0e4372100cc3`, `bdb5991db86a`, `92dce80d3b78`, `212786689dc2` | Original package work |
| `assets/visual/bench_pad.usda` + `textures/bench_pad.png` | Authored for this package by `assets_build/make_pad.py` (deterministic) | `0e16644e781d`, `4ce842ac7b1f` | Original package work |
| Model search (no match, nothing downloaded is shipped) | `nvidia/simready-assets` rev `7879e0e50423290300ac20ecd6e727325f680b60` (1,616 packages), imagine.io SimReady listing, `Ruinwalker/LabUtopia-Dataset` rev `e622f0ec8123c9098d913ea56a4116fd83a29662`: no snap-cap culture tube, LDPE snap cap or 3-place 17 mm rack (LabUtopia test tubes are glass screw-cap tubes fused into 5-tube racks) | n/a | n/a |

## Runtime and seeds

- Media: the Dream-1 video, the scripted-demo video and the setup screenshot (official `320 x 176` top camera) are attached at upload, which fills the `media` field of `task.json`.
- `2,400` control steps (`2 minutes` at `20 Hz`) per trial; one fixed starting layout (`-E object_jitter=0`).
- Seeds: none (fixed layout, `-E object_jitter=0`; the scripted demo is open loop and reads no simulator state).
- Measured in Isaac (`yam_scripted`, 2026-11-16, NVIDIA A10 host, `1280 x 704` cameras): the demo ends as a success after `762` steps (`38.1 s` of simulated time, `task_score: 1`), about `5 min` wall clock including Isaac start-up. Each cap passes the first click, goes no deeper than `5.3 mm` during the press and ends `5.0`, `5.0` and `4.9 mm` down. The last `1 s` shows all three caps still on the tubes and both arms parked clear of the rack.
- Dream-1 (`--policy dream`, `320 x 176`, the exact `task.json` instruction, 2026-11-16): `task_score: 0` after the full `2,400` steps (no tube counted). One complete episode; an earlier attempt was cancelled at 1,655 steps after repeated session-expired messages and is not used.
- Run log: every step of the full-score hold prints one `SUBACTION_END` JSON line per tube (the capping cap's origin in m, its pressed depth `engagement_m`, the 0-based step, and `run_end: true` on the step the run ends), so a successful run's log ends with the end state of each sub-action. The hook only reads state.

## Offline checks (no GPU needed)

```bash
python -m pytest -q test_cap1.py
```

`33` unit, shortcut, stillness, tilt-threshold, reach, occlusion, visual-alignment and detent tests pass offline. The detent tests check the roller and lip geometry against an independent brute-force contact calculation (a cap set on the tube sits at `0 mm`, on the lower row at `5.0 mm`, sealed at `9 mm`) and that the mechanism is springs and contact only (eight sprung slide joints per tube, guide friction well below the spring force, no hook that writes simulator state). The click forces are measured on the MuJoCo bench (Fidelity) and the full demo in Isaac (above).
