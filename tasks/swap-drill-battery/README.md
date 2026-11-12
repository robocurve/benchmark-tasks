# Swap a cordless drill's flat battery for the charged one

An 18 V drill-driver lies on its side with its flat slide pack locked on its foot; a charged pack lies 26 cm away,
in line with the rail. The instruction is "Swap the drill's battery for the charged one." The flat pack only comes
off while its catch's button is held in, and the charged pack only counts once it is pushed square and all the way
home so the catch drops into its notch.

> **Status:** run and tuned in Isaac Lab-Arena 0.3.1. The scripted demo reaches 1.0 (3 of 3 runs); pulling the flat
> pack without pressing its button scores 0.

| File | What it is |
|---|---|
| `task.json` | Task details |
| `task.py` | Drill, two packs (floating articulations: case + sprung catch), scorer and `outcome()` |
| `geometry.py` | Dimensions, rail, catch, layout, scoring thresholds |
| `build_assets.py` | Writes `assets/drill.usda`, `assets/pack_flat.usda`, `assets/pack_charged.usda` |
| `demo.py` | Open-loop demo; `--no-button` (squeeze the pack without the button) must score 0 |
| `scenekit.py`, `demokit.py`, `usdkit.py`, `taskkit.py` | Helpers |

## Run a trial

```bash
python build_assets.py
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swap the drill's battery for the charged one." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/swap-drill-battery/dream
python demo.py task.py demo_out
python demo.py task.py demo_nobutton --no-button --no-record    # must be 0
```

## Scoring

All measured in the drill's frame (so a drill nudged on the table doesn't matter).

| `outcome()` | Condition | Score |
|---|---|---|
| `swapped` | Flat pack off the rail; charged pack within 2 mm of home, on the rail line (5 mm, 5°), catch up (pressed < 1.5 mm), no fingertip within 1 cm of it, drill and packs still for 1 s | 1.0 |
| `on` | Flat pack off; charged pack on the rail line and no more than 3 cm forward of home | 0.5 |
| `off` | Flat pack off the rail (≥ 6.5 cm forward of home, or > 2 cm off the rail line) | 0.25 |
| `other` | Flat pack still on the rail | 0.0 |

## Checked in simulation

- At rest: the drill and both packs stay put, both catches up, score 0.
- The open fingers take about 4–6.5 cm at full depth. A 69 mm pinch (on a 62 mm-tall pack) stalled on the pack's
  top, so the pack is a slim 50 mm one and the pinch across it, onto the 6 mm button, is 56 mm.
- The drill stays put by its own weight while the packs slide (they ride on the table, not on the rail). A left-arm
  hold on the handle shoved the drill a few mm as its first finger touched, so the demo doesn't use one (`--hold`
  keeps it as an option).
- Demo: squeeze the button and slide the flat pack off, push it aside, then slide the charged pack 26 cm onto the
  rail (commanded 15 mm past home; the arm lags about 8 mm and the stop takes the rest): 1.0 in 565 steps, 3 of 3
  runs.
- `--no-button` (squeeze the flat pack by its back half, over the rail walls): the catch holds, the pads slip, the
  pack and drill stay put: 0.

## Runtime, seeds and reset

Run limit 1,600 steps (80 s); the scripted demo takes 565 steps (28 s). Fixed layout, no seed. Reset per
`reset_notes`.

## Third-party assets and licences

| Asset | Source | Licence |
|---|---|---|
| Drill, packs | Built from primitives by `build_assets.py` | Same as this repository |
| `scenekit.py`, `demokit.py` | Robocurve program template | As distributed by the program |
