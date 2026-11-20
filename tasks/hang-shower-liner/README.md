# Rehang a continuous shower liner edge

Hang the two loose grommets of the shower liner on the inner hooks. Let go of the liner.

## Bench layout

A 1.20 m rod with Moen SR2201-style double hooks sits on two short legs with feet flat on the tabletop. A cut section of the decorative curtain hangs on the outer hooks. A separate cut section of PEVA liner starts on two finger-clearance pads and has two reinforced grommets 174 mm apart.

## What the demo does

Both working eyelets are attached to one continuous hem across the full active width. The old independently moving tabs and separate blue backdrop have been removed. Product grounding is the Moen SR2201 dual-hook threading step. This bench-height pilot tests the local end section with two holes. Its rigid folded liner is an approximation of flexible PEVA.

## Scoring

0.35 per released or held eye at the corresponding hook throat after it passed above the cap, with ±7 mm X and ±6 mm Y limits. Full credit after both eyes remain in their throats with the continuous hem released and still for one second with at most 1 mm of tabletop contact tolerance.

## Reset

Place the linked two-eye hem on both supports. Leave the decorative curtain on the rack.

## Fidelity

The active section is one rigid folded sheet with a connected reinforced hem. Its visible fold vertices define the convex collider and are used to log the lowest point of the liner relative to the tabletop. Set `FAILSAFE_SIDE_CAMERA=1` to capture the fixture from the side. Bench height, the modeled rigid sheet, locally widened 16 mm grommet bores and static hooks simplify a full-height physical shower liner.

[Moen SR2201 technical drawing](https://assets.moen.com/shared/docs/product-specifications/sr2201sp.pdf) · [Frost PEVA liner](https://www.frostproductsltd.com/wp-content/uploads/2022/03/Frost-Code-1144-503-Specification-Information-1.pdf)

## Run the trials in YAM Arena

From this task folder, activate the existing Arena environment and load the instruction and limit from the task specification. Use a clean reset, seed 0 and zero object jitter. Both trials run at 20 Hz with the `task_score` scorer.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
```

### Scripted demo

The recorded demo used the graphical display and 640×352 frames. Set `-E headless=true` to run without a display.
The recorded demo places the top camera at the side of the fixture.
```bash
FAILSAFE_SIDE_CAMERA=1 inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E object_jitter=0 -E headless=false -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/demo
```

### Dream-1

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E object_jitter=0 -E headless=true -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/model_run
```

Read the final `task_score` under `metrics` in `inspect-robots inspect <saved-eval.json>`.

The scripted demo scored **1.0** in the saved evaluation `adhoc_a7ee850b.json`.

The recorded side view shows both eyelets supported and the liner clear of the table. The scorer measured 13.0 mm of clearance at the released end state.

Dream-1 scored **0.0** in the saved evaluation `adhoc_687fbf10.json`.
