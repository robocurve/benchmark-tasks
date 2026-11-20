# Change a 54 mm portafilter from one cup to two

Remove the one-cup basket using the service lever. Install the two-cup basket evenly in the portafilter and release it.

Changing recipes changes the basket. A low lever lifts the nested one-cup basket out of a cold bottomless portafilter so the robot can grip its rim. The new basket seats on the original rim.

## Setup

Cold 54 mm-class bottomless portafilter on a 280 × 114 mm bench cradle with a pivoted ejector. The modeled stainless baskets are 24 mm and 29 mm deep. The single basket starts seated in the portafilter, the double basket is loose at (229, −225, 31) mm, and the 83 × 81 mm discard tray is centered at (253, 193, 0) mm. Coordinates use the tabletop as Z zero.

## Scoring

0.15 after the lever moves downward beyond 8.6° while the original rim rises more than 4.5 mm. 0.35 with the old basket released within 25 mm of the discard tray center and its rim 23–39 mm above the table. 0.65 with the double basket centered within 3.6 mm horizontally and 3.2 mm vertically at the seat with tilt under 8.2°. Full credit after a continuous second with the lever no more than 2.6° below its upright pose, the old basket still in the tray, and both baskets and lever released with the new basket still.

## Reset

Reinstall the one-cup basket, reset lever upright, return the double basket to its start and clear tray.

## Physical replication

Use a cold removable-basket bottomless handle, matching 54 mm one- and two-cup baskets, a workholding cradle and the dimensioned lever. Before using real items, measure the flange depth and the ejection force. Film the bottom lever and the top rim, and inspect clearance and fit before using the spec.

Mark the unloaded lever angle and seated basket rim height. Check that moving the lever downward past 8.6° raises the old rim by more than 4.5 mm. Place the single basket in the discard tray, return the lever, and measure the new rim at opposing sides to check level seating. Verify the replacement remains released within 3.6 mm horizontally and 3.2 mm vertically for one second. Reseat the old basket, return the lever to its unloaded mark, and clear the tray for reset.

## Fidelity

Breville-compatible 54 mm basket class and bottomless head use a disclosed custom ejection cradle. Rigid contacts measure lifting and rim seating. Retention spring force is not calibrated and this task makes no claim about espresso extraction.

[Breville 54 mm double basket](https://www.breville.com/en-us/product/sp0001520) · [Normcore 54 mm bottomless portafilter](https://www.normcorewares.com/products/normcore-naked-bottomless-portafilter-with-handle?variant=44611364258040)

## Run in YAM Arena

Run from this task folder. The recorded settings use seed 0, 20 Hz, zero object jitter, and the step limit and instruction in `task.json`.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$instruction" \
  --seed 0 --scorer task_score --max-steps "$steps" --no-prompt --store-frames \
  --log-dir "logs/demo-$(date -u +%Y%m%dT%H%M%SZ)"
```

Use `inspect-robots inspect <saved-eval.json>` to read the final `task_score` and verify the released end state in the saved frames.

The Lambda scripted demo reached **1.00** in `adhoc_c16d0fa0.json`. The lever ejected the old basket, returned to within 0.4° of upright and cleared the replacement. The final replacement rim datum was offset −1.9 mm in X and 1.7 mm in Y with a 0.0 mm vertical error in the logged frame.

### Dream-1

From the task folder with the same environment, instruction, and limit:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --seed 0 --scorer task_score --max-steps "$steps" --no-prompt --store-frames \
  --log-dir "logs/dream-$(date -u +%Y%m%dT%H%M%SZ)"
```

Dream-1 scored **0.0** in the saved evaluation `adhoc_94fc7dde.json`.
