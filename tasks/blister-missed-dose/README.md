# Press today's tablet out of a calendar blister pack that has a missed dose

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [blister_missed_dose.py](blister_missed_dose.py).

A weekly calendar blister card lies in a blister popper. Monday's and Tuesday's tablets are gone, Wednesday's was missed, and it is Thursday. The robot has to press out Thursday's tablet, the fourth cell from the green tab, and leave Wednesday's in. Pressing the next full cell, the easy habit, takes the missed dose instead; pressing both doubles the dose. Either scores 0, as does pressing any other cell. The cells are 3 cm apart, so the press also has to land on the right 1.4 cm dome.

[partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py). It builds the card as one rigid part with a spring-loaded slider under each dome, and gives the task step and reset callbacks. A dome pushed down 4 mm then sinks below the foil and stays there, leaving an empty cell like a crushed blister, and the trial's record of pressed cells is cleared on each reset.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=blister_missed_dose.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "It is Thursday. Press today's tablet out of the blister card. The card starts on Monday at the green tab." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/blister-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 400`. The right arm lowers its closed fingers onto Thursday's dome, presses it flat, lifts and moves clear.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on which cells were pressed out.

## Checks

Scripted variants of the same scene, each replacing only `demo()`:

| Variant | Cells pressed out | Score |
|---|---|---|
| Demo | Thursday | 1.0 |
| Next full cell | Wednesday | 0.0 |
| Missed dose, then today's | Wednesday, Thursday | 0.0 |
| Wrong day | Friday | 0.0 |
| Nothing pressed | none | 0.0 |

The demo was also run for two trials in one process: the reset restores Thursday's dome and clears the record, and both trials score 1.0. No other dome was pressed out in any variant.

There is no partial credit: every outcome other than Thursday's tablet alone is a dosing error or no dose.

## Runtime and seeds

- Demo: about 220 steps (11 s simulated), under a minute on an L40S.
- Dream-1: 1,200 steps (1 minute simulated), about 3 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
