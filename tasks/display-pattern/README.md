# Continue a shop display's product pattern

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [display_pattern.py](display_pattern.py). The products are models from Arena's object library: `alphabet_soup_can_hope_robolab`, `ketchup_bottle_hope_robolab`, `green_beans_can_hope_robolab`, `mustard_bottle_hope_robolab` and `yogurt_cup_hope_robolab`.

The display repeats soup, ketchup, green beans; the two empty spots must take green beans, then soup.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/display-pattern/display_pattern.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the two empty spots on the display so it continues the same pattern." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/display-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. The right arm moves each item out past the stock row before carrying it across, so it never passes a tall bottle.

The run prints `task_score` at the end. `score()` in the task file gives 0.5 per empty spot holding the right product, and 0 if a product tipped over or the display was moved; `success()` is a score of 1.0.

## Runtime and seeds

- Demo: 429 steps (21 s simulated), about 1.5 minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 7 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
