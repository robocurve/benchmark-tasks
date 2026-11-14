# Hold a filter tray steady and scrape the filter cake into a collection container

The whole task (objects, scoring and scripted demo) is one task file for the `yam_arena` embodiment:
`filter_cake_scrape.py`.

## Offline check (no GPU, a few seconds)

```bash
python tasks/filter-cake-scrape/check_demo.py
```

## Play the scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/filter-cake-scrape/filter_cake_scrape.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the filter tray steady with one hand and use the spatula with the other to scrape all of the solid off the filter into the collection container." \
  --scorer task_score \
  --max-steps 700 --store-frames --log-dir ~/yam-eval/scrape-demo
```

The left arm presses the tray's left end down with its closed fingers; the right arm takes the spatula, sets its edge on the filter behind
the cake, scrapes it off the tray's right end into the container, and puts the spatula back. About 23 s.
Set `SCRAPE_DEBUG=1` to print the crystals in the container and the tray's movement once a second.
`SCRAPE_DEMO_NO_HOLD=1` plays the right arm alone, to check that the tray slides when nobody holds it.

## Run a trial (Dream-1)

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/filter-cake-scrape/filter_cake_scrape.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the filter tray steady with one hand and use the spatula with the other to scrape all of the solid off the filter into the collection container." \
  --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/scrape-dream
```

## Scoring

`task_score` gives 1.0 when the episode ends in success (all twelve crystals in the container, at rest, the tray
steady throughout, held 1 s); otherwise the task file's `score()` at the end of the run: the share of crystals in
the container, halved if the tray ever moved more than 1 cm or tilted or turned more than 5 degrees.
`score()` is called every control step to track the tray's movement over the run.

## Runtime and seeds

- Run limit: 2,400 steps (2 minutes at 20 Hz)
- Wall-clock per trial on one L40S: about 3 minutes, plus about 1 minute of start-up
- Fixed layout (`-E object_jitter=0`); no seed needed