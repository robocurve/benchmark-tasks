# Find the 10 mg pack among look-alike cartons whose labels face away, and dispense only it

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [check_label.py](check_label.py).

Four identical white medicine cartons stand in a tote with their labels facing away from the arms. One label has the red band of the 10 mg strength, the other three the green band of 5 mg. No camera can see a label at the start (checked below), so the robot has to turn or lift cartons to look before it chooses: active perception, the step a policy acting on what it currently sees skips. Dispensing a guess is the error the scoring separates from dispensing after a check: the right carton served without its label ever being turned to the arms scores 0.5, not 1.0, and any 5 mg carton in the basket scores 0.

The label is a coloured band, not text, because the simulation renders no readable text.

[partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py). It builds each carton as one rigid part with its label panel and band on one face, and gives the task a reset callback, which clears the record of the label check at the start of each trial.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=check_label.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the 10 mg box in the basket. The 10 mg packs have a red band on their label." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/check-label-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1500`. Each arm turns its two cartons half a turn, in two quarter turns with a fresh grip between, so their labels face the arms; the right arm finds the red band on its second carton and lifts that carton into the basket. The arms take turns at the two middle cartons so they never work side by side.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on what ends in the basket and on whether the 10 mg label was checked first.

## Checks

At the start, with the arms at rest, none of the three cameras shows a single red or green band pixel. After the demo has turned the cartons, the top camera shows the bands.

Scripted variants of the same scene, each replacing only `demo()`:

| Variant | Score |
|---|---|
| Demo: turn the cartons, dispense the red-band one | 1.0 |
| Dispense the 10 mg carton without turning any (a lucky guess) | 0.5 |
| Dispense a 5 mg carton without turning any | 0.0 |
| Turn all four, then dispense a 5 mg carton | 0.0 |
| Turn all four, dispense nothing | 0.25 |
| Nothing done | 0.0 |

The demo was also run for two trials in one process; both scored 1.0.

## Runtime and seeds

- Demo: about 1,300 steps (65 s simulated), about three minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
