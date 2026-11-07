# Titrate vitamin C with iodine: swirl, and stop at the first lasting blue-black

A chemical technician's bench titration. One arm holds a burette's spring clip squeezed, so that iodine runs into a flask of vitamin C and starch, while the other arm swirls the flask under it. The solution stays milky until the vitamin C is used up, then turns blue-black within a fraction of a millilitre. The arm on the clip must let go at that moment: what has gone in cannot be taken back.

Two things make this harder than filling to a mark. How much iodine the flask needs differs from trial to trial, and nothing shows it in advance. And iodine that has only just landed colours the solution locally, so a dark patch at the surface is not the end until the swirl has mixed it and the colour has stayed.

**Both arms.** The clip shuts by itself when let go, so holding it takes one arm, and the flask needs the other. One arm alternating clip and swirl can still reach the colour, but then at most 0.5, because the score asks for at least 80% of the iodine to go in while the flask moves. `check_scorer.py` checks that score line on a flask dosed still and swirled only afterwards, and `dose_then_swirl` below shows the same dosing in physics, with two arms.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `mechkit.py` and `demokit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them, so that the parts spawn from USD text written at start-up.

## Run a trial

From this folder. `--seed` sets the endpoint, through the seed that `inspect-robots run` derives from it and passes to the reset: `--seed 1` gives reset seed 2566101560 and an endpoint of 14.8 mL. The score record gives both.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 1800 --seed 1 \
    --instruction "Titrate the vitamin C solution in the flask with the iodine in the burette. Hold the burette's spring clip squeezed with one arm to run iodine into the flask, while the other arm swirls the flask under it. Release the clip as soon as the whole solution stays blue-black. Stand the flask back on the white tile and let go with both arms."
```

## The demo

```bash
python demokit.py task.py OUT --seed 1                   # the full-score run, with video, at 640 x 360; endpoint 12.1 mL
python demokit.py task.py OUT --seed 1 --no-record --render-every 1 --mode none,no_swirl,first_flash,fixed_mark,late,very_late,dose_then_swirl,spill,knock,still_held
```

The demo is closed loop. The arms follow the flask's pose and their own joints. The stop comes only from the top camera's pixels on the near side of the flask, compared with what they showed before dosing began. It never reads the endpoint.

The full-score runs and the runs of faults. Seed 1's full run is at 640 x 360, the others at 320 x 176:

| Run | Seed, endpoint | What the arms do | Delivered | Score |
|---|---|---|---|---|
| full | 1, 12.1 mL | The demo | 12.5 mL | 1.0 |
| full | 3, 8.7 mL | The demo | 9.1 mL | 1.0 |
| full | 4, 15.5 mL | The demo | 15.9 mL | 1.0 |
| `none` | 1 | The arms wait | 0 | 0 |
| `no_swirl` | 1 | The clip is held for 25 s with the flask still | 20.1 mL | 0 |
| `first_flash` | 1 | The clip is let go at the first local colour | 0.5 mL | 0 |
| `fixed_mark` | 1 | Stopped at 12 mL by the clock | 12.1 mL, 0.04 short | 0 |
| `late` | 1 | The clip is let go 2.2 s after the colour | 14.2 mL | 0.5 |
| `very_late` | 1 | The clip is let go 5 s after the colour | 16.5 mL | 0 |
| `dose_then_swirl` | 1 | The flask is still while dosing, and swirled between doses | 12.6 mL | 0.5 |
| `spill` | 1 | The flask is carried aside while dosing | 3.4 mL outside | 0 |
| `knock` | 1 | The flask is pushed over | 0 | 0 |
| `still_held` | 1 | The right arm never lets go | 12.5 mL | 0.5 |

`python check_scorer.py` checks every score line on hand-built states, with no simulator. It includes the right final reading with no swirl, which scores 0 because the bulk never turns, and the same reading swirled only afterwards, which scores 0.5. A burette stopped at a fixed 12.0 mL mark and swirled scores 1.0 on 2 of 20 seeds, those whose endpoint happens to fall within its 1 mL band.

## Scoring

`--scorer task_score` prints the total. Per-step `info` is not saved, so the task writes its record to `titration_scores/` (or `$TITRATION_SCORE_DIR`) every second.

V is the iodine delivered, E the endpoint.

- **1.0:** the bulk has reached E, V is at most E + 1.0 mL, at least 80% of V went in while the flask's mouth moved at 2.5 cm/s or more, the flask stands within 3 cm of the tile's centre and at most 10 degrees from upright, every fingertip is at least 3 cm from the flask and from the clip, the clip is shut, and all this holds for 1 s with the flask still (under 1 cm/s and 0.5 rad/s).
- **0.5:** the bulk has reached E and V is at most E + 3 mL, but V is over by more than 1 mL or under 80% was swirled. Also 0.5 for a 1.0 titration not stood back and let go.
- **0.25:** both of those faults.
- **0:** the bulk never reached E; V is over E + 3 mL; more than 0.2 mL landed outside the flask's mouth; or the flask tipped past 45 degrees or fell off the table, at any moment of the trial.

## The model behind the colour

Liquid is drawn, not simulated. Flow is 0 below a quarter of the clip's travel and rises to 0.8 mL/s at 85%. Iodine that lands joins an unmixed pool, which empties into the bulk at 0.01 per second in a still flask and at up to 3 per second with the mouth moving at 5 cm/s or more. The bulk turns once it holds E, pale at once, then blue at 0.05 mL past E and blue-black at 0.15 mL. While iodine is unmixed a dark patch shows at the surface, and it grows at 1 mL unmixed or once the bulk holds 85% of E.

A real trial needs none of this: starch and iodine do it by themselves. The falling jet may mix a still flask sooner than this model does, so a real unswirled run may earn 0.5 where the simulation gives 0. About 0.01 M iodine reacts one to one with vitamin C, about 1.76 mg per mL, so 8 to 16 mL takes 14 to 28 mg of vitamin C in about 150 mL of water with starch. Titrate one flask of the batch first to fix E. Clamp the stand to the table. Iodine at this strength can stain, so work over the tile, in plastic ware.

## Seeds and endpoints

The endpoint is `round(numpy.random.default_rng(seed).uniform(8, 16), 1)` of the seed the reset receives, which `demokit.py --seed` passes as it is: seed 0 gives 13.1 mL, seed 1 gives 12.1 mL, seed 2 gives 10.1 mL, seed 3 gives 8.7 mL, seed 4 gives 15.5 mL. With no seed, seed 0 is used.

## Assets

Every mesh is generated by `task.py` and `mechkit.py`. There are no textures and no third-party assets. Sizes follow a common 250 mL Erlenmeyer flask and a 25 mL burette.

## Licence

MIT
