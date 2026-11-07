# Serve a beer, and start again when a wasp lands in it

A pint is filled at a bar's tap and served to a customer on a round pad. Then a wasp lands in it and her face turns upset. The instruction asks for that glass to be taken off the pad and a new one served from the other clean glass, and does not name the wasp.

The job looks finished once the first drink is on the pad, and the only strong signal that it is not is her face. A careless policy stops there, or refills the spoiled glass.

The instruction says "one arm" and "the other arm", but **the scorer does not check which arm does what**. The handle stays where it is put, so one arm might open the tap and then fill a glass alone. By the harness's IK, without collisions, the right arm misses the handle by 9.5 cm straight down and reaches it with a tilted or level grip. That route has not been run.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source <ARENA_DIR>/yam.env`).
- `kitchenkit.py`, `beer_usd.py` and `beer_mesh.py` are in this folder, with the textures in `textures/`. Importing `kitchenkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them. Each part, declared as a subclass of `Box`, then spawns from a USD file written at start-up: modelled meshes for what is seen, simple separate shapes for what collides.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 2400 \
    --instruction "Fill a clean glass from the tap: hold it up under the spout with one arm, turn the red handle to open the tap with the other arm, and turn the handle back when the glass is full. Serve it by placing it on the round pad on the bar. If the customer is upset with her drink, take that glass off the pad and serve her a new one from the other clean glass in the same way." --store-frames --log-dir out
```

`bash run_dream.sh OUT_DIR [ROWS_PER_REQUEST] [MAX_STEPS]` runs that command and stops it if the model's session is lost mid-run. The gateway drops a session left idle for about 8 s of wall time, and the adapter then quietly starts a new episode. The slowest 30 steps took 6.77 s in the Dream-1 run, alone on the GPU, so 20 rows take about 4.5 s.

## Scripted runs

`bash run_scripted.sh OUT_DIR MODE STEPS [WIDTH HEIGHT]` runs one, with `BEER_DEMO` set to `MODE`, using `--policy yam_scripted`. The plans are open loop, from the known layout. The right arm grips each glass level from the side, so that its mouth stays open under the stream. The straight-down plan cannot express that, so a small hook, used only by the scripted policy, carries the whole grip rotation through a waypoint. The heights commanded at the spout are raised 3 cm for the arm's droop under load, measured with a glass in the grip. The left arm pinches the handle's wing from above and turns it with the wrist.

| Mode | What the arms do | Steps | Score |
|---|---|---|---|
| `safe` | The whole service: both deliveries, ending with the handle shut | 1,521 | 1.0 |
| `sweep` | The first delivery; the first glass is knocked over, then the other glass is delivered. Her face turns happy, but the score stays at the first delivery | 1,699 | 0.5 |
| `offer` | The first delivery only | 900 | 0.5 |
| `off` | The first delivery, then that glass is taken off the pad | 1,239 | 0.5 |
| `unoffered` | As `off`, then the other glass is filled and stood back on the table | 1,857 | 0.5 |
| `refill` | As `off`, then the first glass goes back under the tap and back on the pad. It is already full, so this does not show its level rising again | 1,727 | 0.5 |
| `top` | A straight-down grip round the glass, held up under the open tap for 6.7 s with the palm over its mouth, so it does not fill | 831 | 0 |
| `none` | The arms wait | 640 | 0 |

`inspect-robots video <log>.json` writes one MP4 per camera, and `python make_video.py <frames dir> out.mp4 --setup setup.png` joins them and writes the setup screenshot. `python stills.py <frames dir> <out dir> --at 0 6` writes stills at the run's size and at 320 x 176. `python make_textures.py` rewrites the textures.

## Scoring

`--scorer task_score` prints the total. Per-step `info` is not saved, so the task also writes its record to `beer_scores/score_*.json` (or `$BEER_SCORE_DIR`) once per simulated second, and every step's positions to `trace_*.json`. Lines starting `[beer]` in the log mark each event.

- **Full:** 3.6 s in total, 90% of a 4 s fill, with the handle at 75 degrees or more, the glass's rim from 0 to 2.5 cm below the spout's tip, its axis within 2 cm of the spout's and tilted at most 15 degrees, and no part of either gripper over its mouth (above its rim and within its radius of its axis).
- **Delivered:** full, and for 0.5 s its axis within 3 cm of the pad's centre, its base within 6 mm of the pad's top, tilted at most 5 degrees and still (under 1 cm/s and 0.5 rad/s). For the first delivery, every finger must also be at least 3 cm horizontally from the glass's side throughout that 0.5 s. The first delivery counts, and the wasp sets off, when that 0.5 s ends.
- **Knocked over, at any moment of the trial (latched):** a glass tips over 45 degrees or falls off the table. It no longer counts, and full score is out of reach. A delivery already made stands.
- **1.0:** once the wasp has landed, the other glass is delivered, and then it is still, the handle is at 10 degrees or less, every finger is at least 3 cm horizontally from the glass's side and neither glass has ever been knocked over, all held for 1 s. Where the first glass ends up is not looked at otherwise. It cannot stay on the pad, because the other glass takes its place.
- **Otherwise, at the end of the trial:** 0.5 with a first delivery; else 0. Refilling the first glass and putting it back stays at 0.5, because only the other glass counts as the second delivery.
- **Refusal:** If the model refuses at any point, the trial ends there and scores as at the end of a run, so never 1.0. The harness then prints no `task_score`, so read `score_now` in the last `score_*.json`; it never reads 1.0, and full score comes only through the 1 s hold.

The sink is in the scene and no scoring line reads it. The demo stands the spoiled glass back on the table, because level fingers cannot reach into the basin.

## Checks

These run offline with the harness's Python, from this folder, with no simulator: `HARNESS_DIR=<harness> python <script>`.

- `check_two_arms.py`: whether one arm could do both jobs, by the harness's IK. It gives the figures for the handle above.
- `check_side_grip.py`: which side grips the right arm can hold at each place a glass goes, by the harness's IK and the gripper's collision meshes.
- `check_cover.py`: the scorer's rule on a gripper over a glass's mouth, tried at each grip with the task's own points and test.

## Results

| Run | Steps | Score | What happens |
|---|---|---|---|
| Scripted `safe` | 1,521 | 1.0 | First glass full at 20.45 s. The wasp sets off at 30.25 s and lands at 32.75 s, and she turns upset. Second glass full at 65.85 s and on the pad, and she is happy at 73.70 s |
| Dream-1 | 2,400 | 0 | The right arm keeps its gripper nearly shut from the first seconds and hovers over the two glasses from about 10 s to 75 s. At 58 s its fingertips are about 2 cm above one glass's rim height and 3.3 cm off its axis. It touches neither, then moves away. From about 85 s to the end the left arm hovers 5 to 7 cm from the red handle and does not turn it (at most 0.1 degree). No glass moves, nothing fills and the wasp never comes |

The Dream-1 run was one session from the first action to the end: 119 calls at 20 rows per call, round trip 0.52 s on average and at most 3.67 s, 276 s in all. It ended at the step limit.

## Simplified

Liquid is not simulated: the fill level, foam and stream are visuals, and a glass keeps one mass whether full or empty. The wasp and her faces are visuals driven by the scene's state. Holding the glass up is the task's rule, not physics: a bartender does it so that the pour does not foam, and a real glass on the drip tray would still catch beer, so a real trial applies the same 2.5 cm rule. A real trial uses a plastic glass, a real or mock tap with a 90 degree handle and a stop, a toy wasp dropped in by a person, and a face card changed by a person.

The demo's heights at the spout fit this layout, and the glass leans up to 6.7 degrees in the grip while it fills. The wasp is a speck at 320 x 176, so her face is the cue.

## Runtime and seeds

On an RTX 5090 with `--store-frames`, including Isaac Sim's start-up: 276 s for the 2,400 steps of the Dream-1 run at 320 x 176, alone on the GPU, and about 196 s for the 1,521 steps of the `safe` run at 640 x 360, with the GPU possibly shared. The first start on a new machine adds several minutes while Isaac Sim compiles its shaders. The layout is fixed with `-E object_jitter=0`, no seed is passed (the log records the default, 0), and every trial starts identically.

## Sources

| Asset | Source | Licence |
|---|---|---|
| `textures/face_happy.png`, `textures/face_upset.png`, `textures/dark_wood.jpg` | Our own, from `make_textures.py` | MIT, as this task |
| Meshes | Our own, from `beer_usd.py` and `beer_mesh.py`, proportioned on common products: a pint glass, a bar tap, a drip tray and a bar sink | MIT, as this task |

## Licence

MIT.
