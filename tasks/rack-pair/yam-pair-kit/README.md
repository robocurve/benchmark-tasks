# yam-pair-kit: a four-arm YAM benchmark kit (two stations, two policies) for Inspect Robots

The kit puts two bimanual YAM stations, A and B, on one table in Isaac Lab-Arena. That is four arms in total, and each
station is driven by its own policy. Each policy sees only its own station's three cameras (320×176), its own 14-D
joint state and its own instruction. Its 14-D action goes only to its own two arms. Physics runs at 240 Hz and control
at 20 Hz. The scorer can also read cross-station contact, meaning arm-to-arm contact between the two stations.

| Kind | Name |
|---|---|
| embodiment | `yam_pair`, which loads a task file: the shared scene, objects, scoring and scripted demo |
| policies | `dream_pair` (two concurrent Dream-1 sessions) and `yam_pair_scripted` (the task's demo) |
| scorer | `pair_task_score` (`task_score` also works unchanged) |
| media | `yam-pair-video`, which tiles the 6 cameras, plus an optional overview, into one MP4 |

[CONTRACT.md](CONTRACT.md) is the task-file contract. `examples/shared-kitting/` is a complete example task.

## Install

It needs the Isaac Lab-Arena venv with yam-arena, inspect-robots and inspect-robots-dream already installed. These are
the same requirements as for any `yam_arena` task. The kit only adds entry points, and `--no-deps` leaves every other
package in the venv as it is:

```bash
source <ARENA_DIR>/yam.env                   
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-deps -e ./yam-pair-kit    # or: pip install --no-deps -e ./yam-pair-kit
inspect-robots list | grep -E "yam_pair|dream_pair|pair_task_score"          # check the registration
```

A task folder can ship a copy of the kit, as `./yam-pair-kit`, or refer to a shared copy by a relative path, for
example `../yam-pair-kit`.

## README section that a task can ship

````markdown
## Run a trial

This task runs on the four-arm `yam_pair` embodiment in `yam-pair-kit` (two bimanual YAM stations, each with its own
policy, cameras, state and instruction). Install the kit once into the Isaac Lab-Arena venv:

```bash
source <ARENA_DIR>/yam.env
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-deps -e ../yam-pair-kit
```

Each robot's own instruction is in `task.json` (`instructions.a`, `instructions.b`). The combined `instruction` is the
label the trial is logged under.

```bash
INSTR="$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')"
# Dream-1 on both stations: two independent sessions, queried concurrently every step
inspect-robots run --embodiment yam_pair -E task=task.py -E overview=true --policy dream_pair \
    --instruction "$INSTR" --scorer pair_task_score --max-steps 1800 --store-frames --log-dir logs/dream
# the scripted demo
inspect-robots run --embodiment yam_pair -E task=task.py -E overview=true --policy yam_pair_scripted \
    --instruction "$INSTR" --scorer pair_task_score --max-steps 1800 --store-frames --log-dir logs/demo
yam-pair-video logs/dream dream.mp4            # 2x3 grid of the robots' cameras, plus the overview
```

To give the robots other instructions, use `-P instruction_a="..." -P instruction_b="..."`. To run a solo control, use
`-P only=a` (or `b`): the other station holds still and opens no session. `-E overview=true` adds a third-person camera
for the video. Neither robot sees it.
````

## Commands

| Command | |
|---|---|
| `inspect-robots run --embodiment yam_pair -E task=task.py --policy dream_pair --instruction "$INSTR" ...` | a Dream-1 trial |
| `... --policy yam_pair_scripted ...` | the task's scripted demo |
| `-E layout=facing\|side_by_side -E separation=0.8 -E offset=0 -E yaw_a=0 -E yaw_b=0` | move the stations (yaws are in degrees) |
| `-P instruction_a=... -P instruction_b=...` | per-station instructions that override the task's |
| `-P only=a\|b` | one Dream session; the other station holds |
| `-P rows_per_request=N -P model=... -P timeout_s=...` | passed to both Dream sessions |
| `yam-pair-video <log-dir\|frames-dir\|log.json> out.mp4 [--every 2] [--no-overview]` | the video |

## Tests

```bash
source <ARENA_DIR>/yam.env
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest yam-pair-kit/tests -q     # no simulator; about 1 s
```

The tests cover the following:

- the layout transforms;
- task loading and instruction resolution;
- the information boundary: each station gets copies of its own images and state only;
- the routing in `dream_pair`, its concurrency and its error handling, tested against fake Dream sessions. One test
  goes through Inspect's own `rollout()`: a refusal from B becomes a `PolicyError` naming station B, and the partial
  steps and both transcripts are kept;
- the scorer and the video tiler.

Two simulator probes check a new install. They take about 60 s each:

```bash
cd yam-pair-kit
inspect-robots run --embodiment yam_pair -E task=tests/sim_contact_probe/task.py --policy yam_pair_scripted \
    --instruction probe -P instruction_a=probe -P instruction_b=probe --scorer pair_task_score --max-steps 220
```

Both should score 1.

## Validation (one RTX GPU)


| Run | Command | Result |
|---|---|---|
| `scripted1`, `scripted2` | `--policy yam_pair_scripted -E overview=true --scorer pair_task_score --store-frames` on `examples/shared-kitting` | **score 1.0**; `success` at step 497 (24.9 s); A delivered 2/2 and B 2/2; collision_free 1, 0.0 N. Both runs are identical. |
| `scripted3_bare` | the same, but with no overview, no frames and `--scorer task_score` | **task_score 1.0**: yam_arena's scorer works unchanged |
| `contact_probe` | `tests/sim_contact_probe/task.py` drives A's left and B's right grippers into each other | contact goes above 0.5 N at step 64; only the `a_left`/`b_right` pair reports force (`a_right` stays at 0 N); the force falls back to 0 after they part; `pair.tcp()` puts both tool points on y = 0.305 |
| `layout_side` | `-E layout=side_by_side -E yaw_b=-20` | builds and renders as expected |
| `dream1` | `--policy dream_pair --max-steps 200` (Dream's default 48 rows) | both sessions connected: 4 requests each × 48 rows, round trips of 0.4–2.5 s, inference about 0.3 s, both made concurrently. Both stations' arms moved to the shared tray, the arms tracked the rows to within 0.05 rad, and there was no contact. Score 0, as expected in 10 s. Sessions were replaced 1× (A) and 2× (B) by idle eviction, so `rows_per_request` now defaults to 24 (CONTRACT §6). |

Wall time per control step (20 Hz control, 240 Hz physics, 4 arms, 6 cameras rendered at 640×360):

| Configuration | s/step |
|---|---|
| scripted, 6 cameras, no frames | 0.126 mean (0.126 median, 0.152 p95) |
| scripted, + overview camera + `--store-frames` (7 cameras) | 0.129 mean (0.125 median) |
| `dream_pair` at 48 rows | 0.226 mean, including the request round trips (0.175 for the simulator alone) |

An 1,800-step `dream_pair` trial therefore takes about 6–7 minutes, plus about 25 s to start Isaac. A 1,800-step frame
store with the overview takes about 3 GB (about 1.7 MB per step).

## Limits

- **The information boundary is enforced by the pair policies, not by the embodiment.** Inspect Robots runs one
  policy per trial, so `yam_pair` returns all six cameras and both states. `dream_pair` and `yam_pair_scripted` give
  each station only `boundary.station_observation(...)`, which the tests check. Any new pair policy must use it too.
- **Dream session idle eviction (about 8–10 s).** It is handled by retrying on a new session, as `dream` does, but the
  model's episode memory restarts. `dream_pair` uses 24 rows per request and counts restarts in
  `trial_metadata.yam_pair.dream.sessions`. A heavily shared GPU can still push the gaps up.
- **The eval log does not keep step `info` or scorer metadata.** The cross-station contact peak, the task's
  `report()` and the step time are in `samples[].trial_metadata[].yam_pair`, and the per-step series is in
  `<log-dir>/yam_pair/`.
- **Cross-station contact covers arm-arm normal force only,** over every link (fingers included) and every physics
  substep. Contact through a shared object (both robots pushing one part) is not counted. A task that needs it
  measures it in its own `report()`.
- **The example's `scripted.py` assumes the facing 0.8 m layout.** Moving the stations with `-E`
  breaks the demo, but not the task.
- **One Kit app and one scene per process.** The task file and layout are fixed for the run, and epochs reset the
  same scene.
- In this venv, run pytest with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`: a ROS `launch_testing` plugin on the path breaks
  pytest's start-up.
