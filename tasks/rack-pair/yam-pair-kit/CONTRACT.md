# yam_pair task contract

`yam-pair-kit` adds a four-arm, two-station YAM embodiment to Inspect Robots. It registers the following:

| Kind | Name | What it is |
|---|---|---|
| embodiment | `yam_pair` | two bimanual YAM stations (A and B) on one table in Isaac Lab-Arena, loaded from a task file |
| policy | `dream_pair` | two independent Dream-1 sessions, one per station, queried concurrently every control step |
| policy | `yam_pair_scripted` | plays the task file's `demo()` for both stations |
| scorer | `pair_task_score` | the same value as `task_score`, plus cross-station contact and the task's `report()` in metadata |
| command | `yam-pair-video` | tiles a `--store-frames` run (6 cameras, plus the overview) into one MP4 |

`task_score` (from yam_arena) works unchanged on `yam_pair`.

Status: implemented and checked in the simulator (see README "Validation"). The scripted example scores 1.0, both
probes score 1.0, and a Dream-1 run connected both sessions. Calling a task-folder scenekit's decorator hooks is implemented but
has not been run. Breaking changes are listed at the end of this file.

---

## 1. Frames and layout

**Station frame.** Each station uses yam_arena's station frame unchanged: x points forward from the arms, y to their
left, and z up, in metres. The arm bases sit at (0, ±0.305, 0.75) and the table top at z = 0.7475
(`TABLE_TOP`). A single-station plan, IK call or waypoint list therefore works in a station's frame without changes.

**World frame.** Task objects are placed in the world frame. Its origin is the midpoint between the two station
origins, and z is the same as in the station frames. A station's origin is the midpoint between its arm bases on the
floor. Station *s* maps to the world as `world = origin_s + Rz(heading_s) @ local`.

| `arrangement` | A origin | A heading | B origin | B heading | default `separation` |
|---|---|---|---|---|---|
| `facing` (default) | (−sep/2, −offset/2) | 0° | (+sep/2, +offset/2) | 180° | 0.8 m |
| `side_by_side` | (−offset/2, +sep/2) | 0° | (+offset/2, −sep/2) | 0° | 1.0 m |

`yaw_a` and `yaw_b` are in **degrees**. Each one turns its station about the station's own origin, on top of the
heading in the table. In the `facing` layout at 0.8 m, A's arms stand at world x = −0.4 and B's at x = +0.4,
across the table from each other. In `side_by_side`, both stations face +x
and A is on the left (+y).

Use `yam_pair.task.Layout` to convert between frames: `layout.to_world("a", p)`, `layout.to_local("b", p)`,
`layout.yaw_to_local("b", yaw)`, `layout.bases("a")` (world positions of the arm bases) and `layout.origin("b")`.

The defaults come from the task file's `LAYOUT`. Any field can be overridden with `-E` (see §4).

## 2. The task file

A task file is a plain Python module that does not import Isaac at module level. The embodiment imports it before
the simulator starts. It can import from its own folder: the task's directory is put on `sys.path`.

```python
"""What is on the table, for the log and for text-reading policies."""
import math
from yam_pair.task import Box, Cylinder, Sphere, Asset, UsdBody, Shape, Layout, TABLE_TOP

LAYOUT = Layout("facing", separation=0.8)        # optional (this is the default)
TABLE = {"size": (1.30, 1.35), "center": (0.0, 0.0)}  # optional; default covers both stations; None = no table
PHYSICS_HZ = 240                                 # optional (default 240; must be a multiple of control_hz)
OBJECTS = [Box("tray", pos=(0, 0, TABLE_TOP + 0.003), size=(0.19, 0.40, 0.006), color=(0.7, 0.7, 0.7)),
           Box("part", pos=(0.025, 0.12, TABLE_TOP + 0.02), size=(0.04,) * 3, mass=0.06)]
INSTRUCTIONS = {"a": "...", "b": "..."}          # optional; otherwise task.json "instructions" (see §3)
SUCCESS_HOLD_S = 1.0                             # optional
SCORE_OVER = "end"                               # or "best"

def score(objects, pair) -> float: ...           # 0..1 partial credit -> info["score"]
def success(objects, pair) -> bool: ...          # held SUCCESS_HOLD_S -> terminated, "success"
def distance(objects, pair) -> float: ...        # -> info["distance"]
def report(objects, pair) -> dict: ...           # extra numbers -> info["report"] (e.g. collision_free)
def demo(layout) -> dict: ...                    # scripted plan for both stations (see §5)
```

The scoring hooks can take one argument, `objects`, as in yam_arena, or two, `objects` and `pair`. `demo` and
`isaac_objects` can take no argument or `layout`.

### Objects (world frame)

| Spec | Notes |
|---|---|
| `Box`, `Cylinder`, `Sphere`, `Asset` | yam_arena's task-file objects, unchanged. With `mass` the object is free; without it the object is fixed. `Asset.xy` is in world coordinates and the asset rests on the table. |
| `UsdBody(name, usd="assets/x.usda", pos, rot_xyzw, mass, articulation, joint_pos, drives, friction, ...)` | scenekit's USD body, with the same fields. The path is relative to **the task file's folder**. |
| `Shape(name, kind="box"/"capsule", pos, rot_xyzw, size/radius/height, collide)` | scenekit's posed fixed shape. |
| `def isaac_objects(layout) -> list` | escape hatch, called after Isaac has started. It returns raw `isaaclab_arena` `Object`s, for physics properties the specs above can't express. Their names appear in `objects` with bounding-box states. |

Reserved names, which objects must not use: `floor`, `table`, `light`, `dome_light`, `lamp_a`, `lamp_b`,
`{a,b}_{left,right}_robot`, `{a,b}_{top,left,right}_cam`, `overview_cam`, `{a,b}_{left,right}_contacts` and
`{a,b}_stand_*`. Each station has its camera stand behind its arms, as on Dream's station.

`objects[name]` is yam_arena's `ObjectState`, with `pos`, `quat_xyzw`, `lin_vel`, `ang_vel`, `low`, `high` and
`resting()`. All values are in the world frame. `pos` is the bounding-box centre. For a `UsdBody` it is a `BodyState`
with the body origin and a `joints` dict.

### `pair` (a `PairState`, built after every control step)

| Field | Meaning |
|---|---|
| `step`, `t` | control steps and simulated seconds since the reset |
| `contact_n` | largest cross-station normal force in this control step (N); every one of the 12 physics substeps is read |
| `contact_peak_n` | largest cross-station normal force since the reset (N) |
| `contact_by_arm` | `{"a_left": N, "a_right": N, "b_left": N, "b_right": N}`: each arm against any link of the other station, this step |
| `arms` | `{"a": 14-D, "b": 14-D}` measured joint state in each station's own layout |
| `layout` | the `Layout` |
| `tcp(station, side)` | world position of that arm's tool point, from the measured joints and URDF FK |
| `env` | the Isaac Lab env, for advanced use; it is `None` in pure-Python tests |

"Cross-station contact" means a contact between any link of one station's arms, fingers included, and any link of
the other station's arms. It is measured with Isaac contact sensors that are filtered to the other station's links.
It does not include arm-object, object-object or arm-table contact.

### Isaac-side hooks (optional; each takes `env`, the Isaac Lab env)

The reset puts every object back at its spawn pose. For a `UsdBody` articulation, the embodiment also writes its
`joint_pos` at rest, with matching drive targets, on every reset; scenekit leaves that to a `before_reset` hook.
Anything else a hook changes, such as joint limits or drive gains, stays changed until a hook sets it back. Hooks can
use `from yam_pair.sim import write_joint_positions, set_joint_targets`, which work like scenekit's and take any
articulation by name.

`on_build(env)` runs once after the scene is built, for example to add visual prims to the stage.
`before_reset(env)` runs at the start of every reset. `on_reset(env)` runs after the reset has settled.
`on_step(env)` runs after every control step, before scoring. A task that imports a copy of scenekit gets its
`@on_step`, `@on_reset` and `@before_reset` hooks called as well. `scenekit.robot_links` does not apply here: it reads
`left_robot`. Use `pair.tcp()` or `env.unwrapped.scene["a_left_robot"]` instead.

## 3. Instructions and task.json

Keep `task.json` next to `task.py`. Use the usual Robocurve fields, and add the per-station instructions:

```json
{ "instruction": "Robot A: ... Robot B: ...",
  "instructions": {"a": "Put both camera modules ... orange tray.", "b": "Put both servo motors ... blue tray."},
  "...": "title, category, keywords, context, setup, scoring, run_limit, ..." }
```

`instruction` is the combined text that the trial is logged under, so pass it as `--instruction`. Each station's
policy receives only its own instruction, resolved as follows:

| Station | Resolution, first match wins |
|---|---|
| A | `-P instruction_a=...`; if `-P instruction_b` was given, then `--instruction`; then `INSTRUCTIONS["a"]` or `task.json` `instructions.a`; then `--instruction` |
| B | `-P instruction_b=...`; then `INSTRUCTIONS["b"]` or `task.json` `instructions.b`; otherwise an error |

This gives two ways to run: from the task (the default), or ad hoc with `--instruction "<A's>" -P instruction_b="<B's>"`.
The resolved pair is recorded in the policy transcript, under `instructions`.

## 4. Embodiment `yam_pair`: interface

`-E` arguments:

| Argument | Default | |
|---|---|---|
| `task` | required | path to the task file |
| `layout` | task's `LAYOUT`, else `facing` | `facing` or `side_by_side` |
| `separation`, `offset` | per layout | metres |
| `yaw_a`, `yaw_b` | 0 | degrees |
| `control_hz` | 20 | control rate; physics is `PHYSICS_HZ` (240) with `PHYSICS_HZ/control_hz` substeps and one render per step |
| `cam_width`, `cam_height` | 320, 176 | policy image size; cameras render at 640×360 as in yam_arena, then `cv2.resize` |
| `object_jitter` | 0 | metres; with a seed, free objects are shifted in xy |
| `overview` | `false` | adds `overview_cam`, a third-person view, to the observation, so it is stored with the frames. **No station policy ever receives it.** |
| `overview_width`, `overview_height` | 640, 400 | |
| `headless`, `device` | `true`, `cuda:0` | |

**Observation.** The images are `a_top_cam`, `a_left_cam`, `a_right_cam`, `b_top_cam`, `b_left_cam` and
`b_right_cam` (plus `overview_cam`). The state is `a_joint_pos` (14) and `b_joint_pos` (14), each
`[left j0..j5, left grip, right j0..j5, right grip]` with joints in rad and grippers from 0 (closed) to 1 (open).
`instruction` is the scene's instruction.

**Action.** 28-D: `[A's 14-D | B's 14-D]`, absolute joint targets, clipped to the URDF limits. The first 14 values go
only to A's two arms and the last 14 only to B's.

**Per-step `info`.** `score`, `success` and `distance` come from the hooks, as in yam_arena. `cross_contact_n` and
`cross_contact_peak_n` are always present. `report` holds the dict that `report()` returns.

**Trial summary.** Inspect's eval log keeps only score values, not step `info`. For that reason both pair policies
write a summary into `record.metadata["yam_pair"]`, which appears in the eval log under `samples[].trial_metadata`.
The summary has `steps`, `termination_reason`, `final_score`, `final_report`, `cross_contact_peak_n`,
`steps_over_threshold` (0.5 N), `first_step_over_threshold`, `step_wall_s` (mean, median, p95, max),
`sim_step_s_mean`, `instructions` and `layout`. The per-step series (score, cross-station contact, report) goes to
`<log-dir>/yam_pair/<run>_<scene>-e<epoch>.json`.

**Information boundary.** Inspect Robots runs one policy per trial, so the embodiment returns every camera. The pair
policies split the observation with `yam_pair.boundary.station_observation(obs, station, instruction)`. That function
returns a standard single-station YAM observation: `top_cam`, `left_cam` and `right_cam` at 320×176, state
`joint_pos` (14), and the station's own instruction. The station policy sees nothing else: not the partner's images,
state, actions or instruction, and not `overview_cam`. A custom pair policy must use the same function.

## 5. Scripted demo: `demo(layout) -> {"a": X, "b": Y}`

Each station's value is one of the following. Positions and yaws are in **that station's frame**; use
`layout.to_local(s, world_xyz)`.

1. **An open-loop plan.** `{"left": [Waypoint, ...], "right": [...]}`, using yam_arena's `Waypoint` and
   `pick_place_waypoints`, re-exported from `yam_pair.task`. An arm without a plan holds READY.
2. **A closed-loop controller.** An object with `act(view) -> 14-D` and an optional `finished` attribute. `view` has
   `station` (also `agent`), `t` (also `time`), `state` (measured 14-D), `layout` and `objects()`. `objects()`
   returns privileged world-frame object states and is meant for scripted demos only.
3. **A generator script.** `script(ctx)`, in the style of demokit: `ctx.left` and `ctx.right` are `yam_pair.demo.Arm`s
   in the station frame, with `move`, `move_joints`, `set_grip`, `go_home` and `command`. The context also has
   `ctx.states()`, `ctx.state`, `ctx.t`, `ctx.station`, `ctx.layout`, `ctx.local(p)` and `ctx.world(p)`. The script
   yields once per control step. `yam_pair.demo.together` and `yam_pair.demo.wait` are also available.

`yam_pair_scripted` stops the trial when both stations are done, after `-P hold_s` seconds (default 3), unless
`success()` ended it earlier.

## 6. Policies

```
-P instruction_a=... -P instruction_b=...   # see §3
-P model=dreamlabs/dream-1-beta  -P rows_per_request=24  -P timeout_s=300  -P record=false  -P dry_run=false
-P only=a|b      # control run: only that station gets a Dream session; the other holds its reset pose
```

`dream_pair` creates two `inspect_robots_dream.DreamPolicy` objects. Each has its own `DreamSession`, its own client,
its own 9-frame window and its own instruction. Both are asked for each control step concurrently, in two threads,
and act on the same simulator step. Each one keeps Dream's 32-step pre-roll. A refusal or error from either session
raises `RuntimeError("station B Dream session: <reason>")`. Inspect then records it as a `PolicyError` and marks the
trial as errored, with its partial steps saved, exactly as for `dream`. Both sessions are closed afterwards. The
transcript holds `{"instructions", "a": <dream transcript>, "b": <dream transcript>, "session_ids", "session_audit",
"error"}`.

**`rows_per_request` defaults to 24, not `dream`'s 48.** Dream evicts a session that has been idle for about 8–10 s and
starts its episode memory over. `inspect_robots_dream` retries on the new session, so the run continues, but the model
has forgotten the episode. A four-arm step takes about 0.13–0.23 s, so 48 rows leave each session idle for about 10 s
between requests. In a 48-row run, 3 of 8 requests hit an expired session. With 24 rows the idle time is about 5 s.
`record.metadata["yam_pair"]["dream"]["sessions"]` counts `session_restarts` per station, and a warning is printed
when any occur.

## 7. Commands

```bash
source <ARENA_DIR>/yam.env
INSTR="$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')"
# scripted demo
inspect-robots run --embodiment yam_pair --policy yam_pair_scripted -E task=task.py -E overview=true \
    --instruction "$INSTR" --scorer pair_task_score --max-steps 1800 --store-frames --log-dir logs/scripted
# Dream-1 on both stations
inspect-robots run --embodiment yam_pair --policy dream_pair -E task=task.py -E overview=true \
    --instruction "$INSTR" --scorer pair_task_score --max-steps 1800 --store-frames --log-dir logs/dream
# video: 2x3 grid (A on top, B below; top/left/right), with the overview beside it when present
yam-pair-video logs/dream dream.mp4          # [--every 2] [--no-overview] [--trial scene-0-e0]
```

`yam-pair-video` takes a log directory, a frames directory or an eval log JSON.

## 8. Example

`examples/shared-kitting/` is an example task written to this contract. `task.py` defines the trays as
`Box`es, the four parts through `isaac_objects()` and the product detail through
`on_build`. It scores 0.25 per part in its tray, and `report()` gives `collision_free` with a 0.5 N threshold.
`task.json` holds the per-station instructions, and `scripted.py` is a closed-loop controller (§5.2).

## Changes

- v0.1.0 (2026-11-21): first version. `dream_pair` defaults to `rows_per_request=24` (§6). `UsdBody.joint_pos` is
  restored on every reset (§2). Trial summary in `trial_metadata` (§4).
