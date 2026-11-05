# Open a sliding door jammed by a block in its track

A clear acrylic door, 18 cm wide, stands in a straight 60 cm aluminium bottom track on the table, closed against the
track's right end stop. A small wooden block stands in the left end of the same track, against the left end stop.
Slid to the left, the door stops 8 cm short of the end stop however hard it is pushed. The instruction does not
mention the block: the robot has to notice why the door stops, lift the block out of the track, and then finish
opening the door. Forcing the door derails it, which scores 0.

**Instruction:** `Slide the door to the left along its track until it touches the track's left end stop. Keep the door in its track and everything on the table.`

## Why this task

Sliding doors on wardrobes, display cases, shower enclosures and machine guards usually jam because something has
fallen into the bottom track. Pushing harder lifts the door out of its track or bends the track; the right response is
to stop, find what is in the way, take it out and then open the door. The task tests whether a policy notices that
the door did not reach where it was asked to go, works out why from what it can see, and changes its plan instead of
pushing on.

It also needs both arms in turn. The track is longer than one arm can cover: only the left arm reaches the block and
the open position, and only the right arm reaches the closed door. A policy has to clear the track with one arm, start
the slide with the other and finish it with the first.

The scoring separates three behaviours that a progress-only measure would mix up: forcing the door against the block
(0.25 at most), clearing the track (0.5), and clearing it and then opening the door fully (1.0).

## Design

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y left). The arm bases are at
y +-30.5. Left and right are the robot's: the left arm (+y) is on the left of the top camera's image. Both arms start
in the ready pose, grippers open.

| Object | Size | Mass | Notes |
|---|---|---|---|
| Track base | 60 x 4 cm, 0.6 cm thick | fixed | Aluminium strip along y at x 25, from y -41 to +19 (its middle 11 cm right of the midline) |
| Track lips | 60 cm long, 0.4 cm thick, 1.5 cm tall | fixed | Two aluminium lips on the base, leaving a 1.3 cm wide channel |
| End stops | 1 cm long, 2.1 cm wide, 1.5 cm tall | fixed | Aluminium, one at each end; inner faces at y -40 (right) and +18 (left) |
| Door | 18 cm wide (y), 12 cm tall, 1 cm thick | 257 g | Loose clear acrylic, standing upright in the channel against the right end stop: y -40 to -22. Rendered opaque light blue in simulation |
| Block | 8 cm long (y), 4.5 cm tall, 1 cm thick | 18 g | Loose pine, painted red, standing on its long thin edge in the channel against the left end stop: y +10 to +18. It sticks up 3 cm above the lips |

Nothing else is on the table. The track guides the door by contact alone: there is no slide joint, roller or top
guide, and the 1.5 cm lips keep it upright.

## Scoring

Measured at the end, with a tape measure and a photo. The block counts as *on the table* when it lies on the table or
the track with its bottom less than 3 cm above the table top: not on the floor, and not held up.

| Score | When |
|---|---|
| 0 | The door is out of its track: bottom edge more than 0.5 cm above or below the channel floor, centre more than 0.6 cm off the track's centre line, or leaning more than 10 degrees |
| 1.0 | The door's left edge is within 2 cm of the left end stop, the block is on the table, and both are still for 1 s |
| 0.5 | The door is open that far but the block is not on the table or something is still moving; or no part of the block is left in the track between the door's left edge and the left end stop (below the height of the door's top edge), and the block is on the table |
| 0.25 | The door's left edge has moved at least 10 cm to the left |
| 0 | Otherwise |

In the simulation, `score()` is evaluated at the end of the episode, and `success()` (the 1.0 conditions held for
1 s) ends the trial early with `task_score` 1.0.

With the block in place, the door's left edge can get no closer than 8 cm to the left end stop, so a full score needs
the block taken out. Where on the table the block ends up is not scored. Only the end state is scored: a door lifted
out of the track and set back in it at the left end counts the same as one slid there, since the block still has to
be out of the way. What other strategies score:

- Doing nothing: 0.
- Pushing the door against the block: 0.25 at most (its left edge moves 32 cm and stops 8 cm short of the end stop).
  The forcing control below scores 0.25.
- Forcing the door until it climbs out of the channel or leans over: 0.
- Lifting the block out and setting it on the table, without moving the door: 0.5.
- Knocking the block off the table: no credit for clearing the track; 0.5 at most, with the door fully open.
- Knocking the block over so that it lies across the track at the left end: it still blocks the door, so 0.25 at most.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the door to the left along its track until it touches the track's left end stop. Keep the door in its track and everything on the table." \
  --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/sliding-door-dream
```

`task_score` prints the trial's score at the end (`task_score: ...`): 1.0 when the episode ends in success, otherwise
`score()` at the end of the episode.

Videos (one MP4 per camera) and the setup screenshot (the top camera's first frame):

```bash
for log in ~/yam-eval/sliding-door-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/sliding-door-dream/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the door to the left along its track until it touches the track's left end stop. Keep the door in its track and everything on the table." \
  --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/sliding-door-demo
```

`demo()` is open loop: it uses the known starting layout, like the walkthrough's own demos, and does not react to the
door stopping. The arms work in turn, because neither reaches the whole track:

1. **The block (left arm, 0 to 9.6 s):** the left arm, the only one that reaches the block, pinches it across its 1 cm
   thickness above the lips (grip turned 85 degrees), lifts it straight out of the channel and stands it on the table
   at (16, 40) cm, behind the track.
2. **Most of the slide (right arm):** the right arm has been waiting above the door's left end. Once the block is
   carried clear of the track, it pinches the door's top edge 1.5 cm in from its left end, lifts it off the channel
   floor (10 mm commanded; the door rises less, because the arm gives a little under its weight, and stays well down
   between the lips), slides it 26.4 cm to the left, as far as it comfortably reaches, sets it down, lets go and goes
   back up.
3. **The rest (left arm):** the left arm pinches the door at the same place, slides it the last 13 cm to 5 mm short
   of the left end stop, sets it down and lets go.

Each arm pulls the door by the end that leads. Pinched at its trailing end instead, the door tips: its leading bottom
corner drags on the channel floor and the door pivots up out of the pinch.

With `yam_arena`'s IK, the right arm cannot take the door to the end (27 mm of IK error with its pinch at the end
position) and the left arm cannot reach the closed door (93 mm of IK error at the door's start).

**Forcing control (expected score 0.25 at most):** the demo without clearing the track. The block stays in it, the
right arm slides the door to the middle as in the demo, and the left arm pulls it on into the block, commanding its
pinch 1 cm past the contact, and keeps pushing for 2 s before it sets the door down and lets go:

```bash
DEMO_FORCE=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the door to the left along its track until it touches the track's left end stop. Keep the door in its track and everything on the table." \
  --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/sliding-door-force
```

## Files

| File | What |
|---|---|
| `task.py` | The `yam_arena` task file: the objects, `success()`, `distance()`, `score()` and the scripted `demo()` |
| `test_sliding_door.py` | Offline checks with `yam_arena`'s loader and IK: layout arithmetic, scoring of hand-built states, the demo's reach and joint limits. They do not run the simulator |
| `task.json` | Submission details |

The task uses only `yam_arena`'s own `Box` objects in the station scene: no assets, no extra modules and no changes to
`yam_arena`. It runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Runtime and seeds

Run limit: 1,200 steps (1 minute at 20 Hz). Wall time per trial on one RTX workstation GPU (shared with other jobs),
including Isaac Sim's start-up: about 2.5 minutes for the scripted demo, which ends at success after 625 steps, and
about 12 minutes for a Dream-1 trial that runs to the step limit. Isaac Sim's very first start on a
machine also compiles its shaders, once: the first demo there took 8 minutes in all.

The layout is fixed with `-E object_jitter=0`: every object starts exactly where `task.py` puts it. There is no other
randomness and no seeds to choose.

## Tests

```bash
python -m pytest -q test_sliding_door.py     # in this folder, in the yam_arena environment; no GPU needed
```

## Verified in simulation

Isaac Lab-Arena 0.3.1 (Isaac Sim 6.1), with the commands above.

| Run | `task_score` | What the log and video show |
|---|---|---|
| Scripted demo | **1.0** | The left arm lifts the block out of the channel and stands it on the table; the right arm slides the door to the middle; the left arm finishes the slide. The door stays upright in its track throughout, and success, held 1 s, ends the trial after 625 steps (31 s) |
| `DEMO_FORCE=1` | **0.25** | The right arm slides the door to the middle; the left arm pushes it into the block and keeps pushing for 2 s. The door stops against the block, 8 cm short of the end stop, and stays upright in its track; the block stays in the track. The plan ends after 602 steps |
| Dream-1 (1,200 steps) | **0.0** | The left arm folds down beside its base within the first seconds and stays there: it never goes to the block or the track. The right arm lowers its gripper to the door's left end by about 10 s and hovers there to the end without taking hold of it. The door and the block do not move |
| `python -m pytest -q test_sliding_door.py` | 15 passed | |

Two runs of the scripted demo on the same machine gave identical top-camera frames: the demo is deterministic.

## Real-world build

- **Track:** an aluminium bottom track 60 cm long: a 4 cm wide, 6 mm thick base strip with two 4 mm thick, 15 mm tall
  lips leaving a 13 mm channel, and a 10 mm long end stop at each end. It can be made from aluminium flat bar screwed
  to a base strip. Clamp or screw it to the table along x 25 cm, from y -41 to +19 cm.
- **Door:** 10 mm clear acrylic (PMMA) sheet, 18 x 12 cm, about 257 g, edges sanded.
- **Block:** pine, 8 x 4.5 x 1 cm, painted red, about 18 g.
- **Setup:** stand the door in the channel against the right end stop, and the block on its long thin edge in the
  channel against the left end stop.
- **Reset:** nothing is consumed. Slide the door back against the right end stop and stand the block back against the
  left end stop.
- **Safety:** the door is acrylic, not glass, and every part is light; nothing is sharp, hot or under tension.

## Still approximate or unverified

- All loose objects use the simulator's one rubber-like friction (static 1.2, dynamic 1.0), well above acrylic on
  aluminium, so the simulated door drags harder than a real one.
- The door is rigid and opaque light blue in simulation; the real one is clear, which makes it harder to see.
- The track is fixed boxes: no rollers, no top guide, no wear or bending. Forcing the door cannot bend the track in
  simulation; it can only derail the door, which the scoring catches.
- No trials have been run on a real robot.
- One Dream-1 run (1,200 steps) scored 0: it never went to the block, and the door did not move.

## License

MIT.
