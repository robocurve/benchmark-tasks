# Fit a condenser to a still head and clip the joint, holding the condenser in line while the other hand snaps the clip on

A distillation set-up stands on a lab stand: a round-bottom flask in a cork ring with a still head on it. A clamp holds
the still head, whose side arm reaches horizontally toward the right and ends in a ground-glass cone. A glass Liebig
condenser lies on two cork cradles in front of it. A yellow plastic joint clip stands on the table on the left.

One hand lifts the condenser and pushes its ground-glass socket onto the side arm's cone, and keeps holding it level
and in line. The other hand takes the clip and pushes it down over the joint until it snaps on. Then both hands let go,
the condenser held by the clipped joint, and both arms go back to where they started.

The task is one task file,
[`clip_a_condenser_onto_a_still_head.py`](clip_a_condenser_onto_a_still_head.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

**What it tests:**
- aligning a long part on a joint and holding it in line against its own weight;
- placing a small fastener over the joint with the other hand;
- the right order: seat the joint first, then clip it;
- finishing with both hands free.

**Why two hands:** the socket fits loosely on the cone, so an unclipped condenser sags off line as soon as it is let
go. The clip only snaps on while the condenser is seated in line, and it can't be pushed on by the hand holding the
condenser. The one-hand check scored 0: the right did its part as in the demo while the left never moved.
The right let go once the socket was on, and the condenser slid back off the cone and swung down 15 degrees, which counts
as dropped.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres.
- The set-up (stand, cork ring, flask, still head, side arm and cone) is one kinematic body made of shapes from
  `bodykit.py`, with low glass friction.
- The condenser and the clip are rigid bodies. The condenser's socket is a ring of plates 3.5 mm clear of the cone all
  round, with a slightly flared mouth; the cone has a short narrower nose.
- **The snap:** after every control step, the task file checks two things. The condenser must be seated: its rim
  within 6 mm short of or 10 mm past the cone's shoulder, its axis within 5 mm and 6 degrees of the cone's. The clip
  must be over the joint: within 8 mm across and 10 mm along it, its spine within 5 mm of the socket's top, and
  upright. Once both hold, two fixed joints (`Glue` from `bodykit.py`, to a hidden fixed body under the table) switch
  on. They hold the condenser square on the cone and the clip at its seat.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s; `success()` means full score (which
includes both arms back where they started) with the condenser and the clip still. Otherwise it gives `score()` at the
end of the episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clip_a_condenser_onto_a_still_head.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the glass condenser to the still head: with one hand lift the condenser off its cradles and push its ground-glass socket all the way onto the cone at the end of the still head's side arm, keeping it level and in line, and hold it there; with the other hand take the yellow joint clip and push it down over the joint until it snaps on. Then let go of both, leaving the condenser held by the clipped joint, and take both arms back to where they started. Do not drop the glass." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/condenser-dream
```

The scripted demo reaches full score:
1. The right hand comes straight down on the condenser at its balance point, 15.5 cm from its socket, fingers closing
   across it (turned 4 degrees against the twist the slanted finger pads give it), and lifts it
   off the cradles.
2. The right carries the condenser up in line with the side arm (aiming 8 mm high, against the arm's sag), pushes its
   socket onto the cone and holds it there.
3. The left hand comes down on the clip's tab, fingers closing across it, and lifts the clip.
4. The left carries the clip along above the side arm to above the socket, just behind its mouth, and pushes it down
   over the joint until it snaps on.
5. Both hands let go, and both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clip_a_condenser_onto_a_still_head.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the glass condenser to the still head: with one hand lift the condenser off its cradles and push its ground-glass socket all the way onto the cone at the end of the still head's side arm, keeping it level and in line, and hold it there; with the other hand take the yellow joint clip and push it down over the joint until it snaps on. Then let go of both, leaving the condenser held by the clipped joint, and take both arms back to where they started. Do not drop the glass." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/condenser-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/condenser-dream/*.json; do inspect-robots video "$log"; done
```

Set `CLIP_DEBUG=1` to print what the score is built from twice a second:
- where the condenser's socket rim is and how far it droops;
- whether it is seated, or has been;
- where the clip is against its seat, and whether it has snapped on;
- whether the condenser was dropped;
- where each TCP is against the clip's tab and the condenser's grip point;
- whether both arms are home.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 900 steps (121 s on the clock, with Dream answering in 0.78 s on average), and about 2 minutes for the scripted demo (376 steps, 18.8 s of simulated time, 107 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
