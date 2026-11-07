# Lace a board's next eyelet, handing the lace's tip through the hole from one hand to the other

An upright plywood lacing board stands on the table with two brass eyelets side by side. A polyester lace is knotted behind the first
eyelet, comes through it, hangs in a loop and ends in a long stiff plastic tip (an aglet) resting on two wooden pegs, pointing at the
second eyelet. One hand pushes the tip through the second eyelet and keeps hold of it; behind the board the other hand
takes the tip that comes through, the first lets go, and the second pulls the lace on through. The task is one task
file, [`lace_a_board_through_the_next_eyelet.py`](lace_a_board_through_the_next_eyelet.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: lining a tip up with a hole and pushing it through, and a handover through the hole between two hands
on opposite sides of the board.

Why two hands: the handover is the point of the task, and the score asks for it. From the moment the tip is out behind
the board until it is all the way through, it must not be let go of; so the hand that pushed it holds on until the
other hand has taken it. Pushed through, let go of and fetched from behind by the same hand, it scores 0.5 at most.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The board (with its foot, two eyelets made of panels
and a funnel in front of the second) and the two pegs are kinematic bodies, fixed where they are put; the board and
the eyelets are slick. The lace is a chain of 15 short rigid pieces, each on a ball joint (a `Ball` from `bodykit.py`)
to the next, the first carrying the knot; the tip is a rigid rod on a ball joint at the end of the chain.

The task file finds where the lace crosses the board's plane from the pieces' poses, and notes whether the tip was let
go of on its way through.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the lace still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lace_a_board_through_the_next_eyelet.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Push the lace's blue tip through the second eyelet with one hand and keep hold of it until the other hand has taken the tip behind the board; then let go, and with the other hand pull the lace on through until the whole tip is through." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/lace-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the tip near its back end, fingers
closing across it, lifts it just off its pegs and pushes it straight on through the second eyelet, about 4 cm out
behind the board. The right hand, level and pointing at the board from behind it, fingers closing across the tip,
takes it there; the left lets go and backs away, and the right pulls the lace on through, 8 cm:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lace_a_board_through_the_next_eyelet.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Push the lace's blue tip through the second eyelet with one hand and keep hold of it until the other hand has taken the tip behind the board; then let go, and with the other hand pull the lace on through until the whole tip is through." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/lace-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/lace-dream/*.json; do inspect-robots video "$log"; done
```

Set `LACE_DEBUG=1` to print what the score is built from (where the tip's point and back end are, whether the lace
goes through the second eyelet, whether the tip is in it, through it or pulled through, whether it was let go of on
the way, and where each gripper's TCP is from the tip) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (157 s on the clock, with Dream answering in 0.76 s on average), and about 1 minute for the scripted demo (404 steps, 20.2 s of simulated time, 62 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
