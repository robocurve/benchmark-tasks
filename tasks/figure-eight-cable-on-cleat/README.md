# Lay a figure-eight round twin posts

A twin-post cleat (a pair of bitts: two grey 30 mm posts with 42 mm round heads, 8 cm tall, 8 cm apart) stands on
the table in front of the right arm. A 60 cm dark-blue 8 mm rope comes out of a fixed grey block on the left and lies
straight on the table in front of the posts. The robot must lay one full figure-eight round the posts, as a deckhand
starts belaying a line: around the outer side of the right post, diagonally back between the posts, around the outer
side of the left post, and diagonally back between the posts again, crossing the first diagonal; then let go. The
goal is topological: the rope must turn one way around one post and the opposite way around the other, in that
order, which no single pick-and-place of the rope end achieves. The right post must be done first, as the
instruction says, and the score checks it. The rope must pass round the posts, not over their
heads.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Make one full figure-eight with the blue rope around the two grey posts: take it around the outer side of the right post, back diagonally between the posts, around the outer side of the left post, and back diagonally between the posts again so it crosses the first diagonal. Keep the rope low on the posts, below their heads, then let go of it."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed-loop grasp (reads the live rope): the right arm takes the rope 10 cm from its free end and winds it with
the held point: a low circle (6 cm radius) round the outer side of the right post, then it lifts over the post heads
and lowers in front of the left post, circles its outer side the other way, lifts over again and lowers in front of
the right post, so the rope crosses between the posts at table level and the second diagonal lies over the first.
It lays the tail on the table and lets go. Success holds for 1 s and ends the episode at step 384 (~19 s);
task_score 1.0 (reproduced on two machines). `demo/` holds its videos, `setup.png` and `result.json`. The left arm
is not needed: the anchored block holds the standing part.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless the rope is released: both grippers' centres at least 3 cm from every part of the rope.
- The cleat line runs through both post centres. Following the rope from the block to its free end, every place
  where it crosses that line is recorded, with its direction, if the rope there is at least 1 cm below the post
  heads (at most 6 cm above the table): *outer A/B* = beyond the left/right post, within 4.5 cm of its centre; *middle* = between the post
  centres; crossings farther out are ignored.
- Right post first: the FIRST crossing along the rope (from the block) must be *outer B* (the right post),
  otherwise n = 0. Because the standing part is fixed at the block and the rope is laid from there outwards, the
  order along the rope is the order in which the posts were wrapped.
- n = the run of crossings from that first crossing on that alternates middle / outer, the outer crossings
  alternating between the posts, all outer crossings in one direction and all middle ones in the other (= opposite
  turns around the two posts, the figure-eight).
- n 0-2 = **0** (nothing done, draped over or across the posts, wound round one post only, looped round both
  posts together, started round the left post); n 3 = **0.5** (round the right post, between the posts, round the
  left post); n >= 4 = **1.0** (back
  between the posts again: one full figure-eight).
- `success()` additionally requires the rope to be at rest. `distance()`: 4 - n (crossings still missing).

Cheat / failure variants (test only, `FIG8_CHEAT=<name> python cabledemo.py task.py out`), all score exactly 0:
`none` (do nothing), `drape` (lays the rope straight across the post heads and lets go: n = 1), `oneh` (winds
round the right post twice, never the left: n = 1), `oval` (one loop round both posts together, same direction:
n = 1), `hold` (the full figure-eight, n = 4, but the gripper keeps holding the rope: not released), `leftfirst` (a full
figure-eight started round the left post: crossings outer left, middle, outer right, middle, a complete figure-eight in the wrong order; first crossing is not outer right, so n = 0).

Partial tier, `FIG8_CHEAT=tier05`: round the right post, between the posts, round the left post, then the tail is
laid down behind the left post and let go: n = 3, score 0.5.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). The demo needs 384 steps. One trial takes ~3-5 min wall time on one GPU including the
  ~1-2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the rope is never jittered (it always starts straight), so every trial
  starts identically.
