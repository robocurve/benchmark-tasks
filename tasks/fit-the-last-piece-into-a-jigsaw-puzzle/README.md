# Fit the last piece into a jigsaw puzzle

A wooden jigsaw puzzle (1/4 in plywood) lies finished on the table except for one gap near its right-hand edge. The last piece lies
picture up beside it, turned a quarter turn from the way it fits. The robot puts the piece into its gap:

- It pinches the 6.35 mm piece by its edges right at the table, clear of the knobs.
- It turns the piece to match the gap: it fits only one way round, with three knobs and one hole.
- It brings the piece down squarely over the gap, so that all three knobs drop into their sockets at once and the
  neighbour's knob goes into the piece's hole. There is 0.5 mm of play all round.
- It presses the piece flat.

The scene, scoring and scripted demo are in
[`fit_the_last_piece_into_a_jigsaw_puzzle.py`](fit_the_last_piece_into_a_jigsaw_puzzle.py), for the `yam_arena`
embodiment. It uses the helper [`bodykit.py`](bodykit.py), kept next to it, which builds the piece and the puzzle round
its gap as rigid bodies made of many thin boxes. In plan view their outlines, round knob heads included, are built from
0.5 mm strips.

## Requirements

- [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at commit `0059d02`.
- Isaac Lab-Arena 0.3.1 and Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU.

The repository's `setup.sh`, or the program's Brev launchable, installs everything, including the Dream-1 policy. The
task uses only boxes, so it needs no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`. A trial scores 1.0 once `success()` holds for 1 s: the piece seated in its gap and still,
and the hands 3 cm or more from it. Otherwise the trial gets `score()` at the end of the run: 0.75, 0.5, 0.25 or 0.
`-E object_jitter=0` keeps every object where the task file puts it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fit_the_last_piece_into_a_jigsaw_puzzle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the last piece into the jigsaw puzzle." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/jigsaw-dream
```

The scripted demo, with the right hand:

1. Pointing down, it pinches the piece across two straight stretches of its edges, by two corners and clear of the knobs,
   with the fingertips down at the table, and lifts it 2 cm.
2. It turns the piece the quarter turn while carrying it over the gap.
3. It lowers the piece squarely over the gap until its underside is level with the top of the puzzle, and lets go. The
   piece comes to rest on the gap within about a millimetre, but not yet in it.
4. It presses the piece down with the closed fingertips, circling a little, so that the knobs find their sockets and the
   piece drops in flat. Then it goes back to its start.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fit_the_last_piece_into_a_jigsaw_puzzle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the last piece into the jigsaw puzzle." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/jigsaw-demo
```

Set `JIG_DEBUG=1` to print the piece's position, turn and tilt relative to its gap, and the right hand's position,
once a second (`JIG_DEBUG_N=10` prints every 10 steps instead of every 20).

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/jigsaw-dream/*.json; do inspect-robots video "$log"; done
```

## Runtime per trial

About 3.5 minutes for an 800-step Dream-1 trial on one NVIDIA L40S, about 1 minute of it starting Isaac Sim. The
scripted demo ends on success after about 340 steps and takes about 2.5 minutes.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the loose piece shifts.

## The real trial

Use a wooden jigsaw puzzle laser-cut from 1/4 in (6.35 mm) birch plywood, 30 x 30 cm in 10 x 10 classic knob-and-hole
pieces about 3 cm across. Finish it except for the second piece in from its right-hand edge, in the fifth row from the
robot, which has three knobs and one hole. Lay that piece picture up on a taped mark 9.5 cm to the right of its gap,
turned a quarter turn clockwise so its hole faces left. To score:

- Take a photo from straight above to check the piece's place and turn.
- Lay a straightedge across the piece and its neighbours: a seated piece leaves no gap under it.
