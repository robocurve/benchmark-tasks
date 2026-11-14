# Put a quick-release toilet seat back on its two hinge posts and click both buttons

Instruction given to the policy: `Put the toilet seat back on.`

The steps it leaves unstated: lift the back of the seat clear of the post tops, bring both sockets over their posts
together (cocked, one socket goes on and the other sits on its post or jams), lower them onto the spring buttons and
push each socket down until its button clicks, then let go. Partial credit follows the real stages: 0.2 for each
socket on its post and 0.3 for each socket clicked on, so a seat left resting on both buttons scores 0.4 and one
clicked on one side only (the other socket resting on its button) scores 0.7. The seat ring turns on the hinge piece
that carries the sockets, so once fitted it lifts on its hinges.

Files: `task.json`, `toilet_seat_quick_release_hinge.py` (scene, `success()`, `score()`, the scripted `demo()` and an
offline self-test), `mechkit.py` (joints added to the stock runtime in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/toilet-seat-quick-release-hinge/toilet_seat_quick_release_hinge.py
```

Expected:

```
scoring: 16 cases pass
left: 18.0 s plan, worst IK 0.090 mm / 0.013 deg
right: 18.0 s plan, worst IK 0.077 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/toilet-seat-quick-release-hinge/toilet_seat_quick_release_hinge.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the toilet seat back on." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/toilet-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step. A click is read from
the button cap's height (its bounding box), and only counts when the socket itself pressed it (post inside the bore).
Stillness is read from how far the sockets and the seat's front corners moved over the last 20 Hz control step.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (log dir `~/yam-eval/toilet-demo`). Open loop and deterministic, an 18 s
plan. Each arm pinches one socket housing across its sides, lifts the back of the seat 4.5 cm (the front stays on the
rim and the seat turns in the pinch), carries it 4.5 cm back over the posts, lowers both sockets onto the buttons
together, pushes both down until they click, lets go and backs off. `TOILET_DEBUG=1` prints the buttons, sockets,
seat swing angle and score every 20 steps (`TOILET_DEBUG=2` adds a line every step).

Degenerate and check runs (same command, `DEMO_VARIANT` in the environment):

- `unlatched`: sets both sockets on their posts and lets go without pushing; the seat rests 12 mm high on the
  buttons. Partial: 0.4.
- `one_side`: sets both sockets on their posts; the right arm holds its side up while the left arm pushes its
  socket down until it clicks, then both let go. The right socket stays on its unclicked button. Partial: 0.7.
- `lift_seat` (check, not degenerate): the full demo, then the left arm pinches the seat's left rail and swings the
  seat up on its hinges and back down while the right arm still holds its socket; then both let go. 1.0.

## Videos and screenshot

```bash
inspect-robots video ~/yam-eval/toilet-demo/*.json
ffmpeg -y -i ~/yam-eval/toilet-demo/frames/<run>/*top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout (`-E object_jitter=0`); the demo is deterministic; Dream-1 is hosted and not seeded.
- A trial takes roughly 5 to 7 minutes of wall clock on an L40S, including Isaac Sim warm-up.

## Reset between trials

Lift the seat off the posts, let both buttons spring back up, and lay the seat flat on the rim with its back edge
4.5 cm forward of the posts.
