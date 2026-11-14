# Hex-dog a panic exit bar

Instruction given to the policy: `Set this exit bar so the door stays unlatched, and take the key out.`

The scene is the lock-edge corner of a closed door and its frame, laid flat on the bench push face up. A rim
exit device is on the door; its latch bolt sticks 21 mm out past the door edge into a rim strike on the frame jamb,
so the door is latched. Drawing the bolt in 18 mm or more puts its end clear of the strike and frees the door.

The instruction leaves the method unstated. One hand holds the push bar fully down (25 mm, against a 40 N return
spring) while the other puts the short leg of the 4 mm hex key into the 6 mm dogging hole on the mechanism case
and turns it a quarter turn against its detent. The dogging cam only catches the bar if the bar is down 23 mm or
more; otherwise the cam springs back to 0 when the key is let go. The cam has a hard stop at 94 deg, so it cannot
be turned past dogged. Then the bar is let go (it stays down, latch
drawn in), the key is drawn out of the hole and laid on the door, frame or bench. A dogged bar with the key left in the hole scores 0.75.

Files: `task.json`, `panic_bar_hex_dogging.py` (the scene, `success()`, the partial-credit `score()`, the scripted
`demo()` with its degenerate variants, and an offline self-test) and `mechkit.py` (joints, sliders and catches
added to the stock runtime in memory).

## Environment

This runs on the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and
inspect-robots as it installs them. Run `source ~/IsaacLab-Arena/yam.env` before every shell. The Dream key is in
`~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/panic-bar-hex-dogging/panic_bar_hex_dogging.py
```

This asserts the scoring rules on hand-built states and replays every demo variant through the kinematics at
20 Hz, checking joint limits and IK error. It prints `scoring: 15 cases pass` and one line per arm and variant.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/panic-bar-hex-dogging/panic_bar_hex_dogging.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set this exit bar so the door stays unlatched, and take the key out." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/panic-bar-dream
```

`task_score` is 1.0 when `success()` has held for 1 s. Otherwise it is `score()` at the last step.

## Run and score the full-score demo

Run the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/panic-bar-demo`. The demo is open
loop and deterministic. The left arm presses its closed fingers down on the bar and holds it at the bottom of its
travel. The right arm pinches the key's long leg 2.5 cm out from the short leg, lifts it out of its stand, lowers the short
leg into the dogging hole and turns it a quarter turn (wrist from -45 to +45 deg while the hand follows the arc).
The left arm lets go (the bar stays down), then the right arm draws the key straight out and lays it on the door.

Degenerate checks use the same command with `DEMO_VARIANT` set in the environment:

- `hold_only` (scores 0): the left arm holds the bar down to the end of the run; the key is never touched.
- `no_bar` (scores 0): the bar is never pressed; the key is turned a quarter turn, let go (the cam springs back),
  then taken out and laid down.
- `key_left` (scores 0.75, stage credit): everything in order and the bar is dogged, but the key is left in the
  dogging hole.
- `overturn` (scores 1.0): the full demo, but the wrist tries to turn the key 30 deg past dogged; the cam stays at
  its hard stop (94 deg; at most 98 deg was measured while the wrist forces it) and the job finishes normally.

`PANIC_DEBUG=1` prints the state every 20 steps.

## Videos and screenshot

```bash
inspect-robots video ~/yam-eval/panic-bar-demo/*.json
ffmpeg -y -i ~/yam-eval/panic-bar-demo/frames/<run>/*top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz). The demo plan is about 33 s long.
- One fixed layout (`-E object_jitter=0`), so the seed does not change the start. The demo is deterministic.
  Dream-1 is hosted and not seeded.
- A trial takes roughly 5 to 8 minutes of wall clock on an L40S, including Isaac Sim warm-up.

## Reset between trials

Hold the bar down, put the key in the hole and turn the cam back to 0, let the bar up (the bolt goes back into the
strike), and stand the key back in
its stand with its long leg pointing forward-right.
