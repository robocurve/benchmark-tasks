# Load four weights onto a balance plank in matched pairs so that it never tips

A loose wooden plank rests level across a narrow steel pivot ridge, like the beam of a balance. Four weights of the
same size stand on felt pads around the table in no particular pattern: two red ones of 400 g and two yellow ones of
250 g. All four have to end up on the plank, one of each colour on each side, the red ones near the ends and the
yellow ones closer in, without the plank ever tipping or coming off the ridge. The plank stays level only while the
centre of everything on it is within about 1.2 cm of the ridge, so the weights can only go on in balanced pairs, set
down at the same moment at mirrored distances: the two red ones together, then the two yellow ones together. A weight
on its own, or a red and a yellow one set down opposite each other, tips the plank. The whole task (scene, scoring and
the scripted demo) is one task file, [`balance_plank_matched_pairs.py`](balance_plank_matched_pairs.py), for the
`yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes and cylinders, so it needs
no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise `score()` at the end of the episode
(0.5 or 0). The tip and ridge rules hold for the whole trial and are latched in the task file's module state: once
the plank tilts more than 6 degrees, or its centre moves more than 2 cm from the ridge or is lifted 1 cm off it, the
trial scores 0. Run one trial per process (as below) so this state starts fresh. `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout (the plank is loose too, and
any jitter would start it off the ridge).

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=balance_plank_matched_pairs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put all four weights on the plank, one of each colour on each side: the red ones near the two ends and the yellow ones closer in. Don't let the plank tip or come off its pivot." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/plank-dream
```

The scripted demo, which reaches full score (two rounds: each gripper picks up the red weight on its own side, both
carry them above the end zones, lower them onto the plank on the same timeline and let go together; then the same
with the yellow weights and the inner zones):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=balance_plank_matched_pairs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put all four weights on the plank, one of each colour on each side: the red ones near the two ends and the yellow ones closer in. Don't let the plank tip or come off its pivot." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/plank-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/plank-dream/*.json; do inspect-robots video "$log"; done
```

`BP_DEBUG=1` prints the plank's tilt (and the largest so far), whether it has tipped or left the ridge, and whether
each pair is in place, every 10 steps.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (290 s on the clock, with Dream answering in 0.77 s
on average), and under 1 minute for the scripted demo at 320 x 176 (564 steps, 28 s of simulated time; 1.5 minutes
when rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A 48 x 9 x 1.8 cm pine board (about 300 g) laid across a 2.4 cm wide steel or hardwood ridge 5 cm tall, screwed to
the bench; four cylinders 5 cm across and 6 cm tall (capped plastic tube filled with lead shot to the mass), two of
400 g painted red and two of 250 g painted yellow, on felt pads. Check the balance before each trial: the
plank should rest level and tip when a red weight is set on one end alone.
