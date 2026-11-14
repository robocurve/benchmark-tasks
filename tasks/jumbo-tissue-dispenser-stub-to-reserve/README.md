# Restock a twin jumbo-roll tissue dispenser

Instruction given to the policy: `This dispenser is almost empty. Restock it with the new roll so the old one gets used up first, and lock it.`

The instruction leaves the order and the handling unstated. The dispenser is a twin jumbo-roll unit hanging in its
normal wall orientation on a vertical board at the back of the bench: back plate vertical, both spindles horizontal
and side by side, the cover hinged along its top front edge. Each spindle is a split hub that turns on a bearing, so a
loaded roll spins as tissue is pulled: a fixed upper lug the core hangs on and a sprung lower lug that presses it up.
The cover's front panel has a tissue outlet window over each roll along its bottom edge, and a red changeover
shutter with a thumb tab slides along the outside of the panel and covers one window. As found it covers the reserve
window, so tissue comes from the stub on the primary. Restocking so the old roll is used up first means: the stub
comes off the primary and goes onto the reserve spindle, the new roll goes onto the primary, the cover is closed until
its latch clicks, and the shutter is slid across over the primary window so only the stub's window is open. Each
sprung hub lets a core on or off only while its release button on the bottom lip is held, so one hand holds the
button while the other slides the core along the spindle. The cover is held open by a stay; pressing the stay button
lets it swing down under its weight, spring and damper onto the latch (closing it by hand is fine too). Closing it
over a core that is not seated scores 0, and so does slamming it: still closing at over 6 rad/s within 15 deg of
shut, an edge impact of over 1.8 m/s, well above a brisk hand close. The shutter is scored where it ends up, with the
cover latched.

Files: `task.json`, `jumbo_tissue_dispenser_stub_to_reserve.py` (the scene, `success()`, the partial-credit
`score()`, the scripted `demo()` with its degenerate variants, and an offline self-test) and `mechkit.py`
(joints, sliders, bonds and catches added to the stock runtime in memory).

## Environment

This runs on the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and
inspect-robots as it installs them. Run `source ~/IsaacLab-Arena/yam.env` before every shell. The Dream key is in
`~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/jumbo-tissue-dispenser-stub-to-reserve/jumbo_tissue_dispenser_stub_to_reserve.py
```

This asserts the scoring rules on hand-built states. It also replays every demo variant through the kinematics at
20 Hz and checks that each joint stays inside its URDF limits and the IK error stays under 0.5 mm. It prints
`scoring: 24 cases pass`, after also checking that hand closes at 0.3 to 5 rad/s pass, an 8 rad/s slam fails and a roll turned on its hub still scores 1.0, and then one line per arm and variant.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/jumbo-tissue-dispenser-stub-to-reserve/jumbo_tissue_dispenser_stub_to_reserve.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This dispenser is almost empty. Restock it with the new roll so the old one gets used up first, and lock it." \
  --scorer task_score --max-steps 2000 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/jumbo-dispenser-dream
```

`task_score` is 1.0 when `success()` has held for 1 s. Otherwise it is `score()` at the last step.

## Run and score the full-score demo

Run the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/jumbo-dispenser-demo`. The demo is
open loop and deterministic. The left arm puts its fingers into the stub's core from the front, pointing at the
board, and opens them against the core; the right arm holds the primary button while the left draws the stub off,
carries it across and slides it onto the reserve spindle while the right holds the reserve button. The left arm then
picks the new roll off its carton the same way, carries it across at arm's length, slides it onto the primary while
the right holds the primary button, and pushes its front face home with closed fingers. The right arm presses the
stay button, the cover swings down and latches on its own, and the right arm pushes the shutter's thumb tab across
past half way; the detent takes it to the primary window. The demo reaches 1.0 at about step 1,860.

Degenerate checks use the same command with a variable set in the environment. Each one scores below 1.0:

- `DEMO_VARIANT=shutter_left` does everything except the shutter, which stays over the reserve (stub) window. It
  scores 0.75: the shutter's 0.15 and the final 0.10 need the changeover.
- `DEMO_VARIANT=close_only` releases the cover over the untouched dispenser. The cover points need both rolls
  seated. Scores 0.
- `DEMO_VARIANT=trash_stub` drops the stub into the bin. This trips the bin latch. Scores 0.
- `DEMO_VARIANT=bench_stub` stands the stub on the bench beside the dispenser and loads the new roll on the primary.
  A stub left outside the dispenser scores 0.
- `DEMO_VARIANT=offset_seat` slides the stub onto the reserve spindle without holding the reserve button, so the
  sprung lug is never drawn in over it and the stub is not engaged. Nothing after it counts. Scores 0.
- `DEMO_VARIANT=loose_roll` moves the stub correctly, then slides the new roll toward the primary without holding its
  button, so it hangs up on the sprung lug unseated. Scores 0.25 (the stub stage only).

`JUMBO_DEBUG=1` prints the state every 20 steps.

## Videos and screenshot

```bash
inspect-robots video ~/yam-eval/jumbo-dispenser-demo/*.json
ffmpeg -y -i ~/yam-eval/jumbo-dispenser-demo/frames/<run>/*top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,000 steps (100 seconds at 20 Hz). The demo plan is about 93 s long and reaches 1.0 at about step 1,860.
- There is one fixed layout (`-E object_jitter=0`), so the seed does not change the start. The demo is open loop;
  Dream-1 is hosted and not seeded.
- A trial takes roughly 6 to 10 minutes of wall clock on an L40S, including Isaac Sim warm-up.

## Reset between trials

Open the latch and swing the cover up to 170 deg onto its stay. Hold the primary button and take the new roll off;
hold the reserve button and take the stub off. Put the stub back on the primary (button held) and lay the new roll on
its carton, front end at its mark. Slide the changeover shutter back over the reserve window.
