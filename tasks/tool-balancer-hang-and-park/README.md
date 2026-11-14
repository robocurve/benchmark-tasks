# Hang a nutrunner back on its spring balancer

Instruction given to the policy: `Hang the nutrunner back on its balancer.`

The balancer's constant-force spring is set for the tool: it pulls the cable up with 12.1 N, 3% over the weight
of hook and tool together, so a hung tool drifts up into the park detent. So the 1.1 kg tool has to be lifted to the hook, its hang bail (the loop at its balance point,
not the trigger guard further forward) put over the hook's tip onto the bar, and the tool lowered until the
spring takes it; then hook and tool are let down to the park detent and left there. An empty hook pulled down and
let go whips up into its stop, which fails the run.

Files: `task.json`, `tool_balancer_hang_and_park.py` (the scene, `success()`, the partial-credit `score()`, the
scripted `demo()` with its degenerate variants, and an offline self-test) and `mechkit.py` (joints and sliders added
to the stock runtime in memory).

## Environment

This runs on the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and
inspect-robots as it installs them. Run `source ~/IsaacLab-Arena/yam.env` before every shell. The Dream key is in
`~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/tool-balancer-hang-and-park/tool_balancer_hang_and_park.py
```

This asserts the scoring rules on hand-built states and replays every demo variant through the kinematics at
20 Hz, checking joint limits and IK error. It prints `scoring: 12 cases pass` and one line per variant and arm.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/tool-balancer-hang-and-park/tool_balancer_hang_and_park.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the nutrunner back on its balancer." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/balancer-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (log dir `~/yam-eval/balancer-demo`). Open loop and deterministic. Both
arms pinch the tool (left behind the bail, right just behind the gear head), lift it until its back is 4 mm under
the hook bar, carry it 9 cm sideways so the bail passes over the hook's tip onto the bar, and lower it: the bail
settles on the bar, the spring takes the weight, and the arms lower tool and hook together 1.5 cm to the park
detent, let go and move clear. `BAL_DEBUG=1` prints the cable, speeds and score every second.

Degenerate checks (same command, `DEMO_VARIANT` in the environment), each scoring 0:

- `on_tool`: carries the tool under the hook and sets it back on the bench, then pushes the hook down by its swivel
  until the hook bar rests on top of the bail, and holds it there.
- `guard`: shifts the tool 6 cm back while carrying it, so the trigger guard instead of the bail goes onto the
  hook, then lowers and lets go as in the demo.

## Videos and screenshot

```bash
for log in ~/yam-eval/balancer-*/*.json; do inspect-robots video "$log"; done
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- Roughly 1 minute of wall clock per 400 steps on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Lift the tool off the hook, let the hook up to its stop gently, lay the tool back at its mark (centred at x 26,
y -6.4, along x, loops up).
