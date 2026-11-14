# Mix the two layers in a separatory funnel by shaking it

Instruction given to the policy: `Mix the two layers in the separatory funnel by shaking it, then put it back in its support with the stopper in.`

Files: `task.json`, `sep_funnel_vent_inverted.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (helper module imported by the task file: the stopcock key's friction hinge and the stopper's hold;
it patches yam_arena in memory at import, nothing on disk changes; written for this task set, no third-party code),
`README.md`.

## The liquids and the shaking rule

- **Funnel and liquids:** a 125 mL glass separatory funnel (octagonal bulb 5.6 cm across, 6 cm tall) holding 40 mL
  of water under 40 mL of light petroleum (petroleum ether, bp 40-60 C, density about 0.65), 80 mL in all, about
  two-thirds full, below the three-quarters limit teaching labs give so the layers have room to mix. The liquid is
  not simulated (this simulator's particle liquids can't be read by the scorer or seen by the cameras), and its
  66 g (40 g water + 26 g light petroleum) is not added to the funnel's 120 g of glass.
- **What counts as a shaking stroke:** with the funnel turned over (stem above horizontal) and the stopcock shut
  (under 20 deg), the bulb travels **4 cm or more** between two turning points **and** its speed, from its pose
  at consecutive control steps (distance / 0.05 s), reaches **0.5 m/s** during that stroke. Both must hold;
  a long slow stroke and a short fast jiggle both count for nothing. **12** counted strokes are needed.
- **Where the thresholds come from:** EPA SW-846 Method 3510C (separatory funnel liquid-liquid extraction) says
  to "seal and shake the separatory funnel vigorously for 1 - 2 minutes with periodic venting"
  (https://www.epa.gov/sites/default/files/2015-12/documents/3510c.pdf, section 7.5). EPA SW-846 Method 8032A
  extracts in a 150 mL separatory funnel "using a mechanical shaker at approximately 240 strokes per minute"
  (https://www.epa.gov/sites/default/files/2015-12/documents/8032a.pdf, section 7.2.2). Reciprocating
  separatory-funnel shakers as a product category have a 40 mm stroke. Read as 4 full back-and-forth cycles a
  second of 40 mm, that motion peaks at pi x 0.040 m x 4 /s = 0.50 m/s (about 1.3 g at each reversal); read as
  4 one-way strokes a second, it peaks at half that. The rule takes the faster reading: every counted stroke is
  at least as long as the shaker's and at least as fast at its peak.
- **What it does not establish:** the rule measures the motion that mixes the layers, not the mixing itself.
  12 strokes (about 3 s of vigorous shaking at the demo's pace, in three rounds of four between vents) is a
  minimum, shorter than the 1-2 minutes an EPA method shakes for, so that the shaking and venting fit in the 90 s
  trial.
- **How fast the demo shakes:** it swings the funnel about the left arm's base (first joint only), 13.7 cm strokes
  in 0.25 s each, about 0.8 m/s step to step at mid-stroke in the plan, inside the runner's per-step joint
  guardrail (5% of each joint's range) and under 1.5 m/s. In the simulated demo each of the 12 counted strokes
  measured about 13.5 cm end to end and peaked at 0.77-0.80 m/s (pose change per 0.05 s step); its slow moves out
  to the first end and back to the middle (0.21 m/s) were not counted, and the pressure counter never passed 4.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/sep-funnel-vent-inverted/sep_funnel_vent_inverted.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm, every joint step inside the runner's 5%-of-range
guardrail, the TCP under 1.5 m/s). Expected output:

```
scoring: 23 cases pass
left: 70.5 s plan, worst IK 0.092 mm / 0.014 deg, largest joint step 5.7 deg (35% of the guardrail), top TCP speed 0.81 m/s
right: 70.5 s plan, worst IK 0.076 mm / 0.015 deg, largest joint step 5.6 deg (47% of the guardrail), top TCP speed 0.20 m/s
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/sep-funnel-vent-inverted/sep_funnel_vent_inverted.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mix the two layers in the separatory funnel by shaking it, then put it back in its support with the stopper in." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/sep-funnel-vent-inverted-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sep-funnel-vent-inverted/sep_funnel_vent_inverted.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mix the two layers in the separatory funnel by shaking it, then put it back in its support with the stopper in." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/sep-funnel-vent-inverted-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 70.5 s of plan (about 1430 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/sep-funnel-vent-inverted-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/sep-funnel-vent-inverted-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the funnel back in its support with the stopcock shut and the stopper seated (re-bonded); on a real rig, drain it and refill 40 mL of water and 40 mL of light petroleum, and reset the pressure counter (or use a funnel with a small sealed gas cartridge); the support does not move.
