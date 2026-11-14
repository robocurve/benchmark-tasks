# Key a pulley onto a shaft

Instruction given to the policy: `Fit the pulley to the shaft and lock it.`

The instruction does not mention the key. The scored steps it leaves unstated: pick the key off the mat, press it
fully into the keyseat (about 10 N), turn the pulley until its keyway lines up with the key, slide it over the key to
the shoulder, then turn the set screw three clicks. A pulley forced over a key that is not fully home scores 0.

Files: `task.json`, `key_a_pulley_onto_shaft.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (joints added to the stock runtime in memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/key-a-pulley-onto-shaft/key_a_pulley_onto_shaft.py
```

Asserts the scoring rules on hand-built states and replays both arms' demo plans through the kinematics. Expected:

```
scoring: 14 cases pass
left: 47.7 s plan, worst IK 0.077 mm / 0.013 deg
right: 15.9 s plan, worst IK 0.092 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/key-a-pulley-onto-shaft/key_a_pulley_onto_shaft.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the pulley to the shaft and lock it." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/keypulley-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted`. Open loop and deterministic. The right arm pinches the key, lowers it
into the keyseat and presses it home (about 6 mm past nominal, the pad gives way at 10 N), lets go and parks. The left
arm drags the pulley rim round with closed fingertips from 45 deg to near the key phase, pinches the set screw (which
centres the keyway over the key) and lets go, pushes the boss front face with closed fingertips to slide the pulley
over the key to the shoulder (the target is 32 mm past the shoulder to make up for the arm's give under load), then
pinches the screw and turns the wrist from -40 to +40 deg (three clicks). `KEYPULLEY_DEBUG=1` prints the key, press depth, slide, phase, screw and
score every second.

Degenerate checks (same command, `DEMO_VARIANT` in the environment), all score 0:

- `no_key`: the pulley turned, slid home and screwed with no key in the shaft.
- `key_on_hub`: the key set on top of the pulley's boss instead of in the keyseat.
- `screw_only`: the set screw turned three clicks with the pulley still at the shaft end.

## Videos and screenshot

```bash
for log in ~/yam-eval/keypulley-*/*.json; do inspect-robots video "$log"; done
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- Roughly 1 minute of wall clock per 400 steps on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Pull the pulley back to the shaft end with its keyway 45 deg off the top, back the set screw out, tap the key out
and lay it on the mat.
