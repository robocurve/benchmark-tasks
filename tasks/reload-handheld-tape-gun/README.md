# Reload a handheld carton-sealing tape gun

Instruction given to the policy: `Load the new roll into the tape gun so it is ready to tape a carton.`

The instruction gives no hint about the steps. Done right: one hand holds the gun by its handle; the other puts the
new roll on the hub with its tape face out (as packed), pushes it 12 N past the hub's detent ball so it drops onto
its seat, swings the red blade guard open, takes the leader's folded tab, peels it back, leads it round the outside of
the sprung tension roller and down along the nose so its end sticks 5 to 25 mm past the blade, and the guard is
swung shut on it. Hands off, nothing moving.

Files: `task.json`, `reload_handheld_tape_gun.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (joints added to the stock runtime in memory; this copy adds a
pivoting catch so the seated roll turns on its hub), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/reload-handheld-tape-gun/reload_handheld_tape_gun.py
```

Asserts the scoring rules on hand-built states (16 cases, plus a sweep showing a leader still wound on the roll never
counts as round the roller, at any roll angle or roller-arm position) and replays every demo plan (full and the four
degenerate variants, both arms) through the kinematics, checking IK error and joint limits.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/reload-handheld-tape-gun/reload_handheld_tape_gun.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the new roll into the tape gun so it is ready to tape a carton." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/tapegun-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (log dir `~/yam-eval/tapegun-demo`). Open loop and deterministic. The left
arm pinches the handle and holds the gun. The right arm lifts the roll by its core, sets it on the detent, lets go,
presses its face down with both pads until the detent gives and the roll drops onto its seat, then pushes the guard
open round its hinge, pinches the leader's tab, peels it back round the roll, draws it past the roller, turns it down
the roller's outside and slides it along the nose face past the blade. The left arm lets go of the handle and pushes
the guard back past its detent; its spring shuts it on the leader. The right arm lets go. `TAPE_DEBUG=1` prints the
state every second.

Degenerate checks (same command, `DEMO_VARIANT` in the environment):

- `beside`: sets the roll down on the bench next to the gun: 0.
- `over`: seats the roll, opens the guard, then lays the leader straight from the roll to the nose so the roller is on
  its inside (draped over it) and pushes the guard shut on it: 0.3 (seated only). Nothing latches, so the guard could
  be opened again and the leader rerouted.
- `tipped`: lifts the gun's handle end, drops it and leaves: 0 (nothing loaded; knocking the gun over on the bench is
  not itself a failure, only something dropping off the bench is).
- `wound_pushed`: seats the roll, leaves the leader wound on it, turns the roll on its hub until the wound leader
  faces the roller, then pushes the roller's arm in onto it and holds it there to the end: 0.3 (seated only). The
  wound leader touches the roller's outer side, but it is still on the roll and does not go round the roller.

## Run limit, runtime and seeds

- Run limit: 1,600 steps (80 seconds at 20 Hz).
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- Roughly 1 minute of wall clock per 400 steps on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Take the roll off the hub (press the detent collar back up), rewind the leader onto the roll and stick its tab down,
lay the roll face up at its mark, shut the guard.
