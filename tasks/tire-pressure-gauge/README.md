# Read a wheel's tyre pressure through its Schrader valve with a pencil gauge, then put the gauge back

The robot takes a pencil tyre gauge from its holder, seats its chuck on the Schrader valve of a pneumatic hand-truck
wheel so that the gauge's depressor pin pushes the valve core down, holds it until the stick has drawn out to the
tyre's pressure, and puts the gauge back upright in its holder. At the end the stick reads 33 to 37 (the tyre is at
35 psi), the gauge is home and still, and both grippers have let go. Anyone with a car, bicycle, wheelbarrow or sack
truck does this; a courier or warehouse picker does it on a soft wheel.

The valve is a real Schrader valve: a brass barrel 8.5 mm across and 19 mm tall with a 4.5 mm bore, and in the bore a
2.4 mm core pin that sits 0.5 mm below the tip on a 0.8 N/mm spring with 2.5 mm of travel. The gauge's chuck is a 12 mm
square socket 12 mm deep whose ceiling is a rubber gasket face, with a 2 mm depressor pin standing 2 mm out of the
ceiling. The chuck drops over the barrel, the gasket face seats on the barrel's tip and seals there (no collar, no
separate plate), and the depressor goes down the bore and pushes the core pin 1.5 mm, which opens the valve. Air flows
into the gauge only while the gasket is seated on the tip, the gauge is square to the valve (3 deg) and the core is
pushed down 1.2 mm or more.

Physics defeats the natural shortcuts. A press that is square and firm but beside the valve, or with the depressor on the
barrel wall instead of in the bore, or too light to push the core, or tilted so the gasket cannot seat, never draws the
stick to the tyre's pressure.

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo and
cheat runs below:

- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 1ff9a16d8e30 (`sha256sum task.py | cut -c1-12`)
- inspect-robots-dream 1.0.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial; yam-arena commit
  0059d02a1a91; task file 1ff9a16d8e30 equals `eval.environment_revision` in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the wheel (with its valve core pin on a sprung slider in the barrel's bore) and the gauge (with its sliding stick) as
PhysX articulations, and calls the task's per-step record after every control step. No fork of yam_arena and no
third-party assets: every body is built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

17 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; nothing, an angled press (5 deg, stick at
12 mm), a light press (core 0.9 mm), a gasket seated with the core not pushed, the depressor on the barrel wall, a
press beside the valve, a lifted gasket with the valve open (leak), a short seal (1.0 s) and a stick pulled out by hand
with no seal all score 0; a held seal with the stick at 31 mm scores 0.4; a gauge left on the valve, or a gripper still
holding at the end, scores 0.7; a harmless early wrong try that was undone still lets the run score 1.0; an interrupted
press restarts the 1.5 s timer; a reset clears the record; and the seal, open and leak limits.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Read the tyre pressure of the wheel with the pencil gauge, then put the gauge back upright in its holder." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/tire-pressure-gauge-dream
```

Scoring is automatic: `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end (`SCORE_OVER = "end"`).

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                       # scripted demo
python linkdemo.py task.py cheat_out --variant angled     # a cheat variant (or DEMO_VARIANT=angled)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only what a camera could measure: the gauge's pose and
the robot's own gripper. It never reads joint values, the core pin's depth or the record.

1. Right arm: approaches the gauge in its holder with the approach 45 deg downward (the arm cannot reach a level
   approach this low), pinches the 12 mm barrel, records the grasp from the seen pose and lifts it 90 mm.
2. Carries it over the valve, hovers with the gasket face 22 mm above the barrel tip, and nulls the seen position
   error to 0.3 mm; lowers in two steps (servo 8 mm above the seated position), so the socket drops over the barrel and
   the depressor pin enters the bore.
3. Presses down until the gasket face stops on the barrel tip (commanded 10 mm below the seated position; the arm's
   soft joints give only a few newtons), the core pin goes down about 1.4 mm, holds 3 s (the stick draws out over the
   first 1.5 s of sealed press), then lifts.
4. Carries the gauge back over its holder, hovers 30 mm above, nulls the seen error, lowers into the pocket, lets go,
   backs away 80 mm, rises 60 mm and waits.

The demo scores 1.0 (`success`, 987 of 1,200 steps): the core pin pushed down 1.43 mm at most, gasket seated within
0.1 mm of the tip, stick at 34.1 mm at the end, gauge back to 0.0 mm in its holder and still, the grippers clear. The
script reads only the gauge's seen pose and the robot's own gripper; never the core joint or the record.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (1ff9a16d8e30); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `beside` | The full demo, but the gauge is pressed square and firm on the rim face 40 mm from the valve (what the old collar rule would have credited as a press). | No gasket on the tip, no core pushed: no reading. | 0.0 |
| `offset` | The full demo with the gauge 1.7 mm off the valve's axis. | The depressor pin lands on the barrel wall, gasket 2 mm off the tip, core untouched: no seal, no reading. | 0.0 |
| `light` | The full demo, pressed only to 1 mm above the seated position (gasket face 2 mm off the tip). | The depressor tip stays at the bore's mouth, the core is not pushed (0.04 mm), the valve is closed: no reading. | 0.0 |
| `angled` | The full demo with the gauge held 5 deg off the valve's axis. | The gasket cannot seat square; the stick stalls near 12 mm at most. | 0.0 |
| `quick` | Presses squarely but lifts after about 1 s. | Sealed for under 1.5 s: the stick never draws out. | 0.0 |
| `left_only` | The left arm alone, no gauge, presses its closed gripper on the valve tip. | No gauge on the valve: no reading. | 0.0 |

`nothing` moves no body, so it was not rerun after the revision (the offline tests cover it). `right_only` is not a separate cheat here: the real job is one-armed, so the demo itself is the right arm alone.

## Build log (demo runs on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | valve rebuilt as a barrel with a bore and a sprung core pin, chuck with a gasket ceiling and a depressor pin; press commanded 3 mm below the seated position | 0.0: core pushed only 0.92 mm, gasket 0.6 mm off the tip, below the 1.2 mm opening limit |
| 2 | press commanded 10 mm below the seated position | 1.0, core 1.43 mm, gasket within 0.1 mm of the tip |
| 3 | cheat runs (table above) | each 0.0 |

The earlier collar-and-chuck version is superseded; see Revision.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the gauge's tilt
from the valve's axis, the offset of the barrel from the socket axis, the gap between the gasket face and the barrel tip
along the gauge's axis, the core pin's depth, the stick's extension, the gauge's pose and speed (from its pose change
between steps) and each gripper's pad clearance and speed. The record is cleared only by the simulator reset.

A press is sealed while, continuously: the gasket face is within 0.4 mm of the barrel tip, the barrel is within 2 mm of
the socket axis, the tilt is at most 3 deg, and the core pin is pushed down 1.2 mm or more (valve open). While sealed,
the stick is drawn toward 35 mm in proportion to the time sealed (full at 1.5 s). Valve open with the gasket not seated is
a leak and draws the stick only to 12 mm; gasket seated with the core not pushed draws nothing; nothing ever pushes the
stick back in. A wrong first try that is undone does not matter: only the sealed hold and the final state count.

- 0: no sealed press of 1.5 s at any time in the run.
- 0.4: a sealed press of 1.5 s was held.
- 0.7: and the stick ends between 33 and 37 mm.
- 1.0: and the gauge ends within 6 mm across and 4 mm up of its holder position, within 4 deg of upright and still
  (under 2 mm/s) for 1 s, and the release gate holds: every pad point of both grippers at least 30 mm from the gauge and
  both moving under 10 mm/s for 1 s.

`success()` is true on the 1.0 rung and `distance()` is 1 - score. Real trial: the real air seal produces the reading, so
success is read straight off the stick against its 1 mm per psi marks with a steel rule (33 to 37), checked against a
reference gauge on the same valve; a press that does not seat the gasket or open the core gives no reading. Gauge home
is read from a top photo with a 6 mm square drawn round the pocket and a level on the gauge's barrel; the grippers'
clearance from the video.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo ends at step 987.
- Seeds: `--seed 0` for every run. `-E object_jitter=0` fixes the layout. The scripted demo is deterministic.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`; `demo_out/result.json`: `task_score = 1.0`, 987 steps. Video
  `demo_out/top_cam.mp4` (with `left_cam.mp4`, `right_cam.mp4`) and `demo_out/setup.png`.
- Dream-1 run: made at filing with the command above on this exact `task.py` (1ff9a16d8e30): 1200 steps, 8 minutes,
  log adhoc_34a78ea1.json, task_score 0.0, ending max_steps.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials and how to build and check each part

- Wheel: a 10 inch pneumatic hand-truck wheel, rubber tyre 260 mm across by 70 mm wide on a steel rim, 1.4 kg, lying
  flat with its axle at (0.275, -0.100) m, chocked by two wooden blocks and a taped outline so it cannot move. Inflate to
  35 psi and confirm with a reference gauge before each session.
- Valve: a real brass Schrader valve (barrel 8.5 mm across, 19 mm above the upward rim face, 55 mm from the axle at
  (0.220, -0.100) m) fitted through a turned plate bolted over the rim, with its own spring-loaded core. The core pin
  top sits 0.5 mm below the barrel tip and the valve opens at about 1.2 mm of travel (check: a depressor pin pushed 1.5
  mm below the tip makes the tyre hiss; the core spring is about 0.8 N/mm). Replace the core if it leaks.
- Pencil gauge: steel and brass, 124 mm long, 12 mm square barrel, 60 g, a 20 mm square chuck head with a 12 mm square,
  12 mm deep socket; the socket ceiling carries a rubber gasket disc (about 3 mm thick, Shore A 70) that seals on the
  valve's tip, and a 2 mm steel depressor pin stands 2 mm out of the ceiling (a standard chuck has one). A 5 mm square
  stick with 70 mm of travel marked 1 mm per psi, held where it stops by 0.5 N of sliding friction (check with a spring
  scale: 0.4 to 0.6 N). Check before a session: the gauge pressed square by hand on a reference tyre at 35 psi reads
  35 +/- 1; tilted 5 deg it reads low or nothing. A commercial pencil gauge has a round barrel and a nonlinear scale;
  this one has the same layout with a square barrel and a linear scale so that both can be read with a rule.
- Gauge holder: grey ABS, 50 x 50 mm base, a 25 mm square pocket 18 mm deep, 8 mm floor, at (0.300, -0.300) m.
- Fidelity: the simulator does not model air. The stick is drawn out only while the gasket face is seated on the barrel
  tip, the gauge is square to the valve, and the depressor holds the core pin down 1.2 mm or more, all measured from the
  simulated poses and the core joint. The barrel and bore are built from square boxes; the real valve is round. In the
  real build the tyre's air does this work and the reading on the stick is the proof.

## Reset between trials

Push the stick back to 0 mm, stand the gauge upright in its holder, check that the core pin springs back to its rest, and
check the wheel and holder against their taped outlines and the tyre at 35 psi.

## Revision

Reviewer points (construct validity, scoring) and the changes:

| Reviewer point | Change |
|---|---|
| The 24 mm-deep chuck has no specified valve-core actuator | The chuck now has a 2 mm depressor pin standing 2 mm out of its ceiling, and the valve has a bore with a sprung core pin 0.5 mm below the tip (0.8 N/mm, 2.5 mm travel); the pin pushes the core 1.5 mm and the valve opens at 1.2 mm. |
| Its gasket seals against a loose collar rather than the Schrader valve tip | The red collar is gone. The gasket is the socket ceiling and seals on the valve barrel's tip (within 0.4 mm, square within 3 deg, barrel within 2 mm of the socket axis); socket depth is now 12 mm. |
| Collar displacement and 3-degree angle treated as proof of a seal | Sealing now needs the core pin pushed down 1.2 mm and the gasket on the tip together; the collar depth is no longer measured. Valve open with the gasket off the tip is a leak (12 psi); gasket on the tip with the core closed gives nothing. |
| Replace the arrangement with a working Schrader valve and gauge; real setup and scoring so an actual pressure reading establishes success | Simulation, `task.json` setup and the real-trial notes are rewritten around the valve; in the real trial the stick reading against a reference gauge is the success proof. |
| Full-score demonstration of the revised task | Demo scores 1.0 (987 of 1,200 steps). |
| Flag: setup rendered as object placeholders | `setup` in `task.json` is now one plain-text string. |

New cheats for the old loopholes (each scores 0): `beside` (square, firm press on the rim face, no valve) and `offset`
(depressor on the barrel wall) reach the old rule's full-score shape but not the end state; `light` (stopped short,
valve closed) and `left_only` are also added.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public Isaac
Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no third-party
assets or brands.
