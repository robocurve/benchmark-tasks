# Fit and test the spring clamp on a glass vacuum filter holder

Seat the glass funnel over the membrane and secure both flanges with the spring clamp. Prove that the joint carries the load by lifting the entire dry assembly by the upper funnel. The other arm must stay clear. Hold the bottom of the flask at least 20 mm above the bench for one second, then set it down upright and release it.

## Run one trial

Use Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0, inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU. The stock YAM station, arm meshes and cameras are runtime dependencies. In the task folder, run:

```sh
uv pip install --no-deps .
python -m rdb_filter_clamp.run scripted --out out/scripted
python -m rdb_filter_clamp.run dream --out out/dream
```

Use a fresh output directory for each run. The Dream-1 client needs a configured credential. Both runs start from the same scene with seed 0 and zero object jitter. The limit is 2,400 actions at 20 Hz, or 120 seconds of simulation. Dream-1 receives overhead and wrist RGB at 320 × 176 plus robot joint and gripper positions. The scripted controller reads object poses and jaw angles and commands only robot joints and grippers.

## Score and replay

The terminal score is in `RESULT.json`. A seated and clamped assembly earns 0.5. A one-handed proof lift followed by upright set-down and release earns 1.0. Other assemblies and latched failures earn 0. The exact geometry, time and contact conditions are in `task.json`.

Use `media/side_cam.mp4` to inspect the flask foot against the stationary 20 mm stripe, then check that the joined parts return to the bench. The per-step `media/grader-trace.jsonl` records measured foot clearance, sole supporting arm, consecutive lift ticks and hands-off ticks. The setup image, overhead video and model images accompany the trial. Contact forces and independent funnel, clamp and flask poses appear in the result diagnostics.

## Setup, reset and tests

`task.json` and `REFERENCE-SPEC.json` describe the fixed scene and reset. Return the flask to its mark, replace the dry membrane, stand the funnel on its wipe and leave the spring clamp shut on its mark. Reset clears proof-lift history. See `TECHNICAL.md` for dimensions, physics and the physical scoring procedure.

```sh
python -m pytest -q tests
```

The optional YAM kinematic test checks controller reach. Only the native PhysX trial tests load transfer through the clamp. Task code and generated geometry are MIT licensed.

## Native qualification

| Run | Score | Control steps | Wall time |
| --- | ---: | ---: | ---: |
| Scripted reference | 1 | 1,205 | 276 s |
| Dream-1 | 0 | 2,400 | 686 s |
