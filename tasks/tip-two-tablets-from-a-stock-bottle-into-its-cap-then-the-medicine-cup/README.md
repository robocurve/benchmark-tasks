# Open a safety-capped bottle and pour two tablets via its cap into a medicine cup

Steady the bottle, push its safety cap down, turn it counterclockwise and lift it off. Without touching a tablet, hold the bottle's cap in one hand and, with the other, tip two tablets from the bottle into it; tip any extra from the cap back into the bottle, then tip the two from the cap into the medicine cup. Leave the bottle standing and let go of everything.

## Run one trial

Use Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0,
inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU.
The stock YAM station, arm meshes and cameras are runtime dependencies; they are not bundled.
Install this folder in that environment:

```sh
uv pip install --no-deps .
python -m rdb_tablets_cap.run scripted --out out/scripted
python -m rdb_tablets_cap.run dream --out out/dream
```

The Dream run requires a Dream Labs credential configured through its client. No credential is included.
Both runs use Inspect Robots, the same scene and scorer, seed 0 and zero object jitter.
Run limit: 4,800 steps (4 minutes at 20 Hz). Use the default layout; additional diagnostic variants are not the submitted trial.

Dream-1 receives 320 × 176 overhead and wrist-camera RGB images plus joint and gripper positions.
The scripted policy reads simulator poses and emits the same 14-dimensional joint/gripper actions.
Each trial writes `RESULT.json`, `media/setup.png`, top and side videos, and `media/grader-trace.jsonl`.
The per-step trace includes cap depression, twist, lift, bilateral fingertip contact, opening stage and tablet counts.
A successful demonstration must reach a score of 1.0 within 4,800 actions.

## Setup and scoring

The bottle starts upright with 30 inert tablets. Its inverted cap is mechanically retained on the neck.
The empty medicine cup stands to the left. The robot must steady the bottle and press, turn and lift the cap
before any tablet leaves the bottle. It can then turn the empty cap over, dispense into the held cap, and
tip the two tablets into the cup. The task card gives the dimensions, coordinates, partial-credit rules and reset.

## Reproduction and tests

The passive interlock uses two tabs, neck ledges, detents and an external spring-loaded collar.
The gripper aligns the cap lugs with openings in the ledges to lift the cap free. The controller commands the robot, and the
opening scorer reads the relative motion of the bottle and cap and contact forces at the fingertips.
See [TECHNICAL.md](TECHNICAL.md) for the geometry, task scoring and sources.
[REFERENCE-SPEC.json](REFERENCE-SPEC.json) contains the full task specification.

The seed-0 scripted trial scored 1.0 in 2,363 steps. The robot opened the cap, caught five tablets, returned
three to the bottle, then delivered two to the cup. [Recorded opening and transfer measurements](TECHNICAL.md#recorded-scripted-trial)

Run `python -m pytest -q tests` for the geometry, opening-sequence, dose-scoring and package checks.

Task code and procedural assets use the MIT license. See [LICENSE](LICENSE).

## Native qualification

| Run | Score | Control steps | Wall time |
| --- | ---: | ---: | ---: |
| Scripted reference | 1 | 2,363 | 529 s |
| Dream-1 | 0 | 4,800 | 888 s |
