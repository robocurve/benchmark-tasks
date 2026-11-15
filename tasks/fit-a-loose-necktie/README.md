# Tighten and fit a loose necktie

Finish a prepared necktie on a collar form: hold the knot, draw the wide blade
through, slide the knot up, and release both hands. The knot starts threaded
but loose. This task does not claim to tie a knot from an untied strip.

## Run

Use Python 3.12 and the Isaac 6.1 / YAM environment pinned in `ENVIRONMENT.json`, including the public asset-root patch. An NVIDIA GPU renders the cameras; robot physics runs on the CPU. From the repository root:

```sh
python tasks/fit-a-loose-necktie/run.py --mode demo --out /tmp/tie-demo
python tasks/fit-a-loose-necktie/run.py --mode dream --steps 1200 --out /tmp/tie-dream
python -m unittest discover -s tasks/fit-a-loose-necktie -p 'test_*.py'
```

Dream uses its normal credential file or `DREAM_API_KEY`. Credentials are never
included in the task or evidence. Both runs use the same scene and material
model, normal joint limits and 20 Hz actions. The layout is fixed; no randomized
seed is needed. The demo scores **1.0** in 580 actions (29 seconds), including a final hold.
Dream-1 scores **0.0** after all 1,200 actions in one session. On an A40, the
final demo took 239 wall seconds and Dream took 524 seconds, including startup
and API latency. The earlier ambiguous instruction caused an API refusal; its
record is included, and the final instruction explicitly names the plastic form.
The model receives measured robot poses and finger openings, never policy
labels, desired actions or a target score. The demo uses joint tracking only;
it does not read the tie state. Dream receives its normal three cameras and
joint observations. Run only one Dream trial at a time per shared API key.

Outputs include front/rear/side videos and photographs, the policy camera
video, measured robot actions, material measurements and source hashes.
`run.json` contains the computed score. To recompute it from a saved report:

```sh
PYTHONPATH=tasks/fit-a-loose-necktie python -c 'import json; from material import score; print(score(json.load(open("/tmp/tie-demo/run.json"))["measurements"]))'
```

Inspect the videos against `JUDGING.md`
before treating that score as submission evidence.

## Material approximation

This is a reduced model of a **prepared knot**, not full cloth physics. Its
topology is fixed. Closing fingers around visible material captures a small
patch; reopening releases it. This is idealized capture, not a resolved pinch:
a material sample must lie between the closing pads with a gap no greater than
30 mm (1.5 mm contact tolerance); opening beyond 40 mm releases it. Relative motion of a held knot and held wide
blade feeds material through the wrap. Reversing that motion loosens it.
Pulling the wide blade without supporting the knot moves the assembly instead
of tightening it. The neckband, wrap and two tails share a fixed 1.26 m material
budget. Pulling an end out invalidates the trial.

The wrap's displayed width follows the material left in it. The knot stays in
the form's front plane; released blades approach their gravity drape. Internal
folds, full cloth contact, reaction forces on the fingers, tearing and arbitrary
untying/rethreading are not simulated. A displaced grasp slips after 25 mm of
separation. Parameters are engineering approximations, not calibrated textile
measurements. The robot and rigid collar still use native PhysX dynamics.

These simplifications limit the task to finishing and adjusting the disclosed
prepared necktie. They cannot establish an ability to tie a necktie from scratch.
The real-world rubric judges the resulting tie with photographs and a ruler.

