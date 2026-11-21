# Revised liner validation

The revised normal-camera demo completed with score **1.0**, 4,910 actions,
245.5 simulated seconds and 613.21 seconds wall time, with no trial errors.
Dream-1 beta completed with score **0.0**, 6,000 actions, 300 simulated
seconds and 746.78 seconds wall time, with no trial errors. Its frozen source
manifest and runtime revision exactly match the demo.
Historical unlined-cap results remain in `historical-validation.json`.

Runtime revision: `5a2dd63eb3e83960e357d96d51deec1d144bdf3ecdbbf71c5b780e96591e3520`.

The geometry audit confirms four pen colliders, 49 cap colliders and 128 visible
noncolliding liner pieces per cap, plus the actual 128/16 solver settings.
Synthetic reset checks pass for joints, insertion/contact/scoring histories
and the resolved-motion history. This audit is not force calibration.
77 task tests and one separate approach-continuity regression pass.

The native top timeline was reviewed every two seconds through the final
245.5-second frame. Both wrist views were sampled at one-second intervals
around each engagement, alongside setup/final frames. All four matching caps
seat and remain attached through support release; two red and two blue
assemblies are released in their correct areas. The held objects partly
occlude the wrist close-ups, so measured engagement records also support the
geometry conclusion. No identifying text was found in the reviewed media.

## Reproduce checks

Activate the documented Arena/YAM runtime and run from this task directory:

```sh
uv pip install --no-deps -e .
python -m pytest -q
python audit_retention_runtime.py --scheduled --output liner-runtime-audit.json
```

The CPU tests cover geometric engagement, closed-end clearance, wrong-color
and held-object scoring, recovery, controller bounds and solver configuration.
The runtime audit inspects actual cap geometry, solver settings and synthetic
joint cleanup. It is not an insertion demonstration or force calibration.

Use the README's normal camera-enabled command for the scripted demo. Use
the same command with `--policy dream -P model=dreamlabs/dream-1-beta` for the
model trial. Both must use identical final source, physical settings, cameras,
seed 0, zero jitter, instruction and 6,000-action limit. Render their actual
terminal JSONs with `inspect-robots video RESULT.json`.

## Evidence retained

- Record actual runtime versions and immutable source/asset hashes.
- Verify runtime geometry: 49 cap colliders, four pen colliders, 128 visible
  noncolliding liner pieces per cap, and actual solver settings.
- Record accepted geometry at all four insertions and after support release;
  verify no retention losses, all four released placements, score 1.0 and a
  completed terminal result within the action limit.
- Review native top and both wrist videos, and an actual initial setup image.
- Complete Dream on matching source and record its actual score, steps,
  runtime and media. No camera-free development run substitutes for these.
- Verify reset removes retention joints and clears contact/insertion/scoring
  histories, then rerun source checks and document the final evidence.

The silicone zone and pose-preserving joint remain a disclosed approximation.
No calibrated elasticity, physical holding force, insertion force or pull-off
force is claimed. Synthetic reset testing does not validate those properties.

Dream review covered the top timeline every five seconds and both wrist
cameras at selected one-second windows across the run, including the final
frame. It manipulates a red pen and loose caps but never establishes cap
retention. The final uncapped pen in the left area correctly earns no credit.

New media remains local. `task.json` retains the historical media ID until an
authorized upload replaces it; it does not identify these revised videos.
