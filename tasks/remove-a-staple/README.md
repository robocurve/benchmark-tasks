# Remove a staple without tearing the papers

Lift the clinched staple with a forked remover, then put it down clear of the
sheets. The scripted demonstration scores **1.0** in 29.8 simulated seconds. Dream-1
scores **0.0** after the full 30-second limit.

## Run

Use Python 3.12 and the Isaac/YAM versions in `ENVIRONMENT.json`, including its
public asset-root patch. An NVIDIA GPU renders the cameras; physics runs on the
CPU. Configure Dream credentials with its SDK. From the repository root:

```sh
python tasks/remove-a-staple/preview_panels.py --out DEMO --steps 596 --physics-hz 240 --quasistatic --paper-profile heavy-200gsm-candidate --panel-spacing-m .04 --minimum-panel-mass-mg 5 --solver-type PGS --position-iterations 128 --recenter-at-paper --judging-views --plan tasks/remove-a-staple/demo-plan.json --joint-tracking --tool-pose-servo
python tasks/remove-a-staple/preview_panels.py --out DREAM --steps 600 --physics-hz 240 --quasistatic --paper-profile heavy-200gsm-candidate --panel-spacing-m .04 --minimum-panel-mass-mg 5 --solver-type PGS --position-iterations 128 --recenter-at-paper --judging-views --dream --dream-rows-per-request 4
python tasks/remove-a-staple/scoring.py tasks/remove-a-staple/evidence/demo-judging.json
```

Use new output directories. Reset is fixed with no randomization. The demo took
550 wall seconds on a shared A40/9-vCPU machine; each action represents 50 ms.
Dream gets only standard cameras and proprioception. The demo's measured-tool
feedback produces ordinary bounded joint targets. Both use the same material
rules. The Dream trial took 596 wall seconds and 142 requests in one session.

## Approximation and failure

Each sheet consists of 190 dynamic panels with passive bending hinges. They
can bend and move; neither sheet is pinned. A 5 mg floor stabilizes tiny panels.
The wire uses a quasistatic clinch model: the fork must enter beneath the crown
and lift it. The legs straighten through the original holes. Pulling across
the fork before extraction records damage; withdrawing the fork loses contact.
After extraction, ideal static friction retains the staple on an upward fork;
it falls if the fork tips. No physics rule refers to demo timing or task score.

This tests approach, seating, extraction and careful release. It does not
calibrate steel bending force, fibre tearing or frictional slip. Native paper
hinge breaks and modelled crosswise damage invalidate a run. Small temporary
panel-joint residuals are numerical compliance, not measured physical tears;
the demo's maximum is 3.27 mm and final residuals are below 0.52 mm. Photos and
native geometry retain intact sheet topology. Do not interpret old micrometre
continuity telemetry as a calibrated damage law.

`setup.json`, `JUDGING.md` and `VALIDATION.json` contain the fixed layout,
physical rubric and evidence. The filename `preview_panels.py` is historical;
the published commands execute complete fresh-reset trials.
