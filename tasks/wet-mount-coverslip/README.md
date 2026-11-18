# Prepare a wet microscope mount

Locally completed implementation and model evaluation. Media upload and PR submission are pending.
Wet-mount preparation is a common manual sample-preparation step in research microscopy. The physical outcome is a released coverslip with a continuous liquid film over the specimen, without a large trapped air pocket; this task stops before microscope imaging.

## Reproduction

Use Python3.12 in the official pinned IsaacLab-Arena runtime; see `runtime-lock.json`.
From this task folder, with a fresh output directory and an available GPU:

```bash
python run.py --mode setup --gpu auto --log-dir /tmp/wet-mount-coverslip-setup
python run.py --mode demo --gpu auto --log-dir /tmp/wet-mount-coverslip-demo
python validate_run.py /tmp/wet-mount-coverslip-demo
python run.py --mode dream --config-only --log-dir /tmp/wet-mount-coverslip-dream-config
```

The last command only checks configuration. An actual Dream run costs API credit,
sends inference RGB/joint data and requires separate authorization; removing
`--config-only` runs up to2400actions/120s with `record=false`, without warmup.
It installs no scripted controller or demo plan. A working Dream SDK credential
is required then, never embedded in this folder.

The scripted demonstration has1000actions/50s;
it uses a known-layout joint plan and is feasibility evidence, not a learned visual
policy. Simulator substeps run at960Hz while controls and original
320×176 RGB cameras remain20Hz. There is one fixed scene and no random seed/jitter.
The original robot, joint limits, lights, camera mounts and camera intrinsics remain.

The runner archives source, all three camera streams, native body/contact records,
material history and the official score. `validate_run.py` checks completeness,
source identity and replays the scorer from native physics. A zero process exit
alone is not a successful task result. Preserve every failed attempt separately.

## Setup and grading

One75×25×1mm glass slide on an80×30×29mm support; one loose22×22×0.15mm glass coverslip on a10×10×36.5mm pickup pedestal; a6×6mm nonliving optical specimen and10µL dyed water centred on the slide. Slide centre(330,25,777)mm; coverslip centre(330,-65,784.075)mm. Original YAM station.

After1s released, supported and still, with the cover entirely on the slide:1.0 if at least98% of the central12×12mm region is wet and no trapped-air component exceeds the area of a1mm-diameter circle;0.5 if at least50% is wet;otherwise0. Interarm collision gives0. Final state, not peak score.

See `SCORING.md`, `MATERIAL_MODEL.md` and the local review package. Free native full-score validation passed. Dream remains unrun, so final scores
and media ID must be filled only after a separately authorized model trial.

## Sources

- [AmScope coverslip sizes](https://amscope.com/products/cs-s22-200)
- [Microscopy technique](https://amscope.co.uk/pages/using-a-microscope)
- [Wet-mount procedure](https://marclos.github.io/SOPs/lab_sops/SOP-L601-wet-mount-microscopy.html)

The task also uses the geometry packages pinned in `requirements-extra.txt`
(the official runtime already contains them in the verified environment).

## Verified local result

Scripted demo: **1.0**, 1000 actions / 50 s. All three original
camera streams contain 1001 frames; exact-source physics replay agrees
with the official final score. This is scripted feasibility, not a Dream result.

Observed local source-archive-to-result wall time: approximately15.1 minutes
on an RTX4090 with concurrent work on the host. The action budget is simulated
time; initialization, rendering, archiving and validation add wall time.

## Formal model evaluation

Dream-1: 0.0/2400 actions/120 s; scripted demo: 1.0/1000 actions/50 s. Both episodes use the same archived runtime, scene, instruction and scorer. Dream receives the original three RGB cameras and joint state at20Hz, with record=false and no scripted plan. The fullscore script is a separate feasibility demonstration. All original frames and exact scoring replays were verified. Material-model approximations and physical grading are documented in the accompanying files.
