# Register a PCB stencil and print solder paste

Locally completed implementation and model evaluation. Media upload and PR submission are pending.
Stencil printing prepares motor-controller PCBs for component placement during robot hardware assembly. This scene uses the real Adafruit Motor Shield v2.3 pad layout. The task ends at visible deposit registration and separation; it does not claim reflow, electrical testing or production solder-joint qualification.

## Reproduction

Use Python3.12 in the official pinned IsaacLab-Arena runtime; see `runtime-lock.json`.
From this task folder, with a fresh output directory and an available GPU:

```bash
python run.py --mode setup --gpu auto --log-dir /tmp/pcb-stencil-printing-setup
python run.py --mode demo --gpu auto --log-dir /tmp/pcb-stencil-printing-demo
python validate_run.py /tmp/pcb-stencil-printing-demo
python run.py --mode dream --config-only --log-dir /tmp/pcb-stencil-printing-dream-config
```

The last command only checks configuration. An actual Dream run costs API credit,
sends inference RGB/joint data and requires separate authorization; removing
`--config-only` runs up to2400actions/120s with `record=false`, without warmup.
It installs no scripted controller or demo plan. A working Dream SDK credential
is required then, never embedded in this folder.

The scripted demonstration has2040actions/102s;
it uses a known-layout joint plan with privileged native blade-pose and normal-load
feedback. All motion uses ordinary joint targets. It is feasibility evidence,
not a learned visual policy; the Dream branch receives none of this private state. Simulator substeps run at960Hz while controls and original
320×176 RGB cameras remain20Hz. There is one fixed scene and no random seed/jitter.
The original robot, joint limits, lights, camera mounts and camera intrinsics remain.

The runner archives source, all three camera streams, native body/contact records,
material history and the official score. `validate_run.py` checks completeness,
source identity and replays the scorer from native physics. A zero process exit
alone is not a successful task result. Preserve every failed attempt separately.

## Setup and grading

One68.58×53.34×1.6mm Motor Shield PCB fixed on a printing bed; one loose125µm steel stencil with117 real apertures, a104×90mm rigid frame and passive PCB-outline registering skirt; an80×25×0.5mm steel blade with25×18×20mm handle. Stencil55g,blade65g;120µL finite paste bead preloaded on the stencil. PCB centre(330,-65,779.7)mm;resting stencil(330,75,791)mm;blade(440,-140,807.5)mm;separate rests.

Return and release the stencil on its supports and clear the PCB for1s.1.0 requires at least90% of117pads to have75% coverage, every pad at least50% coverage, at least80% of deposit area on pads,and no paste bridges, with at least50mg deposited.0.5 at50% qualifying pads and25mg;0.25 at25% and12.5mg;otherwise0. Interarm collision gives0. Coverage is judged by visible footprint; deposit mass by a1mg-resolution balance. Details in SCORING.md.

See `SCORING.md`, `MATERIAL_MODEL.md` and the local review package. The independent copied-source demo, original-image checks and raw score replay
have passed. The model result is recorded below; the uploaded media ID is intentionally unset until publication.

## Sources

- [Adafruit Motor Shield v2 PCB sources](https://github.com/adafruit/Adafruit-Motor-Shield-V2-PCB), by Adafruit Industries, licensed under Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0). The original board, licence and required attribution are preserved in `reference/`; derived aperture/pad geometry retains that licence.
- Original design, licence, exact commit and pad extraction are included in `reference/` and `board-geometry.json`.

The task also uses the geometry packages pinned in `requirements-extra.txt`
(the official runtime already contains them in the verified environment).

## Verified local result

Scripted demo: **1.0**, 2040 actions / 102 s. All three original
camera streams contain 2041 frames; exact-source physics replay agrees
with the official final score. This is scripted feasibility, not a Dream result.

Observed local source-archive-to-result wall time: approximately17.5 minutes
on an RTX4090 with concurrent work on the host. The action budget is simulated
time; initialization, rendering, archiving and validation add wall time.

## Formal model evaluation

Dream-1: 0.0/2400 actions/120 s; scripted demo: 1.0/2040 actions/102 s. Both episodes use the same archived runtime, scene, instruction and scorer. Dream receives the original three RGB cameras and joint state at20Hz, with record=false and no scripted plan. The fullscore script is a separate feasibility demonstration. All original frames and exact scoring replays were verified. Material-model approximations and physical grading are documented in the accompanying files.
