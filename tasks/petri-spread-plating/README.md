# Spread a liquid sample and close an agar plate

Spread plating is a routine manual preparation step in research microbiology. The task isolates handling the lid and standard T-spreader while distributing a finite liquid sample without gouging the gel. A coloured nonliving stand-in permits repeated physical trials; microbial growth and sterile technique are outside this task.

## Reproduction

Use Python3.12 in the official pinned IsaacLab-Arena runtime; see `runtime-lock.json`.
From this task folder, with a fresh output directory and an available GPU:

```bash
python run.py --mode setup --gpu auto --log-dir /tmp/petri-spread-plating-setup
python run.py --mode demo --gpu auto --log-dir /tmp/petri-spread-plating-demo
python validate_run.py /tmp/petri-spread-plating-demo
python run.py --mode dream --config-only --log-dir /tmp/petri-spread-plating-dream-config
```

The last command only checks configuration. An actual Dream run costs API credit,
sends inference RGB/joint data and requires separate authorization; removing
`--config-only` runs up to4000 actions/200 s with `record=false`, without warmup.
It installs no scripted controller or demo plan. A working Dream SDK credential
is required then, never embedded in this folder.

The scripted demonstration has3540 actions/177 s;
it uses a known-layout plan with privileged native tool-pose feedback, including
ordinary inverse-kinematics pose corrections. Motion uses ordinary joint commands; no
rigid-body transforms or material values are written by the controller. This
is feasibility evidence, not a learned visual policy. Simulator substeps run at240Hz while controls and original
320×176 RGB cameras remain20Hz. There is one fixed scene and no random seed/jitter.
The original robot, joint limits, lights, camera mounts and camera intrinsics remain.

The runner archives source, all three camera streams, native body/contact records,
material history and the official score. `validate_run.py` checks completeness,
source identity and replays the scorer from native physics. A zero process exit
alone is not a successful task result. Preserve every failed attempt separately.

## Setup and grading

One92×16mm nominal Petri dish:base outer diameter89mm with1mm floor and15mm wall, lid92mm with1mm roof and12mm skirt;86.8mm diameter,5mm thick agar;100 µL of 2.00 mg/mL Brilliant Blue FCF in water at the centre of the 1.50% w/w plain agar gel. One140×35mm nominal T-spreader with4mm shaft/bar diameter, on three passive rests. Dish centre(330,40,747.5)mm; tool shaft centre(430,-120,803.5)mm. Base28g,lid8g,tool2.1g.

Released closed dish upright on the table and spreader released on its rack, all still for1s:1.0 for at least85% wet coverage of the central80mm circle and at least75% in each quadrant;0.5 for50% overall;0.25 for25%;otherwise0. Any gouged area of1mm² or more, or interarm collision,gives0. Final state.

For physical trials use NOW Real Food 6410 plain agar in distilled water and Supelco 80717 Brilliant Blue FCF. For each trial use one fresh plate from the timed batch, a dry spreader and a fresh tracer aliquot at 22 ± 1 °C. Apply the central drop and start 10 ± 2 s later with the lid closed. Photograph the opened plate within 10 s after the final hold.

See [physical specimen, tracer preparation and timed reset](PHYSICAL_GRADING.md), [scoring](SCORING.md) and [material model](MATERIAL_MODEL.md).

## Sources

- [GVS standard cell spreaders](https://www.gvs.com/en/catalog/cell-spreader)
- [Spread/lawn plate technique](https://practicalbiology.org/standard-techniques/making-a-spread-or-lawn-plate.html)
- [University of Otago lab procedures](https://blogs.otago.ac.nz/classprep/files/2024/02/MicrobiologyLaboratoryProceduresHandbook2024.pdf)

The four noncolliding black rim ticks define the grading quadrant diameters.
They are ordinary visible ink marks, not additional sensors or constraints.

Release holds the measured arm posture while the jaws open, followed by a
vertical retreat based on joint feedback. No object pose is assigned.

The demo repeats the earlier physically stable sweep route seven times to
distribute the finite liquid, with no integrated height or tilt correction.

After four passes the demonstration places and releases the tool on its
original rack, then grasps it again before continuing. This ordinary physical
regrasp relieves accumulated shaft slip; no state is reset or assigned in code.
The scorer does not require this strategy.

## Verified local result

Scripted demo: **1.0**, 3540 actions / 177 s. All three original
camera streams contain 3541 frames; exact-source physics replay agrees
with the official final score. This is scripted feasibility, not a Dream result.

Observed local source-archive-to-result wall time: approximately29.1 minutes
on an RTX4090 with concurrent work on the host. The action budget is simulated
time; initialization, rendering, archiving and validation add wall time.

## Formal model evaluation

Dream-1: 0.0/4000 actions/200 s; scripted demo: 1.0/3540 actions/177 s. Both episodes use the same archived runtime, scene, instruction and scorer. Dream receives the original three RGB cameras and joint state at20Hz, with record=false and no scripted plan. The fullscore script is a separate feasibility demonstration. All original frames and exact scoring replays were verified. Material-model approximations and physical grading are documented in the accompanying files.
