# Pipe a shell border

Pipe a joined row of ridged icing shells between two marks on a cake, then set
the dispenser down. The task tests coordinated pressure and tool motion.

Native demo: **1.0**. Dream-1 baseline: **0.0**. Native trial 111 scores 1.0:
six ridged shells covering 87 mm, with the dispenser released and no icing outside
the permitted strip. Independent review of the native top, side and detail
photographs agrees with the six-shell result.
Dream completes the full 2,400-action episode and scores 0.0: it deposits no icing.
The judging guide specifies calibrated projected-section photographs for the
small ridge-depth check, including independent reference-step checks and retakes
for unresolved measurements. This physical judging procedure has not been tested
on real icing here. Videos and the setup screenshot are linked by the media ID in `task.json`.

## Setup

The YAM station table is at Z = 0.7475 m. A 180 × 140 × 50 mm rectangular,
crumb-coated cake has its centre at (0.300, 0, 0.7755) m. Its top is Z = 0.8005 m.
A 200 × 160 × 3 mm aluminium cake board is centred at (0.300, 0, 0.749) m. The marked border is
the 90 mm edible guide line from (0.235, −0.045) to (0.235, +0.045) m on the cake top.
The line is 0.5 mm wide; its two transverse end marks are 10 × 0.5 mm.
All guide marks are 0.1 mm thick.

Use a hand-operated 20 ml plastic icing syringe with a passive sliding piston
and a stainless steel six-point star nozzle. Nominal barrel grip: 30 × 80 × 30 mm; piston diameter:
25 mm; maximum stroke: 40.74 mm. The nozzle is 40 mm long, tapering to a hollow
star aperture with a 2.5 mm mean radius, six lobes, and radial limits of 1.625–3.375 mm. The metal outer radius is 4 mm (minimum wall 0.625 mm). The model starts with 20 ml of icing.
Its barrel begins at (0.180, −0.160, 0.7775) m, nozzle along −Y; the plunger centre
is at (0.180, −0.100, 0.7775) m, 60 mm behind it. Both bodies start at yaw 180°.
A 50 × 40 × 15 mm plastic rest at (0.180, −0.160, 0.755) m supports the dispenser. The tool starts
on the table, outside both hands. The 80 g barrel and 25 g plunger are dynamic.

This is a proposed reproducible prop specification. No real dispenser or recipe
has yet been measured. A syringe avoids pretending that a
rigid object is a deformable piping bag; pressure control still comes from
physical piston movement.

## Run and score

Use the Linux/NVIDIA stack pinned in `ENVIRONMENT.json`, with its YAM adapter
installed. Run from the repository root, using the IsaacLab environment's Python.
The saved plan includes the complete physical pickup, six shells and release:

```bash
python tasks/pipe-a-shell-border/run.py --policy demo --joint-feedback --state-assisted-demo --plan tasks/pipe-a-shell-border/demo-plan.json --steps 2400 --tool-substep-trace --out artifacts/piping-demo
python tasks/pipe-a-shell-border/audit_run.py artifacts/piping-demo --out artifacts/piping-demo/audit.json
python tasks/pipe-a-shell-border/run.py --policy dream --steps 2400 --out artifacts/piping-dream
python tasks/pipe-a-shell-border/audit_run.py artifacts/piping-dream --out artifacts/piping-dream/audit.json
```

Configure Dream credentials privately through the client environment or its
credential file. Never put a key in a command, task file or recorded artifact.
Use a new output directory for every run. The generic scene runner's placeholder
score is not this task's score; `run.py` performs the material coupling and
`audit_run.py` independently scores the saved physical record.

Both policies get the same fixed layout,240 Hz physics,20 Hz controls, motor limits
and 2,400-action limit (120 simulated seconds). There is no object jitter or
randomized scene seed. The demo ends after 1,114 actions (55.7 s); trial 111 took 502 s
on an RTX 6000 Ada while another task shared the GPU. Dream takes 1,042 s on the same GPU: one session, 32 client preroll steps,
296 successful eight-action requests and two transient server-error retries.

The scripted demo openly uses native barrel/plunger poses and joint angles to
command bounded ordinary joint targets. A fixed pressure/motion recipe and
recorded joint feedforward compensate tracking lag. It never moves objects,
changes motor gains, reads the score or injects icing. Dream receives its normal
camera and joint observations, without this assistance.

Each trial saves native video, setup and judging photographs, actions, tool
poses, material state, contact audit, source hashes and the exact demo plan.
The optional substep trace allows a separate consistency check:

```bash
python tasks/pipe-a-shell-border/replay_material.py artifacts/piping-demo --out artifacts/piping-replay
```

That replay is a diagnostic of measured inputs, not another robot demonstration.

## Judging and evidence

[JUDGING.md](JUDGING.md) defines the ruler/photo checks and partial credit. Count
actual raised heads, tapered tails and continuous ridges; a plain line or row of
round blobs cannot pass. The scorer uses the final material surface, not intended
shell locations. Full credit also requires a joined finish and released tool.

Trial 111's independent audit finds 87 mm of valid decoration, six ridged shells,
zero icing outside the permitted strip, no cake gouge, and more than 56 mm
clearance from the border at release. Native top, side and close-up photographs
show the raised heads and tapered, ridged tails. Exact millimetre criteria should
be checked with the measured profiles as well as the photographs; the material
surface is visibly faceted. See [the recorded evidence](evidence/native-six-shell-111.json).

The preceding trial 099's exact 11,928-step input replay reproduces deposited
volume to numerical precision. Trial 111 retains its shell-making motion and
slows the return of the dispenser to prevent an unintended final pressure pulse.
No scoring threshold or material parameter changed for this cleaner finish.

The scorer originally treated a low tail shoulder as a separate head and
truncated the last shell. Its correction applies the existing 2.5 mm head-height
minimum before segmentation. Over-height and oversized mounds still obstruct a
shell. A second correction measures each head and tail on the same connected
visible footprint, so detached lateral blobs cannot inflate its width. Both final
records were rescored: demo 1.0, Dream 0.0. No judging threshold changed.

```bash
python -m unittest discover -s tasks/pipe-a-shell-border -p 'test_*.py'
```

The 55 CPU checks cover pressure causality, finite volume, physical contact,
damage, negative shape controls, segmentation, released-tool evidence and
projective photo measurement with calibration rejection.
Older experiments remain in `evidence/` and `VALIDATION.json`; their scores are
historical and should not be substituted for the final pair of runs.

## Simulation limits

A passive sliding piston compresses a finite reservoir. Pressure drives flow
through the physical star aperture. Ballistic parcels and a conservative 0.5 mm
surface grid model deposited icing; a Herschel–Bulkley law controls spreading.
The tool receives pressure, seal-drag and contact reactions. Both the metal
nozzle and material law use the same aperture. See [MATERIAL.md](MATERIAL.md).

The cream and seal parameters are engineering approximations, not measurements
of a particular commercial recipe. The model omits suspended strings, overhangs,
voids and thixotropy. It cannot simulate finger smearing: non-nozzle overlap with
icing or cake invalidates the run. Cake gouging is a penetration measurement,
not a fracture simulation. Review the native photographs alongside the score.

The technique follows [Wilton's shell-border guide](https://wilton.com/how-to-pipe-a-shell-border/wltech-131/):
build a head, draw a tapered tail while reducing pressure, and overlap the next
head with it. Accepted [filing #1374](https://github.com/robocurve/benchmark-tasks/pull/1374)
and feedback on [caulking #1454](https://github.com/robocurve/benchmark-tasks/pull/1454)
and [cake smoothing #674](https://github.com/robocurve/benchmark-tasks/pull/674)
informed the need for causal material manipulation. Those decisions do not
establish acceptance of this task. No other team's task code was copied.

## Rescore the included records without a GPU

```bash
mkdir -p artifacts/cake-records
tar xzf tasks/pipe-a-shell-border/evidence/demo-record.tar.gz -C artifacts/cake-records
tar xzf tasks/pipe-a-shell-border/evidence/dream-record.tar.gz -C artifacts/cake-records
python tasks/pipe-a-shell-border/audit_run.py artifacts/cake-records/demo --out artifacts/cake-records/demo-audit.json
python tasks/pipe-a-shell-border/audit_run.py artifacts/cake-records/dream --out artifacts/cake-records/dream-audit.json
```

The extracted records independently reproduce demo 1.0 and Dream 0.0. Videos and the setup screenshot are uploaded separately to Robocurve.
`task.json` records their media ID. The native record archives provide reproducible
geometry and contact evidence; they do not replace watching the videos.
