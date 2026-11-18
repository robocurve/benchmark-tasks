# Pipe a shell border

Pipe a joined shell border along the L-shaped guide and around its corner,
then set the dispenser down. The task tests coordinated pressure and tool motion.

The 18 November corner revision has a complete native full-score demonstration.
The earlier straight-only result is retained only as a negative control.

## Setup

The YAM station table is at Z = 0.7475 m. A 180 × 140 × 50 mm rectangular,
crumb-coated cake has its centre at (0.300, 0, 0.7755) m. Its top is Z = 0.8005 m.
A 200 × 160 × 3 mm aluminium cake board is centred at (0.300, 0, 0.749) m. The marked border is
a 90 mm line from (0.235, −0.045) to (0.235, +0.045) m, then a 60 mm line to (0.295, +0.045) m on the cake top.
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

### Icing and quick surface preparation

Use **Wilton Ready-to-Use Medium Consistency White Decorator Icing, 15 oz,
SKU 191012026**, from one tub for the coating and piping. Keep the closed tub
at 20–22°C until the contents are at that temperature; refrigerated icing
needs advance tempering. Stir gently with a spatula for 30 seconds without
whipping air into it. Add no water, sugar or other consistency modifiers.
The manufacturer specifies stirring at room temperature and suitability for
borders: [product instructions](https://wilton.com/ready-to-use-medium-consistency-white-decorator-icing-15-oz-tub/191012026/).
The 20–22°C interval is our fixed trial condition, not a manufacturer tolerance.

Before the session, trim a ready-made plain Madeira/pound cake to
180 × 140 × 49 mm with a flat, level top. For each trial, brush off loose crumbs,
spread white icing over the top and draw a straight spatula once across two
1 mm spacer strips resting along its sides. Remove the spacers. The coated top
is now 50 mm above the board. Draw both perpendicular guide sections and end marks with an edible
pen using a ruler. This surface preparation takes a few minutes once the cake
and icing are ready; no baking, chilling or crust-setting stage is required.

For visible contrast, mix 0.10 g Wilton Pink gel icing colour into 100 g of the
same icing for 30 seconds. Record the empty dispenser mass first. Load icing from the rear, tap out visible
air, fit the piston, and prime onto a waste plate until a continuous star-shaped
strand emerges. Adjust and re-prime until the loaded dispenser weighs
**17.0 ± 0.5 g more than its empty mass**, using a kitchen scale reading 0.1 g.
Wipe only the outside of the nozzle. The simulator uses a nominal 20 ml at its
stated approximate 850 kg/m³ density (17 g); actual product density is uncalibrated.
Pipe within 10 minutes of surface preparation, at 20–22°C. Keep unused icing
covered. Between trials scrape the top flat, renew its 1 mm white coat and marks,
or replace the cake if damaged. Do not reuse a gouged surface.

The simulator's cream and seal parameters remain engineering approximations;
they have not been fitted to measurements of this product. Specifying the
physical consumables does not claim such calibration. A syringe preserves
physical pressure control without pretending that a rigid handle is a soft bag.

## Run and score

Use the Linux/NVIDIA stack pinned in `ENVIRONMENT.json`, with its YAM adapter
installed. Run from the repository root, using the IsaacLab environment's Python.
The saved plan includes the complete physical pickup, ten shells around the corner, and release:

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

Both policies get the same fixed layout, 240 Hz physics, 20 Hz controls, motor limits
and 2,400-action limit (120 simulated seconds). There is no object jitter or
randomized scene seed. The revised demo scores **1.0** in 1,648 actions;
Dream-1 scores **0.0** after all 2,400 actions. On a shared A40, they took
24.9 and 58.4 minutes respectively, including rendering and model waits.
Dream used 296 requests across 133 response sessions;
server-session recoveries are recorded, so this is not an uninterrupted baseline.

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

The top and close photographs must show the straight sections, joined corner,
raised heads, tapered tails and ridges. Use the 90 mm and 60 mm guides for scale.
Numerical profiles are optional diagnostics, not required judging tools.

```bash
python -m unittest discover -s tasks/pipe-a-shell-border -p 'test_*.py'
```

The checks cover pressure causality, finite volume, physical contact, damage,
shape failures, straightness, corner completion and released-tool evidence.
The old straight-only demo is retained as a negative control for the new rubric.

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
python tasks/pipe-a-shell-border/audit_run.py artifacts/cake-records/demo-corner --out artifacts/cake-records/demo-audit.json
python tasks/pipe-a-shell-border/audit_run.py artifacts/cake-records/dream-corner --out artifacts/cake-records/dream-audit.json
```

The included archives contain the current complete corner trials. Videos and the setup screenshot are
uploaded separately to Robocurve.
`task.json` records their media ID. The native record archives provide reproducible
geometry and contact evidence; they do not replace watching the videos.


Current corner evidence is in `evidence/demo-record.tar.gz` and `dream-record.tar.gz`.
Extract either archive and pass its `demo-corner` or `dream-corner` directory to
`audit_run.py`. The archived code and action plan identify the actual run.
See the [top photograph](evidence/demo-judging-top.png) and
[second-side detail](evidence/demo-judging-last-detail.png) for the same final icing.
Older numbered development summaries are historical diagnostics, not evidence
for the revised corner task.
