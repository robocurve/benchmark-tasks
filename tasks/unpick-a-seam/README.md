# Unpick a seam without damaging the fabric

Open six tacking stitches with a seam ripper, preserve the two canvas patches,
and put the tool down. Loose thread remnants may remain in the needle holes.
The task tests selective tool use during garment repair.

## Run and score

Use Python 3.12 and the Isaac/YAM stack in `ENVIRONMENT.json`, including the
public asset-root patch. An NVIDIA GPU renders the cameras; rigid-panel physics
runs on the CPU. Configure Dream credentials through its SDK. From the repo root:

```sh
python tasks/unpick-a-seam/run.py --policy demo --out DEMO
python tasks/unpick-a-seam/run.py --policy dream --out DREAM
python tasks/unpick-a-seam/assess_trace.py DEMO --out DEMO/score.json
```

Each command uses a new output directory and a fresh fixed reset. There is no
layout randomization. Control is 20 Hz and physics is 480 Hz. The action limit is
1,600 steps (80 seconds), including a final three-second released hold. The demo
uses 1,447 ordinary actions and measured tool/stitch alignment. Dream receives
only the standard cameras and joint observations; both policies use the same
material model. Fixed close-up cameras are for judging, not policy input.

`JUDGING.md` defines the physical rubric. The scorer uses actual cut-end gaps
and final tool geometry, not the number of commanded cuts. Any canvas damage
invalidates partial as well as full credit. A completed trial is required to
score; reconstructed development checkpoints are never submission evidence.

The demo scores **1.0** in 1,447 actions (72.35 simulated seconds); Dream-1
scores **0.0** after 1,600 actions. On a shared A40 with nine vCPUs, their
wall times were 1,967 and 1,198 seconds. Run one Dream trial at a time per API key.

## Fixed setup

Coordinates are metres in the YAM station frame; the table is at Z = 0.7475.
Two flexible 140 g/m² cotton-canvas patches measure 75 × 80 × 0.4 mm and overlap
by 20 mm. The upper middle surface is X = 0.2996, Z = 0.8027–0.8777; the lower
is X = 0.3004, Z = 0.7477–0.8227. Both span Y = −0.040–0.040.

Six red 0.8 mm polyester tacking loops are at Y = −30, −18, −6, 6, 18 and 30 mm.
Each uses two 1.6 mm square needle holes per patch, at Z = 0.8077 and 0.8177.
The loose front bridge stands 3.8 mm above the cloth middle surface. These are
separate hand-tacking stitches, not a machine lockstitch.

A bench clamp grips the upper patch's top 8 mm. Its front jaw is 6 × 85 × 16 mm;
the 12 g rear jaw is 6 × 85 × 8 mm and moves on a passive linear guide. A
1,250 N/m spring, 8 N·s/m damping and 0.6 N force limit close the jaw. The
initial gap is 0.60 mm and its travel is 0–1.60 mm. The lower patch's bottom
edge rests on the table. A rigid 65 mm-wide backing board and metal front
rails support the work outside a 26 mm-high open seam window, Z = 0.7997–0.8257.
`passive_clamp.py` and `repair_backing.py` list every stand and guide solid.
The stand is bolted down. Ideal static grip retains only the upper patch’s
8 mm top edge inside the jaws; the rest of both patches remains dynamic.

A 19 g seam ripper starts at (0.200, −0.120, 0.7555), pointing along +X.
Its polymer handle is 70 × 12 × 12 mm. The steel cutting edge is
1 × 0.5 × 4 mm, 50 mm ahead of the handle centre, between a fine 0.5 mm-diameter
long point and a guarded short point. `tool_geometry.py` lists all tool parts.
Neither hand starts holding it.

Reset with two fresh patches and six tacks in this layout. Return the tool to
its starting mark. Real trials use the fixed stand and keep people clear of
the blade and moving arms.

## Material approximation

Each patch bends through 118 native panels joined by passive hinges. Native
fixed constraints approximate a tightened clamp’s nonslip grip on the top edge. A 5 mg
minimum panel mass stabilizes small pieces. Closed tacks follow the adjacent
fabric and connect the layers at the two original needle holes. Measured
inward blade contact of at least 5 mN for one physics step releases those
connectors and the tack becomes a flexible, open 12-link thread. Thread/fabric
contact is omitted, including after cutting; tool and table contacts remain.
This approximates loose fragments sliding free of the original needle holes,
not the detailed removal of fibres. Its numerical
mass is 0.175 g. A measured, enclosed handle grip enables an ideal static-friction
constraint; opening the hand releases it. These rules apply to any policy.

This preserves alignment, feeding the point under a stitch, selective cutting,
and safe release. It does not calibrate fibre friction, textile tearing or
cutting force. Blade/fabric crossing and native panel fracture invalidate a run.
The model allows up to 5 mm numerical panel-junction compliance; that is not a
5 mm physical-tear allowance. Smaller solver residuals are recorded separately
from the physical damage rubric. Fine thread/hole penetration telemetry is not
a textile-validity certificate for this simplified tack model. No cut, grip or
release is triggered by demo timing or task score.

## Checks and evidence

```sh
python -m unittest discover -s tasks/unpick-a-seam -p 'test_*.py'
```

`VALIDATION.json` records both fresh-start trials, source hashes and the narrow
post-processing correction to an obsolete audit counter. No physics or video
was changed. Public evidence keeps exact scoring fields and native mesh bounds;
full original trace hashes are retained. Reproduce both scores without a GPU:

```sh
python tasks/unpick-a-seam/assess_trace.py tasks/unpick-a-seam/evidence/demo --out /tmp/seam-demo-score.json
python tasks/unpick-a-seam/assess_trace.py tasks/unpick-a-seam/evidence/dream --out /tmp/seam-dream-score.json
```
