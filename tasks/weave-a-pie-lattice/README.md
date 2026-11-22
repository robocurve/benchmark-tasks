# Weave a pastry strip into a pie lattice

Weave one loose strip over one strip and under the other on a pie, then leave
all three intact, flat and spanning the crust. This tests the basic weaving
operation; completing a whole lattice top, trimming and baking are outside the
task. No crossing is made in advance and no strip end is fixed.

## Setup and reset

Use a loose aluminium tart pan, 180 mm inside diameter, 190 mm rim diameter and
36 mm deep, with a bottom pastry shell and commercially canned apple pie
filling level 1–2 mm below the rim. Prepared mass is about 550 g. The simulator
treats pan, shell and filling as one rigid body.

Use Pillsbury refrigerated pie-crust dough from the same product and batch for
all trials. Temper the sealed dough for 15 minutes at room temperature, as
[Pillsbury directs](https://www.pillsbury.com/products/pie-crust/refrigerated-pie-crust).
Unroll on baking parchment, roll between 2 mm guides and cut three 250 × 25 mm
strips with a ruler and pastry wheel. Keep them covered at 2–4 °C; take a set
out five minutes before a trial in a 20–22 °C room. Do not flour, glue or
pre-crease them. Reject cracked, stuck or dry strips. Record product, batch,
temperature and preparation intervals. Prepare several sets together and use
fresh strips and a fresh level surface each attempt. This preparation protocol
has not been physically validated.

Place two strips north–south across the pie, their centrelines 64 mm apart.
The third lies east–west on a 180 × 180 × 42 mm wooden board to the left,
50 mm south of the pie centreline, overhanging each end by 35 mm. Nothing is
attached. Pie centre is (350, 25) mm and board centre (130, 25) mm; worktop
height is 747.5 mm. The board and filling surfaces are level. Exact generated
geometry is in `geometry.py`.

Both arms use smooth stainless finger extensions: 40 × 6 × 1 mm working leaves
and 100 × 8 × 1 mm stems, specified in `pad_covers.py`. Round and deburr edges.
For physical trials, cover grippers and wrists with food-safe sleeves and use
a washable catch mat. Sleeves must not bridge the jaws. Wash extensions and
replace sleeves between attempts. Do not eat evaluation food. Sleeves and mat
are physical-rig protection and are not rendered.

Before physical trials, manually check that prepared strips lift and lay
without sticking or cracking, and that the extensions grip without cutting.
Record rejected preparation; do not quietly stiffen dough or add supports.
No physical rig or material calibration is claimed.

## Scoring

Judge after both grippers are open and 50 mm clear for three seconds, using
overhead/oblique photographs, video and a ruler.

- **0.5:** the cross-strip passes over one lengthwise strip and under the other,
  with both crossings within 80 mm of the pie centre. Either order is valid.
- **1.0:** the above, and all three strips rest flat and reach the crust at both
  ends. Each end reaches the 86 mm-radius inner crust and extends at least
  60 mm to its side of the centre along its main direction (north–south for
  the original two strips; east–west for the cross-strip). Portions over the
  filling must be no more than 12 mm above its surface.
- **0:** no complete over/under pair, hands not released, any strip torn through
  or lost off the work area, or the pie tilted more than 30 degrees at any time.

The actual ends determine crust reach. Curved strips and overhanging tails
are allowed; straightening and trimming are not tested. A U-shaped strip with
both ends on one side does not span the pie. The scorer checks real projected
crossings and height order, not proximity to target poses. Its submillimetre
crossing guards distinguish the two layers in the 2 mm model; a real judge
only checks which strip visibly lies above the other. Partial credit separates
a completed weave from finishing its placement.

## Simulation limits

Each strip has 24 contact-bearing sectors with passive bend/twist joints,
gravity, collision and friction. Joints can tear; finite native grips can
slip. No scripted fold, snap-to-weave or body-position write follows reset.
Stretch, plastic flow, tackiness and temperature dependence are omitted.
Stiffness, the 2 N tear threshold and 0.8 N grip limit are nominal, uncalibrated
approximations, not measurements of the named dough. Pan, filling and shell
are one rigid body. The retained skill is lifting, under-passage and replacing
loose flexible material without tearing or undoing the crossing.

The scripted demo reads measured poses but outputs only ordinary robot joint
commands. Dream and the demo use the same task, hardware and physics.

## Novelty and sources

[King Arthur's lattice method](https://www.kingarthurbaking.com/blog/2021/07/01/how-to-make-a-lattice-pie-crust)
lifts strips, places a cross-strip and replaces them. This task isolates one
such operation. [Pillsbury](https://www.pillsbury.com/everyday-eats/desserts/pies-tarts/how-to-make-pie)
also describes lattice tops using refrigerated pie crust.

Public searches for lattice, pastry and weave on 22 November found no direct
Robocurve match. BEHAVIOR's
[`make_pastry`](https://behavior.stanford.edu/knowledgebase/tasks/make_pastry-0.html)
combines ingredients in a food processor. Related
[hair-braiding task #2296](https://github.com/robocurve/benchmark-tasks/pull/2296)
interlaces sections anchored at a head; this task passes a loose strip through
an orthogonal crossing and replaces its loose support. These bounded searches
do not guarantee novelty across every benchmark.

## Run and verify

Use the pinned Python 3.12 / Isaac Sim 6.1 / YAM runtime in `ENVIRONMENT.json`.

```sh
python tasks/weave-a-pie-lattice/make_plan.py
OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y python tasks/weave-a-pie-lattice/run.py --mode demo --out /tmp/lattice-demo
OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y python tasks/weave-a-pie-lattice/run.py --mode dream --out /tmp/lattice-dream
python tasks/weave-a-pie-lattice/rescore_current.py /tmp/lattice-demo
python tasks/weave-a-pie-lattice/rescore_current.py /tmp/lattice-dream
```

Limit: 1,200 controls / 60 seconds at 20 Hz. Fixed layout, no reset jitter or
random seed; offline IK search seeds are 17, 31 and 43. Runtime saves every
native frame, actions, body traces and frozen source. Rescoring verifies that
source before checking the outcome. CPU checks are not native evidence.
Recorded wall times and audit results are below.

## Asset attribution

Task props and finger extensions are procedural boxes, cylinders and rings
from `geometry.py` and `pad_covers.py`, under this folder's MIT licence.
No downloaded food, kitchen or fabric model is used.

The external pinned YAM runtime supplies the following I2RT robot assets,
under its [MIT asset licence](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt):

- `assets/base.stl`, `gripper.stl`, `link1.stl` through `link5.stl`,
  `tip_left.stl` and `tip_right.stl`, plus their convex decomposition files
  in the corresponding `collision/<link>/` directories.
- Dream-style split visuals `dream/link2_black.stl`, `link2_white.stl`,
  `link3_black.stl` and `link3_white.stl`. The pinned runtime's
  [station module](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/station.py)
  attributes these parts to Dream's station and identifies the same MIT terms.

The robot USD is converted from those assets. The table, floor, camera bodies
and mast are primitive geometry in the external runtime. Robot meshes and
runtime source are not redistributed in this task folder. `inspect-robots-yam`
0.38.0 separately carries an MIT package licence and its MIT I2RT
`assets/MENAGERIE_LICENSE`; all runtime versions are pinned in `ENVIRONMENT.json`.

## Recorded validation

The native demonstration rescores **1.0 in 1020 controls**, with the over–under pair complete, all three strips intact and both hands clear. The matching Dream baseline scores **0.00 in 1200 controls**. Frozen-source rescoring, measured finger-contact audits and every-frame video decoding pass. Recorded elapsed time on an L40S was 11.4 minutes for the demo and 20.8 minutes for Dream.

The Dream key remained on the local computer through the official loopback SDK bridge. API session recoveries continued the same uninterrupted physical episode. Full uncut native videos and the archived traces support simulated feasibility; no physical dough calibration or kitchen trial is claimed.
