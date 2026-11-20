# Renew a lint roller and clean lint off a cloth

Hold the roller by its ordinary handle, lift the edge of one used sheet and
unwind it to the perforation. Roll the fresh adhesive over the blue swatch
while keeping the fabric flat. The task tests sustained tension at a moving
peel front and loaded rolling contact on a loose fabric workpiece.

The commercial reference is a 4 in Scotch-Brite household lint roller with
perforated removable sheets. The newer seam overlaps the previous sheet by
about 1/8 inch. An easy-peel liner uses a low-tack starter zone, with a
shallow raised corner for pinching. Lift the dry corner, unwind the sheet
to its perforation and remove it to expose fresh adhesive.
[3M roller][roller] · [3M seam geometry][seam] · [3M low-tack liner][easy-peel]
· [Embossed starter design][embossing]

## Setup and reset

Use a hand roller of the dimensions below, one pale used sheet and a clean
sheet beneath it, a blue felt or tight-woven cotton swatch, 20 countable pale
cotton tufts, and a non-slip work surface. Use 3–6 mm tufts cut from the same
cotton yarn for each trial. Preload the outer sheet with enough of the same
fibres to cover the contact surface, still leaving the factory overlap
visible. Leave the cloth loose so it can be steadied by a gripper.

Coordinates are relative to the midpoint of the two YAM pedestal mount
points projected to the floor. The positive X axis points into the bench.
Positive Y is left from the robots' perspective. The worktop is 747.5 mm
above the floor datum.

| Object | Size, material and starting arrangement |
| --- | --- |
| Worktop | Existing 1.10 × 1.30 m YAM worktop. Two parallel-jaw arms on the usual pedestals. |
| Lint roller | Cylindrical adhesive portion Ø50.8 × 101.6 mm, centre X 259 mm and Y −34 mm. Its rotation axis is parallel to Y. A plastic handle Ø28 × 80 mm lies along +Y with the ordinary grip centred 126 mm from the roller centre. |
| Used sheet | A single complete used circumference with 3.2 mm overlap and a dry starter zone extending 19 mm from the separation line. Turn the split to face the robot. Orient the raised starter corner 34 mm along the right half of the roll and about 12 mm above the roller axis. The rest of the wrap starts adhered to the underlying sheet. |
| Blue cloth and lint | 220 × 218 × 4 mm felt swatch, centred X 426 mm and Y 20 mm, long edge parallel to X. Place 20 pale cotton tufts in a regular five-by-four array covering about 126 × 65 mm around its centre. |
| Roller rest | Support the head on a 73 × 110 × 30 mm rubber pad and the handle end on a 57 × 21 × 38 mm rubber pad. Put the end pad 169 mm along positive Y from the head centre. Two 6 × 85 × 7 mm rubber ridges on the head pad keep the unheld head near its reset mark. Leave the gap below the grip open. |

Each reset restores the roller and cloth to these poses with both grippers
open. Remove fibres from the cloth and reset the array. Restore the used
sheet and its seam orientation using the same roller type. For a flat
dry-edge liner, raise the outside edge of the dry corner about 12 mm
with a fingertip and give the leading 14 mm a shallow crease. Leave the
adhesive part completely wound on the roller.

## Scoring

Check still images and the trial video. A fully removed used sheet is free
from the roll. For positive credit, the face-up cloth remains on the
worktop within 50 mm of its starting centre.

- **1.0** when the used sheet is free and at least 18 of 20 original cloth
  tufts have transferred to the fresh roller for 0.25 s.
- **0.60** when the used sheet is free and at least 10 original tufts are
  retained on the fresh roller.
- **0.30** when the used sheet is free and the cloth is still in its work area.
- **0.0** for all other outcomes.

A dropped tuft or hidden surface gains no credit. Either arm can counterhold
or peel. Use marks on the table, photographs, and inspection of the fresh
sheet to reproduce the check in a physical trial.

## Run

Use the Robocurve Isaac Lab-Arena and Inspect Robots environment and install
this task in it. Begin with the known station calibration.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy dirty-roll --output runs/dirty-roll
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy peel-only --output runs/peel-only
```

The fixed layout uses seed 1415158438. Trials use 20 Hz control, 240 Hz
native physics and at most 480 actions, or 24 seconds of robot time. The
runner records one native video per camera, the reset frame and score at
every step, and writes a final receipt. For the reference pregrasp, the
runner reads the edge position from the simulator and uses it to adjust
the approach before the jaws close. The action is a YAM joint command.

```sh
inspect-robots run --embodiment lint_roller --policy lint_roller_scripted \
  --instruction "Remove the used outer sheet from the lint roller, then use the fresh layer to clean the lint off the blue cloth." \
  --seed 1415158438 --max-steps 480 --scorer task_score \
  -E cam_width=640 -E cam_height=360
```

## Simulation and fidelity

The real sizes, free-spinning spindle, easy-peel sheet seam, compliant
peel force and loaded pass are retained. A lightweight rigid body with a
2.2 mm effective contact thickness supplies the grippable corner. An
advancing flexible sheet mesh and damped tensile load represent the rest
of the backing. The load is applied oppositely to the two native bodies.
The 1.6 N unwind threshold is at the low end of 3M's measurements for
commercial-style 4 in lint tape. The dry starter has lower resistance.
The cloth has a native movable felt backing. Its compressible nap
supplies normal reaction and friction at the physical contact line.
Tufts are retained on the fresh adhesive when that contact line reaches
them during rolling. They remain visible on the roller. The spindle
uses a small viscous bearing drag. Source includes the geometry and
material parameters for rebuilding the scene.

## Third-party assets and licenses

The lint roller, sheet, cloth, table rest and fibres are generated from
original geometry under the included MIT license.

| Asset | Source and applicable license |
| --- | --- |
| I2RT YAM arm meshes, parallel grippers and collision hulls | I2RT commit `120c3c81400171174604e503943f8d1ebc891058`, converted through the [pinned YAM adapter][adapter]. [MIT][i2rt-license]. |
| Arm-link appearance | I2RT links 2 and 3 from the [adapter][split-meshes], split for black and white appearance. [MIT][i2rt-license]. |
| Station worktop, floor, lighting and camera stand | Procedural primitives from the [adapter room][room]. The [adapter has no declared repository license][adapter-license]. Isaac Lab-Arena is [Apache-2.0][arena-license]. |
| Fixed overview camera | Virtual RGB camera using [YAMLAb optics][calibration] and a task-authored pose. [MIT for calibration][calibration-license]. |
| Left and right wrist cameras | Virtual RGB cameras with [YAMLAb wrist calibration][calibration]. [MIT for calibration][calibration-license]. |

[roller]: https://www.3m.co.uk/3M/en_GB/p/d/v101241015/
[seam]: https://multimedia.3m.com/mws/media/2167113O/scotch-brite-lint-roller-digital-launch-kit.pdf
[easy-peel]: https://patents.google.com/patent/US20090226657A1/en
[embossing]: https://patents.google.com/patent/US7090908B1/en
[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[split-meshes]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE
[room]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py
[arena-license]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE
