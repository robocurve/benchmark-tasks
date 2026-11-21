# Pry the connected caps off a filled eight-tube strip

A linked cap strip seals a row of small laboratory tubes. The gap beneath its
webbing is narrow, and the removal tool must lift against eight plug fits while
the retainer stays on the bench. The [Applied Biosystems removal procedure][guide]
uses two small protrusions under the cap webbing and a slow pry. The reference
uses one YAM arm to hold the retainer and the other to engage, pry and move the
connected caps to an empty dish.

> Use the removal tool to uncap the eight-tube strip. Keep the tubes seated
> upright and put the connected cap strip in the empty dish.

## Setup and reset

The scene uses a YAM station with two opposing grippers, a fixed overview
camera and two wrist cameras. Bench height is 747.5 mm. World X runs forward
from the bases, Y runs toward the left arm, and the tube row runs along Y. The
objects start with zero jitter.

| Object | Parts and size | Starting centre in bench coordinates |
| --- | --- | --- |
| Tube strip and carrier | Eight polypropylene tubes at 9 mm pitch, forming one row of nominal 0.20 mL tubes. For the physical trial use [Applied Biosystems MicroAmp Optical 8-Tube Strip, 0.2 mL][physical-tubes], catalog no. **4316567**. The dry 76 × 117 × 33 mm ABS retainer assembly weighs about 148 g and sits loose on the bench. | X 371 mm, Y 60 mm |
| Cap strip | For the physical trial use one clear [Applied Biosystems MicroAmp Optical 8-Cap Strip][physical-caps], catalog no. **4323032**. It has eight flat-top press-in caps joined by webbing and lifts off as one piece. The modeled strip has a 9 mm pitch, 2.7 mm insertion depth, 6.4 mm flanges and 1.8 g mass. | Fully seated on the eight mouths |
| Removal tool | 85 × 40 × 12 mm polymer hand tool with two 1.7 mm wide prongs, 0.8 mm thick, spaced 36 mm apart. The rails and crosspieces form an open handle. | X 245 mm, Y −183 mm, long axis parallel to X, teeth face +X |
| Discard dish | Empty 114 × 105 × 14 mm polymer dish. | X 248 mm, Y −306 mm |
| Liquid | 100 µL food-coloured water per tube. | All eight tubes, with no loose liquid on the bench |

Seat the specified tube strip in the loose tray/retainer and use a [MicroAmp
Multi Removal Tool (4313950)][tool] for the physical trial. The manufacturer
lists the cap strip and tube strip together in its [consumables guide][compatibility].
The dimensioned tool and retainer are defined in `strip_uncapping/builder.py`.
The retainer can be printed in ABS or PETG. Use its outer walls as the grasp
surface.
Check that the tool's prongs insert beneath the webbing and both tips bear on
it before the handle reaches the tube rims. Fill the tubes with food-coloured
water for a physical trial. After each run, refill any depleted tube, seat all
eight caps, wipe the dish and bench, and return each object to its listed
position.

## Scoring

Use the fixed and wrist views to count exposed mouths and check the cap strip.
The carrier stays within 15 mm of its original bench position, its base rises
at most 4 mm and it never tilts more than 8°. Put a scale beside it in the
side view to check those limits. Each tube must retain at least 90 µL of its
100 µL fill. A matching marked comparison tube or a graduated micropipette
can check the remaining liquid after a physical trial.

- **1.0** when all eight mouths are exposed, and the intact strip of caps
  rests fully within the dish for 0.5 s.
- **0.5** when eight mouths are exposed and the cap strip has yet to settle in
  the dish.
- **0.25** when at least four mouths are exposed.
- **0.0** on any carrier or liquid-limit violation, or fewer than four open
  mouths.

One long cap band comes out together. The criteria permit either hand to
hold the retainer, alternative tool grips and small rack movement.

## Run and score

Use the Robocurve YAM and Isaac Lab-Arena environment. Install this package
into that environment and run the local reference from this folder.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --policy hold --no-cameras --output runs/hold
python run.py --device cuda:0 --physics-device cpu --policy unbraced --no-cameras --output runs/unbraced
```

The default seed is 1415158438. The reference uses native rigid-body poses for
tool registration and joint feedback for action tracking. It sends ordinary
joint targets. Its setup screenshot, trace, three 640 × 360 views and final
score are recorded in the output directory. Control runs at 20 Hz with 960 Hz
physics. The maximum trial is 480 actions or 24 seconds. The two controls test
stationary execution and the same extraction motion without counterholding.

The rendered reference scored 1.0 at action 437. All eight 100 µL fills
were retained. The carrier lifted 0.13 mm, shifted 7.09 mm and tilted 1.10°
at most. The stationary control and the unbraced pry both scored 0.0.
Without counterholding, the retainer lifted 165 mm and tilted 61°.

Inspect Robots registers the task as `strip_uncapping` and the local reference
as `strip_uncapping_scripted`. For hosted evaluation, use the `dream` policy
with the same instruction, render at 320 × 176 per camera and retain the
480-action limit.

## Simulation fidelity

The tool prongs, tube rims, cap strip and retainer have native contact geometry.
Each plug contributes force at its location. The model applies equal and
opposite loads to the cap strip and retainer as the prongs lift the captured
webbing. The force grows during the first millimetre of withdrawal and falls
through the final millimetre of plug travel. The modeled fit peaks at about
0.58 N per cap. That parameter is an engineering estimate. The connected cap
webbing is rigid, and the tubes and latched retainer move as a compound body.
The fill model derives retained volume in each open well from the bore
dimensions and tilt. It resolves quasi-static overflow without droplet
dynamics.

## Asset licenses

All tool, retainer, tube, caps, water and dish geometry is authored here and
covered by the included MIT license. The manual and commercial items above
document the real procedure and specify the physical consumables. The simulated
dimensions and loading law describe the bench model.

| Asset | Source and licensing |
| --- | --- |
| YAM arm, parallel grippers and collision hulls | I2RT YAM at commit `120c3c81400171174604e503943f8d1ebc891058` with the MIT [upstream license][i2rt-license]. The pinned adapter converts this combined rig for the runtime. |
| Black and white visual links 2 and 3 | Distributed by the adapter with [I2RT source attribution][split-links] and the [MIT license][i2rt-license]. |
| Station floor, worktop, dome and lamp | Procedural primitives from the [YAM adapter][adapter]. The adapter's [license status][adapter-license] is documented separately from the MIT robot assets. Isaac Lab-Arena supplies the lighting runtime under [Apache-2.0][arena-license]. |
| Fixed and wrist cameras | Procedural adapter stations and MIT-licensed [YAMLAb calibration][calibration] with the [calibration license][calibration-license]. The overview viewpoint is set in this task. |
| Overview camera housing | Procedural [adapter station primitive][station] with the adapter's [declared license status][adapter-license]. The camera is suspended overhead. |

[guide]: https://assets.thermofisher.com/TFS-Assets/LSG/manuals/4485764_dual_96std_starter_kit_UG.pdf#page=13
[tool]: https://www.thermofisher.com/order/catalog/product/4313950
[physical-caps]: https://www.thermofisher.com/order/catalog/product/4323032
[physical-tubes]: https://www.thermofisher.com/order/catalog/product/4316567
[compatibility]: https://assets.thermofisher.com/TFS-Assets/LSG/manuals/100070431_miniamp_miniampplus_starter_kit_UG.pdf#page=10
[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[split-links]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE
[arena-license]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[station]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/station.py
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE
