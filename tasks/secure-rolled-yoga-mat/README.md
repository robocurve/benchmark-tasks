# Secure a rolled yoga mat in a two-loop carrying sling

Exercise mats spring open after rolling. An elastic sling keeps the loose turn
inside two separate end loops and lets someone lift the full mat by the
connecting shoulder webbing. The hands have to control the foam's springback
while fitting the soft loops over the unsupported ends. The
[Hugger Mugger mat guide][mat-sizing] documents full-size mat dimensions, and
[Sunshine Yoga][sling] and [Beemat][beemat] show commercial elastic end-loop
slings.

## Setup and reset

Use the two YAM arms 610 mm apart, an overview camera and both wrist cameras.
The worktop is a 1,100 × 1,300 × 60 mm board with a level upper face 747.5 mm
above the floor. Coordinates are measured from the midpoint between the arm
bases with X forward, Y left and Z up. Put a 181 × 409 × 17 mm rubber grip
pad at (391, 0) mm, with its long dimension parallel to Y. The support is flat
and leaves the outer 100 mm of the mat clear at each end.

Use a 1,780 × 610 × 4.5–5 mm rollable closed-cell foam exercise mat with a
dry, grippy surface and roughly 900 g total mass. Roll it by hand around
a finger-width hollow core. Set the unrestrained roll horizontally in the
middle of the grip pad, its ends in the Y direction. The compact roll is
about 107 mm across, and the loose outer turn stands about 9 mm above it.
Keep the exposed edge at the upper side. The rest is gravity-supported and
has no straps touching it.

Use an elastic-loop shoulder carrier with 28 mm flat webbing. Each elastic
opening is approximately 123 mm wide by 120 mm high when held upright
without stretching it, and the two end loops are linked by about 900 mm
of webbing, including the 95 mm long center pad. Set the sling in front of
the mat with both end loops empty and roughly parallel to the mat's cross
section. Set the shoulder pad flat at (180, 0) mm. Put each loop on its
lower edge around X = 295 mm, centered near Y = ±347 mm. Let the webbing
lie slack on the worktop between each loop and the pad. Leave both grippers
open and clear of the objects.

To reset, take the end loops off, untwist the shoulder webbing and put it at
its starting marks. Reroll the mat, set the roll on the pad and let the loose
turn recover. If either loop is visibly twisted, flatten the webbing before
resetting it.

## Score one trial

Use the calibrated trial video and a tape measure. Measure from each end of
the mat to the midline of that end's loop. A band is seated when it surrounds
the complete mat cross section on its half of the roll and is 40–145 mm
inboard of the nearest end. The unsupported mat must clear the raised
support by at least 35 mm for 0.6 seconds, with the grippers visibly clear
of the foam and both bands moving at most 12 mm along the roll during that
interval. After setting it down, watch for 0.7 seconds to confirm that the
arms have let go, both bands remain seated and the exposed turn stands at
most 8 mm from the compact roll.

| Outcome | Score |
| --- | --- |
| One seated loop | 0.25 |
| Two seated loops | 0.50 |
| Two seated loops and the unsupported sling lift is demonstrated | 0.75 |
| Both loops remain seated after the lift, set-down and release checks | 1.0 |
| All other outcomes | 0 |

Either arm can support the mat or fit either end loop. The unsupported lift
can be performed with either arm holding the shoulder portion of the sling,
provided both bands retain the mat.

## Run

Use the supported Robocurve Isaac Lab-Arena environment from this task
directory. The fixed seed is 1415158438. The run budget is 1,380 actions at
20 Hz, for 69 seconds of simulated control time.

```sh
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy one-loop --output runs/one-loop
```

The runner writes three 640 × 360 camera recordings at 20 fps, initial
stills, action-by-action state and scoring, and an environment receipt.
The scripted reference reached 1.0 at action 1,273. The stationary control
scored 0.0 through all 1,380 actions. The one-loop control also scored 0.0.

The Inspect Robots environment and policy entries are `yoga_sling` and
`yoga_sling_scripted`.

## Simulation fidelity

The mat's recoverable outer turn is a single sprung sector around the
remaining roll. Each elastic band has six contact-bearing webbing segments,
four local bending joints and an aperture spring. Ball joints connect the
sling's shoulder segments and end bands. The YAMs act through native contact
with these surfaces. The simulator represents foam and webbing compliance
through the articulated geometry rather than individual fibers.

## Third-party assets and licenses

The exercise mat, rubber support pad, elastic loops and shoulder webbing are
procedural geometry released under this task's MIT license.

| Asset | Source and applicable license |
| --- | --- |
| Two I2RT YAM arms and parallel grippers | [YAM adapter][adapter], Robot meshes under the [I2RT MIT license][i2rt]. |
| Split dark and light YAM link meshes | [YAM adapter][adapter] runtime meshes generated from I2RT model links, [MIT][i2rt]. |
| Board, floor, dome light and lamp | Procedural station primitives supplied by the [adapter][adapter]. The adapter has no declared repository software license. Isaac Lab-Arena is [Apache-2.0][arena]. |
| Left and right wrist cameras | [YAMLAb calibration][calibration] under [MIT][yamlab-license], instantiated by the adapter. |
| Overview camera | [YAMLAb optics][calibration] under [MIT][yamlab-license], with a task-authored aim and position. |

[mat-sizing]: https://www.huggermugger.com/pages/yoga-mat-guide
[sling]: https://www.sunshineyoga.com/elastic-carrying-straps-for-yoga-mats/
[beemat]: https://www.beemat.co.uk/collections/mat-storage-and-accessories/products/yoga-equipment
[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[i2rt]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[arena]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[yamlab-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE
