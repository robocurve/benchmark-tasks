# Load a printed page into a clear sheet protector

File a loose Letter handout in a clear top-loading sleeve. One YAM arm
opens and holds the thin film while the other brings the page level with
the mouth, registers both leading corners, and feeds the sheet under the
film. The film can close after the sheet reaches the bottom seam.

## Setup and reset

Use the standard YAM worktop at 747.5 mm above the floor. Positive X
points forward and positive Y points left. Use a [clear top-loading
Letter protector][protector] in an open three-ring binder. The protector
is polypropylene. Its modelled internal cavity is 289 × 222 mm with
a 15 mm reinforced binding margin and a 9 mm upper lip overhang. Use
a binder with two roughly 284 × 250 × 6 mm covers and a 9 mm stack of
filed copy paper under the lower layer. Set the binder on an ordinary
approximately 290 × 500 mm open-book rest inclined 24° to the tabletop.
Its low ledge supports the bottom of the binder. The centre of the
sleeve entrance is at X 481 mm, Y 5 mm and Z 888 mm. The opening faces
the robots. The pocket slopes down away from them.

Print a single page on 120 g/m² white Letter presentation paper, about
279 × 216 mm and 7.2 g. Set it print-side up on a separate plastic
document stand about 266 × 226 mm, also inclined 24°. Centre the trailing
paper edge at X 207 mm, Y −240 mm and Z 969 mm. Leave 22 mm of that edge
past the backing so the pad can reach both faces. A short ledge holds
the far end of the page while it sits on the stand. The loose sheet
starts fully outside the sleeve, and both grippers start open.
For reset, take the document out, replace it if creased, restore the
marked positions and let the sleeve close before opening both grippers.

## Run and score

Run from this folder in the Robocurve YAM simulation environment.

```sh
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy on-top --output runs/on-top
```

The fixed reset uses seed 1415158438 and 480 Hz native physics. A trial
lasts up to 760 actions (38 seconds at 20 Hz). The reference reads
the native paper-link poses to guide the corners and sends joint-position
and gripper commands through the standard action interface. The runner
records three native 640 × 360 views and writes the result, per-step
measurements, and source fingerprint. The wall-clock run time depends
on rendering and the GPU.

Use a ruler and top and oblique photos or video to judge the score.
Clearance means the closest gripper pad is at least 35 mm from the sheet.

- **1.0** when the sheet is entirely inside and under the top film.
  Both leading corners are within 9 mm of the bottom seam. The sheet
  is within 6 mm of flat along the sleeve, with no bent corner, and
  the lip has closed to within roughly 5.5 mm of its flat position.
  Leave both grippers clear and the page still for at least 0.45 seconds.
- **0.5** when at least the leading third (93 mm) is inside and beneath
  the film and the sheet is stationary and released, with both grippers
  clear.
- **0.0** otherwise. Leaving the sheet above the film or on its separate
  stand earns 0.0.

## Simulation fidelity

Paper is a native three-link elastic sheet, which can sag
and buckle longitudinally against the sleeve. Its two passive flexures
have a nominal restoring rate of 0.0105 N·m/rad and a ±48° travel limit.
The film opens with a passive flexure, and the welded sides are represented
by narrow contact envelopes following the closure. The paper and film
are drawn at their thin-sheet scales and use 0.54 and 0.55 mm collision
skins to stabilize native contacts. An optically thin surface renders
the clear pocket over the independent collision layers. The model uses a 9 mm easy-load lip
overhang and estimated material parameters. The stands support their
respective items. The gripper opens the sleeve and crosses both corners
under the film before gravity can finish seating the sheet.

## Third-party assets and licenses

The task-authored sheet, print, binder, film, stand and joint geometry
use the included MIT license. Avery specifies the commercial format and
supplies no mesh, texture or logo bundled here. The runner and YAM
simulation setup reuse the same team's `tasks/core-an-apple` scaffold.

| Asset | Source and use | License |
| --- | --- | --- |
| I2RT YAM arm, gripper geometries and split arm appearance | I2RT URDF and meshes supplied by the [pinned adapter][adapter] | [MIT][i2rt-license] |
| Station worktop, floor and lights | Procedural assets from the [station scene][room] | The adapter has [no declared repository license][adapter-license]. The runtime is [Apache-2.0][arena-license]. |
| Clear-film shader | Built-in OmniPBR shader in the Isaac Sim renderer, configured by the task | Provided by the Isaac Sim installation. |
| Overview and wrist cameras | [YAMLAb calibration][calibration], supplied by the adapter, with the overview pose authored by this task | [MIT for calibration][calibration-license] |

[protector]: https://www.avery.com/products/sheet-protectors/75540
[adapter]: https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730
[adapter-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#license
[i2rt-license]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt
[room]: https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py
[arena-license]: https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md
[calibration]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml
[calibration-license]: https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE
