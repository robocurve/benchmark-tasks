# Technical reference

Dimensions and material values below distinguish published figures, measurements reported by sources, and estimates. Product pages identify the physical objects; no manufacturer assets are bundled.

## The objects, and where their numbers come from

| Quantity | Value used | Source | Status |
| --- | --- | --- | --- |
| Storage drawer, outside | 432 front to back x 365 wide x 175 high mm | Sterilite 16 Qt. Stacking Drawer, model 2301, maker's page <https://www.sterilite.com/product/16-qt-stacking-drawer/> ("17 x 14 3/8 x 6 7/8 in") | read on the page |
| Drawer, inside at the bottom | 416 x 302 x 124 mm | the same page ("16 3/8 D x 11 7/8 W x 4 7/8 H in") | read on the page |
| Unit weight | drawer 0.50 kg; frame fixed | the same page gives a shipping weight of 19 lb, which is for a case of several units; the number in a case was not read | estimate |
| Top of the drawer's sides | 146 mm above the table | drawer floor taken as 22 mm above the table plus the published 124 mm | estimate; measure on the unit |
| Underside of the frame's top at the opening | 152 mm above the table | assumed 6 mm above the drawer's sides | estimate; measure on the unit. This number sets how far the pillow stands above the opening |
| Drawer front, pull, stop | front panel 343 wide, 12 to 148 mm above the table; pull a lip 120 wide standing 12 proud; stop at 300 mm | assumed from the product's general shape | estimate |
| Force to slide the drawer | 3 N | assumed for a light plastic drawer on plastic runners carrying 0.3 kg | estimate |
| Wall thickness | 8 mm in the model | real moulding about 3 mm; published outside and inside sizes are kept | modelling choice |
| Pillow, lying free | 460 x 330 x 150 mm, 325 g | Therm-a-Rest Compressible Pillow Cinch, Regular, retailer page <https://www.altitude-sports.com/p/thermarest-compressible-pillow-cinch-regular-tar-14224> ("18x13in/46x33cm", "Thickness: 6in/15cm", "325g/11.5oz", urethane foam fill, polyester cover) | read on the page |
| Pillow's profile in the drawer | 150 mm thick over the middle 205 mm, 140 mm for 51 mm either side, 118 mm at the two ends | assumed | estimate; measure with a ruler across the drawer |
| Pillow's firmness | 60 N/m and 6 N s/m per pad (16 pads): a 1.5 kg book on the middle sinks to about the rim; springs back in about 0.3 s | assumed; the pillow's cinch cord adjusts firmness | estimate |
| Friction | frame and drawer 0.20, pillow cover 0.40 (average with the other surface) | assumed | estimate |

Other sizes of the pillow range (Small, about 380 x 280 x 127 mm) do not stand above this drawer's opening;
the plain Therm-a-Rest Compressible Pillow is 102 mm thick and does not either. No part number is claimed
beyond the model names above. No product image or mesh is included.

### How the pillow is modelled

PhysX has no cloth or foam here. The pillow is a base slab (410 x 296 x 20 mm, 85 g) lying loose in the drawer
with 16 rigid pads on it, 8 rows from the drawer front by 2 columns, each 48 x 143 x 25 mm and 15 g, on its own
vertical slide with a spring, a damper and 60 mm of travel. Left alone, the rows' tops are 140, 162, 172, 172,
172, 172, 162, 140 mm above the table; the opening is at 152 mm. Pads collide with fingers, drawer and frame,
not with each other. A row whose top is above 152 mm meets the front edge of the frame's top face to face and
stops the drawer; pad edges are square, so pushing harder does not help. A row pressed below 152 mm slides
under, and the frame's top then holds it down. With the drawer out 220 mm, rows 0 to 3 are in the open and the
drawer catches at openings of 214, 163 and 112 mm (rows 3, 2 and 1); row 0 is below the opening.

What is drawn: the pads' and the base's boxes are hidden and `skin.py` draws one closed cushion shape in a
blue-grey fabric colour, 410 x 296 mm in plan, with rounded edges and a seam 50 mm above the drawer floor. Its
upper side lies 1 mm under the pads' tops and is recomputed from their measured heights at every control step,
so a pressed part shows as a hollow and the part under the frame's top as flattened. Toward the pillow's four
edges the drawn surface curves down below the pads' flat tops (by up to 4 mm where the fingertips press, more
in the last 30 mm). The surface has no collider, is not read by the scorer, and the first recorded frame already
shows it. The unit is drawn as a white frame with a clear drawer and a moulded pull at the true outside sizes;
the frame's stacking recess, ribs and the drawer's taper are not modelled.

What this leaves out: a pad is rigid, so once its leading edge is under the frame the whole 48 mm is held,
and the pillow does not bulge between the fingers; a real pillow dimples under a fingertip, its cover wrinkles
and can snag in the gap. Holding one row down takes about 3.5 N; the rows already under the frame press up on
it with up to 15 N, which adds up to 4.5 N of drag to the 3 N of the runners.

To regenerate the asset after editing `layout.py`: `python tools/build_scene.py`.



## Real-world build list

| Item | Details | Cost |
| --- | --- | --- |
| Storage drawer | Sterilite 16 Qt. Stacking Drawer, model 2301 (white frame, clear drawer) | not priced here |
| Pillow | Therm-a-Rest Compressible Pillow Cinch, Regular (18 x 13 x 6 in) | not priced here |
| Clamps and a stop | two clamps on the frame's sides and a batten behind it; the frame alone weighs under 1 kg and would slide | small |
| Book | 1.5 kg, about A4, to set the pillow's firmness | none |
| Tape, ruler | marks for the frame, the drawer's start line | none |

Before the first trial measure and note: the height of the underside of the frame's top at the opening, the
height of the pillow's top along the drawer with a straightedge across the drawer's sides (it should stand 15 to
25 mm above the opening over the middle), and the force to slide the drawer with a spring balance. Set the cinch
cord so that the book on the pillow's middle brings its top to about the rim.



## Third-party assets and licenses

- All geometry is boxes generated by `tools/build_scene.py`; no meshes, textures or images from any third party
  are included. Product names are used only to identify what was modelled.
- Robot, station and cameras: `yam-arena` (the yam-isaac-lab-arena repository; its README states that it has no
  license yet) with I2RT's YAM meshes (MIT). They are dependencies, not copied here.
- Inspect Robots, inspect-robots-yam and the Dream client are installed dependencies under their own licenses.
- The code in this folder is under the MIT license (`LICENSE`).


## Full simulation fidelity statement

Isaac Lab-Arena (PhysX, 240 Hz physics, 20 Hz control). Frame fixed; drawer one rigid body on a slide whose 3 N friction is a force-capped damper. The pillow is not cloth: a loose base slab carrying 16 rigid pads (8 rows of 2, each 48 x 143 mm) on vertical springs (60 N/m, 6 N s/m each), which collide with fingers, drawer and frame. It is drawn as one fabric surface laid over the pads and following their heights; the surface has no collider and the pads' boxes are not drawn. A pad is rigid, so once its leading edge is under the frame's top the frame holds all of it down; a real pillow bulges more locally and its cover can wrinkle or snag. Unit outside sizes and the pillow's size and weight are published; heights inside the unit, the pillow's profile, stiffness and frictions are estimates. Walls are 8 mm thick (real about 3). No software rule moves, holds or releases anything. Not run on hardware.
