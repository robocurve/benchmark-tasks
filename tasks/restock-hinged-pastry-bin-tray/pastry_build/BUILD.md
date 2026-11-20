# Real-world build for the bakery bin task

The task needs a countertop display bin with an unpowered, gravity-closing, rear-hinged clear lid, a stainless inner tray, a second stocked tray, and a counter mat. No single off-the-shelf bin is claimed: the bin is simple enough to fabricate from flat sheet, and the files below give every part with its dimensions. The status of this design: the dimensions are taken from the simulation (`task.py`); a physical bin has **not** been built or tested by us.

## Files

| File | What it is |
|---|---|
| `make_cutsheet.py` | Writes `cutsheet.svg`, the flat parts for laser or saw cutting (15 parts) |
| `cutsheet.svg` | Cut sheet, millimetres, one rectangle per part with its name, size, thickness and material |

## Parts (millimetres)

| Part | Qty | Size (w x h x thickness) | Material |
|---|---|---|---|
| Floor slab | 1 | 200 x 200 x 8 | Stainless steel plate |
| Rear wall | 1 | 200 x 60 x 8 | Cast acrylic, clear |
| Side wall | 2 | 200 x 54 x 8 | Cast acrylic, clear |
| Front wall | 1 | 184 x 42 x 8 | Cast acrylic, clear |
| Hinge block | 2 | 14 x 14 x 16 | Stainless steel, bored for the hinge pin; one on each rear corner, hinge axis 94 mm behind the bin centre and 64 mm above the underside of the floor slab |
| Lid top | 1 | 194 x 196 x 7 | Cast acrylic, clear |
| Lid front skirt | 1 | 196 x 14 x 7 | Cast acrylic, clear |
| Lid side skirt | 2 | 194 x 12 x 7 | Cast acrylic, clear |
| Lid stiffener | 1 | 186 x 18 x 6 | Stainless steel strip, under the lid's hinge edge |
| Handlebar | 1 | 228 x 12 x 12 | Stainless steel bar, 18 mm forward of the lid's front edge |
| Handlebar standoff | 2 | 18 x 6 x 8 | Stainless steel, at 36 mm either side of the lid's centre line |

Bonding: solvent-weld or UV-bond the acrylic joints; screw the stainless parts. The walls stand on the floor slab; the 8 mm walls give an 18.4 cm clear width inside, which the arms' open fingers need.

Closed size: 200 x 200 x 75 mm; the lid closes onto the 54 mm side walls and the 42 mm front wall.

## Hinge

Two small stainless pin hinges at the rear corners (3 mm pin through the hinge blocks), free swinging through 0 to 85 degrees, with no spring and no closing aid: the lid closes by its own weight, which is why a person normally holds it up with one hand. Do not fit a gas strut or a damper.

| Check | How | Target |
|---|---|---|
| Lid mass | Weigh the finished lid with its handlebar and stiffener | 0.28 kg, plus or minus 0.03. If heavier, thin the lid top to 6 mm; if lighter, thicken the stiffener |
| Free swing | Lift the lid to 85 degrees by the handlebar and release it | It closes by gravity from any angle above about 5 degrees, with no sticking; hinge friction torque below 0.01 N m (a 1 N pull at the handlebar's front edge lifts it from closed) |
| Slow-motion rule | Film the lid's last 43 degrees of travel with a 240 frames-per-second phone video | Lowered by hand, the last 43 degrees (0.75 rad) take at least 0.42 s (speed below 1.8 rad/s). An unheld drop takes less than about 0.3 s and counts as a slam |

## Trays and mat

| Part | Spec |
|---|---|
| Empty tray | 98 x 72 x 24 mm stainless tray with a parchment liner, 0.22 kg |
| Stocked tray | The same tray carrying three pastry stand-ins, 0.36 kg in total (so 0.14 kg of stand-ins) |
| Counter mat | Slate or slate-coloured rubber, 130 x 102 x 4 mm |

## Pastry stand-ins (washable, repeatable)

Use stand-ins, not food: food crumbles, spoils and changes mass.

| Item | Spec |
|---|---|
| Shapes | Two cylinders of 23 mm diameter x 26 mm tall, one of 27 mm diameter x 28 mm tall, each with a lighter glaze cap (18 mm and 21 mm diameter x 7 mm) |
| Material | Cast silicone (Shore A 30 to 40), tan body and brown cap, with steel shot or iron powder mixed in to reach the mass below |
| Mass | About 47 g each, 0.14 kg for the three (plain silicone is only 12 to 20 g, so ballast is needed); weigh each and record |
| Placement | Centres at (-20, -14), (-20, +14) and (+17, 0) mm from the tray centre, on the liner |
| Attachment | Fixed to the liner and tray with silicone adhesive so they cannot move or fall off; the simulation also treats them as rigidly attached |
| Cleaning | Wipe with warm soapy water and dry before each session |

## Reset between trials

Return the lid to closed, seat the empty tray in the bin, put the stocked tray and the mat on their marks (left layout by default; see the README), and return both arms to the ready pose. Check the three stand-ins are still attached.
