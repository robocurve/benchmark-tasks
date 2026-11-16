# Scene -- gondola-divider-planogram-respace

Station coordinates (m): `x` forward, `y` left, `z` up, origin between the arm bases; `TOP` = table top. Hole `k` of
the rear perforation row sits at `y = 0.0254 k`.

## Real product dimensions (catalog text)

Lozier 2016 shelves catalog, https://www.midwestretailservices.com/docs/Lozier02-shelves-2016.pdf (fetched
2026-11-15, text extracted with pdftotext):

- p.3 (bullnose shelf): "Standard perforations accept most shelf accessories with two rows of 1/4" diameter holes 1" on center".
- p.25 (Standard Wire Fencing): "Foot attachment creates tension allowing Fronts and Divider to be used independently",
  "Vertical wires on 1" centers"; fronts SWF "Shelf Width: 3', 4'", "Height: 3", 6"", "Standard Finish: ZNC".
- p.25 (Standard Wire Fencing Divider SWD): "Actual Divider length is approximately 2 3/4" less than nominal shelf
  depth"; "Height: 3", 6""; "Shelf Depth: 13", 15", 16", 17", 19", 22"".

Reference images for the builder-made shelf, fence and dividers: the catalog's p.25 Standard Wire Fencing photo (fronts
and dividers on a shelf) and p.3 bullnose shelf photo.

## Estimates (not in the catalog) -- method and uncertainty

The catalog gives no wire sizes or masses, and no measurement was made: the divider wire (4 mm) and fence top wire
(4.5 mm) diameters are estimates (uncertainty about +-1 mm), as are the foot (10 mm below the deck) and hook leg
(14 mm below the rail) lengths and the 24 mm cross-wire base at each end of the divider (+-30 %). The divider mass,
90 g, is computed from that estimated geometry: about 0.91 m of 4 mm steel wire at 0.099 g/mm. The tensioned foot attachment (p.25) has no published geometry or rate; the foot is modelled as a
plain 4 mm peg in the 6.35 mm hole (no preload). Wire flex is negligible: the 260 mm top rail of 4 mm steel wire
(E = 200 GPa) has a mid-span stiffness of 48EI/L^3 = 6.9 N/mm, so the 1.2 N divider weight bends it 0.17 mm; a 14 mm
hook leg is 4.4 N/um stiff. The glove-box mass (450 g, a box of 100 size-M nitrile gloves) is an estimate.

## Objects

| object | model / source | dimensions (mm) | mass | material | position |
|---|---|---|---|---|---|
| shelf deck (fixed) | builder-made from the catalog (p.3); `assets/visual/shelf.usdc` | 914 x 330 (36 x 13 in), sheet 1.2 (collider 4), two rows of 35 holes 6.35 on 25.4 pitch at x 152 and 390 mm | fixed | powder-coated steel, platinum grey; bullnose lip with a grey PVC label channel and blank labels; black bench stands | deck top at TOP + 15 mm, x 115-445, y -457..+457 |
| front wire fence (fixed) | builder-made from the catalog (p.25 SWF 3 ft x 3 in) | 76.2 high; top wire 4.5, bottom and vertical wires 3, vertical wires on 25.4 centres | fixed | zinc-plated steel wire (ZNC) | along x = 130 mm |
| outer dividers x2 (fixed) | builder-made (p.25 SWD 3 in x 13 in) | as below | fixed | zinc-plated wire | holes +10 and -10 |
| middle dividers `divider_l`, `divider_r` | builder-made (p.25 SWD 3 in x 13 in); `assets/visual/divider.usdc` | 260 long (front post to rear foot), top rail 78.7 above the deck running forward over the fence wire into a 14 mm hook leg; rear foot 10 below the deck; two stays; 24 mm cross-wire base at each end; 4 mm wire | 90 g | zinc-plated wire | start holes +2 / -2; planogram holes +3 / -3 |
| boxes of nitrile exam gloves `glove_l`, `glove_r` | NVIDIA SimReady `box_glove_a01` (CC-BY-4.0) | 178 x 117 x 56, lying flat | 450 g (estimate) | printed carton | centred at (0.229, +-0.165), on the deck |
| handheld barcode scanner in its cradle (visual only) | NVIDIA SimReady `computer_scanner_barcode_d01` | 329 x 429 x 211 placed bbox incl. cable | -- | plastic | bench behind the shelf, centre (0.64, 0.25) |
| tape dispenser (visual only) | NVIDIA SimReady `misc_tape_dispenser_a01` | 103 x 170 x 66 placed | -- | plastic, steel | (0.56, -0.23) |
| two spare glove boxes, stacked (visual only) | NVIDIA SimReady `box_glove_a01` | 178 x 117 x 112 | -- | carton | (0.69, -0.065) |
| planogram card (visual only) | builder-made, `assets_build/planogram_card.py` | 148 x 105 x 0.5 laminated card, lanes 7-6-7 drawn | -- | laminated paper | bench behind the shelf, (0.535, -0.06) |

Lanes: outer dividers at holes +-10 bound the planogram section; start lanes 8-4-8 holes, planogram 7-6-7 holes.
Every visual-only prop lies behind the shelf (x >= 0.48 m) where no arm waypoint goes (arms work at x 0.26 m,
|y| <= 0.08 m, park at (0.16, +-0.30) m, 0.20 m up).

## Mechanism the task needs

- Rear foot drops through a 1/4 in perforation (collider: 6.35 mm square opening; the 4 mm foot has 1.2 mm play per
  side) and front hook leg hangs in front of the fence top wire; the divider stands on its bottom rail and cross-wire
  base. Lifting 17 mm frees both (foot out of the hole, hook leg over the wire).
- Each moved divider is one rigid body on an unpowered six-axis joint chain (x, y, z slides; yaw, roll, pitch hinges
  about its centre; no springs, drives or joint friction; light damping against PhysX chain wobble), so it starts exactly seated (the runtime's reset jitter
  would otherwise push a seated foot into the deck). Gravity, contacts and the grippers move it.
