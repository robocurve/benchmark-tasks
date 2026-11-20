# Prepare a blot-transfer cassette and roll out trapped air

An air pocket breaks contact between a soft gel and the membrane placed over it. The operator holds the wet stack in registration, advances a rubber roller to an exposed edge to vent the air, and closes the cassette without shifting or damaging the layers. One arm braces a dry carrier tab while the other operates the free-axle roller. The full-size cassette uses a hinged, perforated cover and an inert wet gel-and-membrane stack. [Bio-Rad cassette assembly and handling, pp. 5–7](https://www.bio-rad.com/webroot/web/pdf/lsr/literature/M1703930.pdf)

## Fixture and materials

All dimensions are in millimetres. [Cassette and reset template](fixture/cassette.scad) provides printable parts for the standalone mechanical cassette. Use the OpenSCAD part selector to export the base, cover, carrier, riser, saddle and pocket placement template individually. Print the wet-contact components in PETG or ABS. Fit a 3 × 120 stainless pin with snug end retainers in the hinge. Use a loose removable rear stop to hold the cover at 96°. Put 7 mm soft open-cell foam strips on the underside of the cover ribs so they retain the stack on closure. A caliper can verify the closed frame gap and stack fit before recording trials.

| Item | Construction |
| --- | --- |
| Cassette | 139 × 106 base, 124 × 95 perforated cover, 3 mm hinge pin. The pivot is 66 mm forward and 27 mm above the bottom centre of the base. Screw the base to the riser through the four holes using M3 hardware. |
| Carrier | The removable tray insert has exposed dry corner tabs and marks for the 100 × 75 working area. Place a 106 × 82 × 2 open-cell foam pad and 104 × 78 × 0.8 cellulose backing paper on it. |
| Fragile gel and membrane | Place a 100 × 75 × 4 plain, clear 1% by mass agarose-in-water gel slab on the backing paper. Use a 100 × 75 hydrated transparent regenerated-cellulose sheet, 0.2 mm nominal wet thickness. The agarose and cellulose serve as blank mechanical test material. |
| Roller | Use a 106 mm wide, 28 mm diameter smooth rubber brayer with a freely rotating axle, a bridge yoke and a 20 × 23 × 44 dry grip centred 81 mm above the axle. The saddle supports the two outboard shoulders of its yoke. |
| Pad protection | Fit thin, snug, wipeable gripper pad sleeves for a shared robot. Use the dry tabs, the dry roller grip and the cover grip. Wipe the handles and change sleeves if they have touched the wet area. |

The YAM table is at z = 747.5, with +x forward and +y toward the left arm. Put the tray bottom centre at (365, 25, 826) and the horizontal axle of the parked roller at (365, −116, 857), parallel to x. Set the upper faces of the two saddle seats 129 mm above the bench, 69 mm outboard of the axle centre and 34 mm deep along y. The tabletop-to-cassette dimensions are embodied in the CAD. Both arms start clear of the fixture.

## Repeatable reset

1. Place the pad and backing paper in the carrier and centre the gel, long edge parallel to x. Keep the side tabs and grips dry. Use a 1 mL graduated syringe to spread 0.50 mL of distilled water over the gel surface. Wet the blank cellulose sheet with a further 0.30 mL, then lay it down without stretching it so it matches the gel outline.
2. Hold the reset template 3 mm above the stack on temporary spacers, square to the carrier marks, to locate the three centres. The centre offsets from the gel centre are (−29, −22), (12, −2), and (28, 21) mm. Use a blunt 1 mm flexible tip on a graduated syringe, slid parallel to the layers from the nearer open side. Place nominal 40, 50 and 40 µL air aliquots at the respective centres and withdraw the tip along the same path. Remove the template before the episode. Use low angled light and a millimetre rule for the reset photo. Accept three separate pockets centred within 2 mm of the marks and within 2 mm of the nominal 14, 16 and 14 mm diameters. Reseat the sheet and repeat the reset if a pocket merges, leaks at the edge or falls outside this range.
3. Return the cover to its rear stop and place the roller on the saddle with its handle dry and upright. Record the initial pocket centres and the gel and membrane outlines in the overhead view. Use a fresh gel and sheet if either has a crease, tear or permanent indentation.

## Scoring

Use the overhead video with a millimetre scale at the side of the cassette. A pocket earns 0.25 when rolling visibly drives it to an exposed edge and vents it there. Award 1.0 when all three vent, the stack is within 3 mm of its registration marks and the cover rests within 2° of its closed position for 0.5 s. A torn, deeply indented or grossly shifted stack receives 0. Either arm can brace, roll or close. A closed cover or the disappearance of a pocket in the interior earns no pocket credit.

The simulator uses a spring-supported wet stack with native shear and vertical deflection, a free-axle rubber drum, and a native revolute cover hinge. The bounded air transport model advances a pocket only in the direction of a near-surface, loaded rolling nip that reaches it. The model caps advance by travel of the nip relative to the mobile stack. Each witness moves with its pocket and contracts as it approaches the edge, and is removed when its centre reaches the exposed edge. The shared springs represent the aggregate compliance of the soft layers and backing. Sustained excessive compression or shear latches damage in the scorer. The benchmark validates native mechanics and a reduced-order model of air transport.

## Run and score

Use the repository Isaac Lab Arena/YAM environment. The fixed reset has seed 1415158438, 20 Hz control and a 540-action (27 second) limit.

```sh
python -B run.py --device cuda:0 --physics-device cpu --output /tmp/cassette-demo
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output /tmp/cassette-hold
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy hover --output /tmp/cassette-hover
python -B run.py --device cuda:0 --physics-device cpu --no-cameras --policy short --output /tmp/cassette-short
```

Each run writes its reset snapshot, native trace and score receipt. Camera runs record the initial image and every action frame in three native 640 × 360 streams. The no-action control spans all 540 actions. The hover control handles the tool and cover without rolling on the wet stack. The short control rolls under load and then closes the cover while air is still inside the stack.

For Inspect Robots, install this directory with `python -m pip install --no-deps -e .`, then pass `blot_cassette` as the embodiment and `dream` as the policy. The hosted task uses the same seed, instruction and action limit.

## Assets and terms

| Asset | Provenance and licence |
| --- | --- |
| Tray and stand | Original procedural geometry, task MIT licence |
| Carrier, gel and membrane | Original procedural geometry, task MIT licence |
| Pocket witness graphics | Original procedural geometry, task MIT licence |
| Hinged cassette cover | Original procedural geometry, task MIT licence |
| Brayer | Original procedural geometry, task MIT licence |
| Brayer saddle | Original procedural geometry, task MIT licence |
| YAM arms and grippers | [Split I2RT geometry](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE), [I2RT asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) |
| Bench and station | [Adapter-generated geometry](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py), no separate asset licence declared |
| Overview camera | [MIT YAM optics calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc418ea5e74ac567133d/LICENSE) with original pose |
| Left wrist camera | [MIT YAM left wrist calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc418ea5e74ac567133d/LICENSE) |
| Right wrist camera | [MIT YAM right wrist calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc418ea5e74ac567133d/LICENSE) |
