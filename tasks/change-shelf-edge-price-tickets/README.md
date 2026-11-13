# Change two shelf-edge price tickets in a clear ticket strip

Swap the middle and right cans' price tickets for the new ones from the stand; old tickets go in the waste tray.

Price changes are routine shelf work. O*NET 53-7065.00 (Stockers and Order Fillers, about 2.8 million US workers)
lists the task "Stamp, attach, or change price tags on merchandise, referring to price list"; retail salespersons
and cashiers do the same in small stores. Most supermarket shelves carry a clear PVC data strip along the shelf
edge: the 39 mm ticket stands in a channel behind a springy clear front that is joined to the strip's back along
the bottom and grips the ticket with its rolled top edge.

The skill: with one hand ease or hold the springy clear front open by pressing its grip edge; with the other press
the 9.5 mm of old 0.3 mm card that shows above the front against the strip's back and slide it up by fingertip
friction until it stands proud enough to pinch, lift it out, then drop a new card into the open channel under the
right product and let the front close on it. Nearest board tasks hold a sprung cover or slider while the other hand swaps a part; none handles a
thin card in a channel narrower than a fingertip.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=change_shelf_edge_price_tickets.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Swap the middle and right cans' price tickets for the new ones from the stand; old tickets go in the waste tray." \
  --scorer task_score --max-steps 4800 --store-frames --log-dir ./demo-run
```

For a model trial replace `--policy yam_scripted` with `--policy dream`. `--scorer task_score` gives the score.
One trial allows 4,800 control steps (240 s at 20 Hz). The scripted demo plan lasts about 121 s (2,415 steps)
and reaches success, ending the trial, at step 2,132 (107 s); it uses only the authored layout and elapsed time
and never reads object state. Wall-clock time per trial on one GPU: about 7 to 16 minutes. Seeds: the start layout is fixed (`object_jitter=0`; the tickets must not be shifted
sideways in their 1 mm channel and slots).

## Scene (metres; world x forward, y left, z up; table top z = 0.7475; arm bases at (0, +-0.305))

| Object | Real item | Model | Size / pose |
|---|---|---|---|
| Shelf | steel gondola base shelf, 400 mm section | fixed: 1.5 mm deck 370 deep, 42 mm front flange, plinth set back 15 mm | deck top z = 0.8675 (120 mm above the table), flange front face x = 0.150, y -0.20..+0.20 |
| Ticket strip | HL Display DBR datastrip, clear PVC, 39 mm label height, item 304622 (1000 mm, tape mounted), cut to 400 mm | fixed back and bottom fold; the clear front is a hinged link | 42 mm high, top flush with the deck |
| Back wall | 1.0 mm PVC | box | front face x = 0.149, z 0.8255..0.8675 |
| Channel | 1.0 mm from the back wall to the inside of the closed front | | floor (bottom fold) z = 0.8285, 39 mm below the back wall's top |
| Clear front | 0.8 mm clear PVC, 29.5 mm tall, 13 g | one rigid hinged piece: top and bottom edges, end stiles and the clear window (the window collides but is not drawn, so the tickets show) | hinge along the fold at x = 0.1476, z = 0.8285; shut against the tickets; a seated ticket's top 9.5 mm stands above it |
| Rolled top edge | the front's top edge curled in toward the back wall | 0.8 mm bead | 0.2 mm from the back wall, z 0.8525..0.8560 |
| Grip edge | flat tab along the front's top | 6 x 1 mm | z 0.8570..0.8580, x 0.1412..0.1472 |
| Front spring | the front's own springback | torsion spring 0.06 N m/rad, rest 10 deg past shut, damping 0.002 N m s/rad | 0.41 N at the rolled edge on the seated tickets; about 2 N on the grip edge to hold it open 30 deg; stop at 36 deg |
| Cans | No. 300 food cans, 3 x 4-7/16 in (76 x 113 mm), 15 oz net | free cylinders 0.47 kg | x = 0.298 (fronts 110 mm behind the strip), y = +0.06 (yellow), -0.04 (red), -0.14 (green), standing on the deck |
| Tickets | white 300 gsm board shelf-edge tickets, 60 x 39 mm, 0.3 mm | free rigid cards, 0.7 g; printed face toward the robot with a 52 x 16 mm band in the product's colour 6 mm above the bottom | seated in the strip: centres (0.1489, y of the can, 0.8480) |
| Ticket stand | desk ticket stand, grey ABS | fixed, 50 x 72 x 15 mm, 4 mm floor, two 1.0 mm slots 64 mm long along y, 25 mm apart | centre (0.075, -0.175); new green ticket in the front slot (centre x 0.0629), new red one in the rear slot (x 0.0879), y = -0.175, z = 0.7710 |
| Waste tray | desk tray, grey ABS | fixed, 100 x 120 mm outside, 25 mm walls, 3 mm floor | centre (0.050, -0.050) |

Friction (static/dynamic, estimates, not measured): tickets 0.8/0.7, strip 0.2/0.15, cans and shelf 0.4/0.35,
tray 0.4/0.35, stand 0.3/0.25. The simulator steps at 240 Hz.

## Scoring

End state. A ticket is seated under a can when its centre plane lies between the closed front and the back wall
(1 mm allowance), its lowest corner is within 2 mm of the channel floor, it is upright within 5 deg, its printed face
points to the aisle within 30 deg, its centre is within 15 mm (along the shelf) of the can's centre line, and the
clear front is within 2 deg of shut. For each of the middle and right cans: 0.25 when its old ticket is out of the
strip and in the waste tray; a further 0.25 when its own new ticket is seated under it and its old ticket is no
longer in the strip. A new ticket under the other can, or in front of an old one, earns nothing. Any credit needs
the untouched left ticket still seated, both grippers released (centre point between the pads at least 30 mm from
every ticket's bounding box) and every ticket, can and the clear front at rest (under 0.5 mm per control step), all
held for 1 s. A can ever tipped more than 20 deg or dropped 20 mm gives a permanent 0. Success is 1.0 held a further
1 s.

Real trial: a front photo with a ruler along the strip (centring and uprightness; a seated ticket's top is flush
with the strip's back wall, checked with a straightedge), the clear front shut, the waste tray's contents; release,
stillness and the cans from the video.

## Real-world build

- Shelf: any steel gondola shelf with a flat front flange at least 42 mm tall (or a 42 mm strip of wood/steel on a
  bench, 120 mm high); the robot must reach the strip from in front.
- Strip: HL Display DBR datastrip, clear, 39 mm (item 304622 cut to 400 mm, or any 39 mm tape-on datastrip with a
  springy clear front), taped to the flange, top flush with the shelf deck.
- Tickets: 60 x 39 mm cut from 300 gsm white board, printed with product name and price; colour band to match each
  can's label.
- Cans: three 15 oz food cans; ticket stand and waste tray: any desk ticket stand with 1 mm slots and any shallow tray
  of the stated size.
- Safety: nothing hazardous; the cans are the heaviest objects (0.47 kg) and stand 120 mm above the table.

## Files and licences

`change_shelf_edge_price_tickets.py` (task), `yam_parts.py` and `yam_joints.py` (our own unmodified parts and joints
kit), `test_change_shelf_edge_price_tickets.py` (scorer tests), `task.json`. Every object is built from primitive
shapes in code; the task uses no third-party assets (no meshes, scans, textures or fonts).
