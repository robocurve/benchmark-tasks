# Real-world build: cage, supplies, latch and connector

This folder has everything you need to build the task's objects: OpenSCAD sources for every printed part, a bill of
materials made of generic parts, print settings, assembly steps, and a calibration check for each mechanism. The
printed parts have the dimensions in `task.py`. Every dimension lives in `params.scad`, and each simulation value
there names the `task.py` constant or box it comes from. `check_params.py` confirms that each of those values equals
`task.py` and that the part files hold no dimensions of their own:

```bash
python fabrication/check_params.py              # or: python -m pytest fabrication/test_fabrication.py
```

Nothing in the build is powered or connected to mains.

## Files

| File | `PART=` | Prints | Qty |
|---|---|---|---|
| `params.scad` | | Shared dimensions: simulation values tagged `task:`, fabrication choices tagged `fab:` | |
| `cage.scad` | `left`, `right` (or `whole`) | Two-bay cage: notched left wall and divider, right wall, roof, back stop, 30-degree lead-ins, rear rail that holds the blades | 1 set |
| `connector.scad` | `blade` | Connector blade: core, two ridges (50-degree front face, 40-degree back face), flange | 2 |
| `connector.scad` | `contact` | Sprung contact carrier (print 2 per supply and turn one over for the right side) | 6 |
| `psu_shell.scad` | `tray` | Supply shell, front section: face, latch guide and spring block, hook window, inlet, ballast bay | 3 |
| `psu_shell.scad` | `housing` | Supply shell, back section: connector slot, carrier pockets, spring bores | 3 |
| `psu_shell.scad` | `lid` | Supply shell top | 3 |
| `handle.scad` | `handle` | Fixed grip handle | 3 |
| `latch.scad` | `tab` | Release tab and link | 3 |
| `latch.scad` | `carriage` | Latch carriage (the arm that slides across the supply) | 3 |
| `latch.scad` | `hook` | Hook: post, square catch, ramp | 3 |
| `plug_inlet.scad` | `plug` | Dummy C13 power-cord plug (live supply only). The inlet it plugs into is part of the tray | 1 |
| `assembly.scad` | | Everything in place, for checking fits in the preview. Set `SECTION = true` to cut through the hook and connector | |

Each file with a `PART` exports an STL already placed in its print orientation, for example:

```bash
openscad -o cage_left.stl  -D 'PART="left"'  cage.scad
openscad -o cage_right.stl -D 'PART="right"' cage.scad
openscad -o contact.stl    -D 'PART="contact"' connector.scad
```

Frames match `task.py`, in mm. In the supply frame, x runs from the face (−x) to the connector (+x), +y is the supply's
left (the hook side), and z = 0 is the table. The cage frame is the supply frame of a supply latched in the left bay.

**Print-fit clearances.** Every sliding fit has 0.3 mm of clearance per side (`FIT_SLIDE`): the tab link in the
face slot, the latch carriage in its channel, the hook in the side window, the contact carriers in their pockets,
the blade through the back stop and the plug tongue in its socket. Located fits have 0.15 mm (`FIT_PRESS`): the
handle spine in its face recess and the link in the carriage socket. Outer faces are printed at the simulation's size
with no clearance added, so the bays keep their 2 mm per side and 1 mm under the roof. If your printer prints large
or has elephant's foot, the supplies will bind in the bays. Turn on elephant's-foot compensation (about 0.2 mm) and
check the fit (step C1) before printing all three supplies.

## Bill of materials (one cage, three supplies)

Use generic parts. The spring specs are targets. Buy an assortment and check each spring's rate with the kitchen
scale (calibration step K0).

| Item | Spec | Qty |
|---|---|---|
| Filament | PLA for the cage, shells, lids, handles and plug; PETG for the sliding and sprung parts (tab, carriage, hook, contact carriers, blades). Colours: grey cage, black supplies, orange handles, green tabs | about 1–1.5 kg |
| Tab springs | Compression spring, music wire or stainless, about 0.5 mm wire, OD 4.5–5.0 mm, free length about 15 mm, rate about 1 N/mm (0.8–1.2), solid length under 6 mm | 3 (+3 spare) |
| Contact springs | Compression spring, about 0.8 mm wire, OD 5.5–6.0 mm with ID at least 3.5 mm (it rides on an M3 screw), free length 13–14 mm, rate about 5 N/mm (4–6), solid length under 8 mm | 6 (+6 spare) |
| M3 heat-set inserts | Brass, M3, up to 5.7 mm long. The holes are 4.2 mm (`INSERT_D`); change that value to your insert's recommended hole size | 36 (+spares) |
| M3 × 8 countersunk (flat head) | Lids (6 per supply) and cage-to-board screws (for a 3 mm board; use board thickness + 5 mm) | 18 + 8 |
| M3 × 8 socket or pan head | Blade flanges (4), tab link (1 per supply), handle (2 per supply, from inside the supply) | 13 |
| M3 × 10 socket head | Tray-to-housing joint (2 per supply), cage divider join (2), plug (1) | 9 |
| M3 × 20 socket head | Contact guide screws (2 per supply) and hook screws (2 per supply) | 12 |
| M3 nuts | Divider join (2, plain); handle (6, nylon-insert lock nuts) | 8 |
| Steel ballast | Mild steel flat bar, 50 × 10 mm, cut to length per supply (3.9 g per mm of length; see B8). Steel washers or steel shot in a small bag for trimming | about 1.8 kg |
| Base board | Smooth, flat hardboard or MDF, 3 mm (or thicker; see "Mounting"), at least 600 × 500 mm | 1 |
| Clamps | C-clamps or F-clamps to hold the board to the table | 2 |
| Small parts | Closed-cell foam strip (about 15 mm thick); amber and green round stickers about 10 mm (status lights); dry PTFE lubricant spray or PTFE tape; optional short length of cut power cord with no conductors connected, for the plug's cord stub | |
| Tools | Soldering iron with an insert tip; calipers; kitchen scale to 5 kg (1 g resolution) that can show negative values after tare; luggage scale or 50 N push-pull force gauge; a 6 mm drill bit to use as a gauge | |

To estimate a spring's rate from its dimensions: k ≈ G·d⁴ / (8·D³·n), where d is the wire diameter, D the mean coil
diameter (OD − d), n the number of active coils, and G ≈ 79,000 N/mm² for music wire (≈ 69,000 for stainless). For
example, 0.5 mm wire at OD 5 mm with 7 active coils gives about 1.0 N/mm. 0.8 mm wire at OD 6 mm with 6 active coils
gives about 4.8 N/mm.

## Printing

Use 0.4 mm nozzle and 0.2 mm layers for every part. The STLs are already placed in these orientations.

| Part | Material | Orientation | Walls / top-bottom / infill | Supports |
|---|---|---|---|---|
| Cage halves (or whole) | PLA | Upside down, roof on the bed | 4 / 5 / 30% gyroid | None (the notches, blade passages and bolt holes bridge) |
| Blade | PETG | Flat on its bottom face, ridges vertical | 100% solid | None |
| Tray | PLA | Floor down | 3 / 4 / 20% | None |
| Housing | PLA or PETG | Standing on its back face | 4 / 4 / 40% | None |
| Lid | PLA | Flat, countersinks up | 3 / 4 / 20% | None |
| Handle | PLA | On its side (8 mm face down) | 4 / 4 / 50% | None |
| Tab | PETG | On its side (8 mm face down) | 100% solid | None |
| Carriage | PETG | Floor down | 4 / 4 / 50% | None |
| Hook | PETG | Upside down (post top on the bed) | 100% solid | None |
| Contact carrier | PETG | Standing on its bottom face | 100% solid | None |
| Plug | PLA | Standing on its front face | 3 / 4 / 20% | None |

The cage needs a 214 × 189 mm bed in one piece. As two halves, each needs 214 × 95 mm. Every other part fits a
220 × 220 mm bed. The shell is printed in three sections (tray, contact housing, lid) to fit the connector pockets
and the latch inside it.

## Assembly

### A. Cage

1. Clean the notches. Keep the front edge of each notch square and sharp, because that edge is what the catch holds
   against.
2. Install heat-set inserts: 4 in the underside of the rear rail, 4 in the underside of the divider (2 per half) and
   4 in the rear rail's back face (2 per bay, for the blade flanges).
3. If you printed halves, join them with two M3 × 10 screws through the divider. Each head goes into its counterbore
   on the left bay's side and each nut into its trap on the right bay's side. Heads and nuts must sit below the
   divider faces.
4. Push each blade in from behind the cage, ridges first, through the passage in the back stop until its flange meets
   the rail. Fix it with 2 M3 × 8. Check: the blade tip stands 15.5 mm in front of the back stop and the ridge crests
   7.9 mm in front of it, 10.0 mm apart across the crests. Lightly coat the ridges with dry PTFE lubricant. The
   simulation uses friction 0.2 at the connector; PETG on PETG is higher.

### B. Each supply (three)

1. Install heat-set inserts: 4 in the tray's lid bosses, 2 in the top of the housing and 2 in the housing's front
   face.
2. Join the tray and housing with 2 M3 × 10 screws from inside the tray, through the rear wall's counterbores. Check
   that the floor and sides are flush. Sand any step on the bottom, because the supply slides on it.
3. **Contacts.** Put a contact spring in each spring bore. Push a carrier into each pocket, tip toward the slot. The
   carriers are prisms, so turn one over for the right side. Pass an M3 × 20 socket screw in from the supply's side,
   through the counterbore, stop wall and spring, and turn it into the carrier's pilot hole. Tighten both screws until
   the gap between the two tips is 6.0 mm. A 6 mm drill shank should slide between them with light drag. The springs
   push the carriers in, so the screw heads are the rest stops. Each carrier must spread at least 3 mm and spring back
   freely.
4. **Handle.** Seat the handle's spine in the face recess. Fit 2 M3 × 8 screws from inside the tray, with heads
   inside and nylon-insert nuts on the spine between the arms.
5. **Tab spring.** Drop the tab spring into the pocket in the spring block, which is the block beside the handle
   inside the face.
6. **Tab.** From the front, push the tab's link through the slot in the face until the tab stands 6 mm to the left
   of the handle bar.
7. **Carriage.** Hold the tab spring compressed and lower the carriage between the front wall and the guide rib, so
   its socket drops over the end of the link. Fix it with an M3 × 8 down into the link. Released, the spring pushes
   the carriage against the left side wall, which is the rest stop.
8. **Hook.** Set the hook on top of the carriage, about 8 mm in from the side wall. Slide it outward until the catch
   and ramp pass out through the window and the post meets the wall. Fix it with 2 M3 × 20 down through the post into
   the carriage. Check: the catch stands 5.0 mm out of the side, and pushing the tab 5 mm toward the handle draws it
   flush, with the ramp's crest 1 mm proud.
9. **Ballast.** Weigh the assembled supply with its lid and screws (W, in g). Cut 50 × 10 mm steel flat bar to a
   total length of (900 − W) / 3.9 mm, for example two equal pieces stacked. Lay the bar between the ballast ribs,
   centred along the supply, and trim the mass with washers to 900 ± 10 g. Fill the space above the steel with foam
   so nothing rattles.
10. **Lid.** Screw the lid on with 6 M3 × 8 countersunk screws, heads flush. Nothing may stand above the top:
    there is only 1 mm under the cage roof.
11. **Finish.** Put an amber sticker on the failed supply's status-light pad and a green one on the live supply's.
    Leave the spare's pad bare. On the live supply only, push the plug's tongue into the inlet socket and fix it with
    an M3 × 10 down through the boss inside (fit it before step 10). The plug is then rigid with its supply, as in
    `task.py`. Glue a short stub of cut cord into the hole in the plug's front if you want. Its conductors are not
    connected, and it is never plugged into anything.

### Mounting and layout

Screw the cage to the base board from below with 8 M3 countersunk screws into the cage's underside inserts. Clamp
the board to the table at two points outside the arms' reach. The board's top surface is the table surface of the
task, and all three supplies stand and slide on it, so it must cover the whole work area. With a thick board, every
height rises by the board's thickness relative to the arms. A 3 mm board keeps that small. If that offset matters
for your robot, clamp the cage directly to the table by its rear rail and leave the board out.

Positions are from the midpoint between the two arm bases, with x forward and y to the left (`task.py`, as in
`task.json` setup):

- **Cage:** bays face the arms, with both mouths on the line x = 369.5 mm. The left bay is centred on y = 0 and the
  right bay on y = −92.5 mm.
- **Failed supply:** latched in the left bay, face 2 mm proud of the mouth.
- **Live supply:** latched in the right bay, face 2 mm proud of the mouth, with its plug in.
- **Spare:** centred at x = 235 mm, y = 220 mm, face toward the arms. Draw its 185 × 73.5 mm outline on the board for
  resets.

## Calibration and acceptance (each mechanism)

1 N reads as 102 g on a kitchen scale. Do each check on all three supplies. Use **Fix** if the result is out of
range.

**K0. Springs (before assembly).** Stand a spring on the kitchen scale, tare, and press it down a measured distance
with a flat object, checking the distance with calipers. A tab spring compressed 5 mm should read 410–610 g (about
1 N/mm). A contact spring compressed 2 mm should read 800–1220 g (about 5 N/mm). Sort out springs outside these
ranges.

**K1. Fit in the bay.** Pushed slowly by hand with the tab held in, a supply slides in and out without binding. With
the tab released, the hook rides in on its ramp and clicks into the notch at home. The face is then 2.0 ± 0.5 mm
proud of the mouth. **Fix:** binding at the sides or roof means an oversize print. Sand the shell, or reprint with
elephant's-foot compensation or about 0.5% smaller outer size. Do not open up the cage.

**K2. Release tab: about 5 N over 5 mm.** Lay the supply on its right side on the kitchen scale (the side away from
the hook), so pushing the tab toward the handle pushes it down. Tare. Press the tab down slowly to its stop. The
reading just before the stop should be 410–660 g (4–6.5 N). At the stop the tab is 1 mm from the handle bar (check
with calipers) and the catch is flush with the side. Released, the tab returns fully, with the catch 5.0 mm out.
**Fix:** too high or too low means changing the spring. A weak return usually means friction: check the carriage
for rubbing and add a little PTFE lubricant.

**K3. Latch holds.** With the supply latched in the bay, hook the luggage scale to the handle bar with a string loop
and pull straight out, without touching the tab, up to 30 N (about 3 kg). The supply must not come out. **Fix:** if
the catch rides out, check that the catch's holding face and the notch's front edge are square, and that the hook
screws are tight.

**K4. Connector mating: about 35 N peak.** Before the lid goes on (or with the lid off), stand a blade upright on
the kitchen scale, flange down, held in a vice or block that rests on the scale. Tare. Hold the supply and lower its
back slot slowly over the blade until the contacts drop past the ridge. Don't let the supply's weight rest on the
blade. Read the peak: it should be 3.1–4.1 kg (30–40 N; the card-edge power connector data cited in the README specify at most 40 N).
**Fix:** too high means lubricating the ridges and carrier tips, or fitting softer contact springs. Too low means
stiffer springs. Keep the tip gap at 6.0 mm; don't change it to tune the force.

**K5. Connector unmating: about 11 N.** On the board, measure the pull needed to slide a supply along the open
board with the luggage scale (friction, F). It should be about 2–3 N: the supplies' friction in `task.py` is 0.25,
which gives 2.2 N for 0.9 kg. Then latch the supply in its bay, hold the tab in, and pull it out by the handle. The
peak minus F should be 8–14 N. In the K4 setup, pulling the supply up off the blade should read −0.8 to −1.4 kg.
**Fix:** as for K4, since the same springs and ridge set both forces. If the board's friction is far from 2–3 N, put
PTFE tape on the supplies' bottoms or change the board's surface.

**K6. Mass and balance.** Each supply weighs 900 ± 10 g (`task.py`: 0.9 kg). Balanced on a pencil laid across its
bottom, it balances within ±10 mm of the middle of its length. **Fix:** move the ballast along the bay.

**K7. Live plug.** Push the plug sideways with about 5 N. The whole live supply moves, and the plug doesn't come
loose (in `task.py` the plug is part of the supply). **Fix:** tighten the plug screw.

**K8. Layout.** Bay width is 77.5 ± 0.5 mm. The notches run 9.0 to 26.5 mm inside the mouth and 13 to 27 mm above
the board. The spare's outline is at the position above, within ±2 mm.

## Where the build differs from the simulation

The outer faces that the robot or another part can touch have the simulation's dimensions. The parts not modelled
in `task.py`, or not printable as modelled, are changed as follows:

- **Shell interior.** The shell is printed as a tray, a housing and a lid, with ribs, bosses, inserts, a ballast bay,
  and the latch's guide channel and spring block inside.
- **Latch.** The link runs on from the tab through a slot in the face to a carriage inside, and the hook is a
  separate part screwed to the carriage. In `task.py` these are one rigid slider body made of separate boxes. The
  printed latch weighs about 15–20 g against 60 g in the simulation. The 5 N spring dominates the forces, so this
  matters little. The tab spring has about 0.5 mm of preload so the hook sits firmly on its stop, which gives about
  5.5 N at the stop against 5.0 N.
- **Contacts.** Each contact tip is task.py's 3 × 4 × 20 mm block on a carrier body, outboard of the ridge, that
  slides on a guide screw. The contact springs have about 0.3 mm of preload, and the printed carriers are lighter
  than 30 g. These are minor compared with the 5 N/mm springs.
- **Handle.** It has a spine that sits in a recess in the face plate, so its upper arm, which ends in the air in
  `task.py`, is held.
- **Plug.** It has a tongue and a screw into a socket in the inlet, so it is rigid with its supply as in `task.py`.
- **Cage.** The lead-ins are printed full height, to the roof top, instead of 40 mm, so they print on the bed, and
  each lead-in is joined to its wall by a small web. The server body behind the back stop is a 10 mm rail. Each blade
  continues through the back stop to a flange behind the rail.
- **Friction.** Printed parts on each other have more friction than the simulation's 0.15–0.25. Calibration steps K4
  and K5 bring the connector forces into range.
- **Status lights** are stickers on raised pads.

## Alternative: real CRPS supplies

Instead of printing the supplies, you can use three real CRPS-format supplies, unpowered and never connected to
mains. They are 185 mm long, about 73.5 mm wide and about 40 mm tall, and come with their own handle, latch tab and
gold-finger edge. Print only the cage, and adapt it:

- **Notch.** Real CRPS latches differ in position between vendors. Measure where the latch on your supply stands out
  of its side and set `NOTCH_X` and `NOTCH_Z` to it. `check_params.py` will then report those values as differing
  from `task.py`, which is expected for this route.
- **Connector.** The printed blade does not apply: on a real supply the gold-finger blade is on the supply. Leave
  the blades out, and mount an unwired server-side CRPS card-edge power connector, specified for CRPS by its maker,
  behind each bay at the height and position of the supply's gold fingers.

This route matches a real server more closely, but it departs from the simulation in three ways:

- The real handle and tab are smaller than the simulation's enlarged ones, so grasps differ.
- The latch spring and the mating force are whatever the supply and connector provide. The connector's data specify a
  mating force of at most 40 N, against about 35 N simulated.
- The mass varies by model.

Weigh the supplies and check K2 to K5 against the simulated values before using them.
