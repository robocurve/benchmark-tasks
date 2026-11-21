# Building the screen and its two squeeze latches

This folder holds the drawings, printable files, parts list and steps for making the real fixture for this task: a
bought screen frame kit, two latches made from three printed parts, a length of steel wire, a stock spring and six
screws each, and the holes in the window's jambs. All dimensions are millimetres unless marked.

**Status.** The package is a design: the drawings, the printable files and the model in `task.py` use the same
numbers (`near_misses.py` checks `dimensions.json` against the model, and `make_package.py` checks that each STL is
a closed solid of the expected volume). No physical copy has been built or tried: the fits (the slide in its slot,
the bolt in the stile) may need a light pass with a file, and the tab's return under its 1 N spring against the
friction of printed parts is untested.

No commercial screen latch was found that is worked by pinching a tab toward a fixed lug. Commercial screen plunger
bolts (for example Prime-Line PL 14660, a 7/32 in spring-loaded diecast bolt with a plastic cap, for 3/8 to 7/16 in
frames, https://www.primelineparts.com/window-screen-plunger-latches-aluminum-25-per-tub/) are drawn in by the cap
on the bolt; this latch is drawn in by squeezing a tab toward a lug, the squeezing
that the review of TK-M74T74 pointed to. It keeps the plunger bolt's diameter and its place in the frame.

## Files

| File | What |
|---|---|
| `sheet-1-latch-assembly.svg` / `.png` | The assembled latch: front view, section through the bolt, parts list, how the latch is read in a trial |
| `sheet-2-printed-parts.svg` / `.png` | Each printed part, the bolt and the gauge card, dimensioned |
| `sheet-3-screen-and-jamb.svg` / `.png` | The screen with both latches, the holes in the stiles and in the jambs |
| `stl/housing-left.stl`, `stl/housing-right.stl` | Part 1, housing plate with spring post |
| `stl/slide-left.stl`, `stl/slide-right.stl` | Part 2, slide (tab, neck, carriage) |
| `stl/lug.stl` | Part 3, lug (two needed) |
| `stl/gauge-card.stl` | Gauge card for reading a trial (one needed) |
| `stl/mesh-check.json` | Triangle count and volume of each solid; every edge is shared by exactly two facets |
| `dimensions.json` | The dimensions the drawings and the model share |
| `make_package.py` | Writes every file above from one table of dimensions (needs shapely 2.1 for the STL files and Pillow for the PNG files) |

"Left" is the latch at the viewer's left when looking at the screen from the room (the robot's left). The STL files
are in millimetres, each part resting in the first octant in the drawing's axes.

![Sheet 1](sheet-1-latch-assembly.png)

## Parts list

| No. | Qty | Part | Source |
|---|---|---|---|
| S | 1 | Window screen frame kit, 7/16 x 3/4 in roll-formed aluminium, mill finish, with four square-cut plastic corners, vinyl spline and charcoal fibreglass mesh; builds up to 36 x 36 in and is cut down to 16 x 14 in | Prime-Line MP8239, https://www.primelineparts.com/screen-frame-kit-7-16-in-x-3-4-in-x-36-in-x-36-in-aluminum-mill-1-kit/ |
| 1 | 1 left, 1 right | Housing, printed | `stl/housing-left.stl`, `stl/housing-right.stl` |
| 2 | 1 left, 1 right | Slide, printed | `stl/slide-left.stl`, `stl/slide-right.stl` |
| 3 | 2 | Lug, printed | `stl/lug.stl` |
| 4 | 2 | Bolt: 7/32 in (5.56 mm) music wire, 40 mm long (sheet 2) | K&S Precision Metals stock no. 510 (7/32 in x 36 in, four lengths), https://ksmetals.com/products/mw36-7-32 |
| 5 | 2 | Compression spring: outside diameter 6.1 mm, wire 0.41 mm, free length 23.83 mm, rate 0.263 N/mm, solid height 3.28 mm, music wire | Lee Spring LC 016C 12 M, https://www.leespring.com/product/compression-spring-lc016c12m-music-wire |
| 6 | 10 | No. 4 x 3/8 in flat-head sheet-metal screw (type A or AB) | hardware store |
| 7 | 2 | No. 4 x 1/2 in pan-head sheet-metal screw (type A or AB) | hardware store |
| G | 1 | Gauge card, printed | `stl/gauge-card.stl` |

Printed parts: black PETG or nylon, 0.2 mm layers, 4 walls, 40 % infill or more. About 21 cm3 of plastic for each
latch. Housing: front face on the bed, post up, with support under the spring peg only (a 3 mm overhang on the
post); afterwards clean its five screw holes with a 3.2 mm (1/8 in) drill. Slide: its bottom face (the face with
the square pilot hole) on the bed, no support. Lug: on its base, no
support. Sizes are nominal; the sliding fits (neck in the slot, tab over the plate, carriage beside the stile) have
0.1 to 0.3 mm clearance, so ease them with a file if the printer runs tight, and run a 7/32 in drill through the
slide's bolt bore if the wire does not push in by hand.

Spring check: in the latch the spring is 20.0 mm long with the tab at rest and 8.5 mm with the tab squeezed onto the
lug, which gives 0.263 x (23.83 - 20.0) = 1.0 N at rest and 0.263 x (23.83 - 8.5) = 4.0 N squeezed. The model uses
1.0 N and 3.9 N.

The flat-head screws (part 6) go into flat-bottomed counterbores 6.2 mm across and 1.8 mm deep: the head's cone
seats on the rim of the 3.2 mm hole and its top ends about 0.4 mm below the surface. The two counterbores for the
lug's screws, in the back of the plate, break 0.2 mm into the inner end of the slot; that is intended.

Tools: 3D printer; rotary tool or bench grinder with an abrasive cut-off wheel; drill with 6.0 mm, 7/32 in, 3.2 mm
(1/8 in), 2.4 mm (3/32 in) and 8 mm bits; small round file; screwdriver; tweezers; spline roller; hacksaw or mitre
saw for the frame; chisel; ruler or calipers; spring balance (0 to 5 N or 500 g); kitchen or letter scale.

## Assembly

Screen frame

1. Cut the kit's frame stock square so that, with the four corner inserts pressed in, the frame measures 406 mm wide
   and 356 mm tall outside (16 x 14 in). Measure the corner inserts first: if each fills a 19.05 mm (3/4 in) square
   at its corner, the pieces are 368 mm and 318 mm long; otherwise follow the kit's leaflet. Assemble the frame with
   the spline groove on the back (window side) face and check the outside size. Leave the mesh off for now.
2. On each stile (side piece), mark a point 45 mm above the frame's bottom edge on the middle of the stile's 11.1 mm
   thickness. Drill 6.0 mm straight through both walls of the stile, from the outside edge. If the drill passes
   through the leg of a corner insert or breaks into the spline groove, carry on; deburr the holes.

Bolts (two)

3. Cut two 40 mm lengths of the 7/32 in wire with the cut-off wheel (the wire is hardened; a saw will not cut it).
4. Grind a bullet nose 2 mm long on one end of each, down to about 3 mm diameter.
5. Grind a flat 0.8 mm deep and 6 mm long on each, starting 2.5 mm from the other end (sheet 2). Remove burrs.

Each latch (left, then right with the mirrored parts)

6. Screw the lug to the front of the housing plate with two screws (part 6) from the back of the plate. The lug's
   pilot holes are on its centre line, so it fits either way round; its base covers u 33.5 to 39.5, v 31 to 59
   (sheet 1), with one long face along the slot's inner end. The screw heads must sit in their counterbores, below
   the back of the plate.
7. Pass the slide's tab through the plate's slot from the back, with the tab's overhang toward the plate's outer
   edge (the edge that will lie on the screen's side). The carriage stays behind the plate.
8. Push a bolt, flat end first and flat toward the slide's bottom face, into the carriage's bolt bore from the outer
   side until its end is about level with the carriage's inner face (0.1 mm short of it). Drive one screw (part 7)
   into the square pilot hole in the slide's bottom face until it bears on the flat; it works as a set screw and its
   head stops about 0.7 mm short of the slide's face. Set the bolt by measurement: its tip 25.5 mm (+/- 0.3) from
   the tab's outer face; if not, back the screw off, move the bolt and tighten again.
9. Put one end of the spring into the carriage's spring bore, compress it with the tweezers and let its other end
   out onto the square peg on the post. The slide now rests against the outer end of the slot.
10. Hold the housing against the frame's room-side face at the bottom corner, about 20 mm in from the side edge,
    with the tab squeezed to the lug. Enter the bolt's nose into the 6 mm hole in the stile's inner wall, then
    slide the housing outward, releasing the tab, until the plate is flush with the screen's side edge and bottom
    edge. The bolt now passes through both holes in the stile.
11. Through the plate's three counterbored holes, drill 2.4 mm (3/32 in) pilot holes into the frame's face (two into
    the stile, one into the bottom rail) and fix the plate with three screws (part 6), heads flush.

Checks, each latch

12. With the tab at rest the bolt's tip stands 9.5 mm (+/- 0.5) proud of the screen's side edge. The tab moves
    11.5 mm to close on the lug, and the tip is then about 2 mm inside the screen's side edge: the bolt has left the
    hole in the stile's outer wall and is carried by the hole in the inner wall, the slide's neck in the slot and
    the carriage's lip behind the plate. Released, the tab returns by itself and the nose leads the bolt back out
    through the outer hole without catching; if it catches, ease that hole's inner edge with the round file. The
    slide has no other retainer: off the frame, nothing but the lip holds it in the plate. Over both bolt tips the
    screen measures 425 mm at rest and 402 mm with both tabs squeezed.
13. A spring balance hooked on the tab reads about 1 N as the tab starts to move and about 4 N as it reaches the
    lug. Weigh the finished screen: the model uses 0.22 kg, an estimate that has not been checked against a
    built screen; each latch adds roughly 30 to 37 g.

Mesh

14. Lay the mesh over the back face and roll it and the spline into the groove, as the kit's leaflet shows. Where
    the bolt crosses the groove, cut the spline either side of the bolt. Trim the mesh.

Window section

15. Build the window section to the setup in `task.json`: opening 412 mm between the jambs' inner faces, head
    366 mm above the sill, a stop behind the opening lapping the screen by 10 mm on all four sides, and the jambs'
    room-side faces 45 mm in front of the seated screen's room-side face.
16. In each jamb's inner face cut a hole 10 mm front to back, 8 mm tall and 12 mm deep, centred 45 mm above the
    sill and 5.55 mm in front of the stop's face (sheet 3): drill 8 mm twice, 2 mm apart, 12 mm deep, and pare the
    ends square with the chisel.
17. Measure the finished opening and the finished screen: the total side play (opening minus the screen's width)
    should be 6.0 mm and must lie between 5.5 and 6.3 mm, so that a tab that has sprung fully back reads 22.3 mm or less
    wherever the screen sits; plane or shim a jamb if it does not. Then seat the screen with both tabs squeezed and
    let go: both tabs return to rest, and the gauge card's 22.5 mm width does not drop in between either tab and
    its jamb, with the screen pushed against either jamb.

## Reading a trial on the real fixture

Each tab and its bolt are one rigid piece, and the bolt's tip is 25.5 mm beyond the tab's outer face (step 8). So
how far the bolt reaches into the jamb is read from the room side, without seeing into the hole.

| What the scoring asks | Reading | Tool |
|---|---|---|
| Bolt latched: its end inside the jamb hole, 3 mm or more past the jamb's face | The distance from the tab's outer face to the jamb's inner face, taken at the tab's root on the latch plate, is 22.5 mm or less. The readings fall in separate bands: a tab that has sprung fully back reads 16.0 to 22.0 mm, wherever the screen sits in its 6 mm of side play (19.0 mm centred); a bolt that missed its hole stops against the jamb's face and reads 25.5 mm or more; a tab squeezed onto the lug reads 27.5 mm or more. A reading between 22.0 and 25.5 mm means a tab stuck part-way, and the 22.5 mm limit decides it | Ruler or calipers; a square-on photo with the ruler or the card in view; or, as the go/no-go, the gauge card laid flat on the latch plate with the card's 22.5 mm width between tab and jamb: the card drops in only when the bolt is not latched |
| Seated: every corner of the back face within 2 mm of the stop | The screen's room-side face is 43 mm or more behind the jamb's room-side face at each corner (45 mm when hard against the stop). On the latch plates, which are 3 mm thick, 40 mm or more | Depth rod or ruler; or the gauge card stood on the frame (43 mm edge) or on the latch plate (40 mm edge): its end does not stand proud of the jamb's face |
| Seated: bottom corners no more than 1.5 mm above the sill | A 1.5 mm feeler does not enter under either bottom corner | Feeler gauge, or the gauge card's 1.5 mm thickness |
| Seated: within 2 degrees of upright | Follows from the four corner depths and the sill reading above | none |
| Hands let go, nothing moving for 1 s | Every part of both grippers more than 5 mm from the screen and its latches, and the screen and tabs still, for 1 s | Video |
| Dropped at any time (0 for the trial) | The screen falls with both grippers 20 mm or more from it; the 0.5 m/s in the rule is reached after about 13 mm of free fall | Video |

The simulator's rule that no side edge is more than 0.5 mm past a jamb's face guards against one body passing
through another in simulation; it cannot happen with real parts.

## Differences from the simulated model

- The model's bolt is a 5.6 mm square bar with a square end; the real one is 5.56 mm round wire with a 2 mm nose.
- The model has the tab, the lug and the 3 mm plate only. The neck, carriage, spring, post, slot and screw heads are
  behind or within the plate and are not modelled; nothing in front of the plate differs except the slot, which
  shows between the tab and the lug, and a 0.2 mm gap under the tab.
- The model's latch spring is 250 N/m with 4 mm preload (1.0 N at rest, 3.9 N squeezed); the stock spring gives
  1.0 N and 4.0 N.
- The model's frame face is 19.1 mm; the kit's is 3/4 in (19.05 mm).
- The model's bolt is 29.5 mm long and ends 20 mm inside the screen's side at rest; the real one is 40 mm long and runs on
  into the carriage. The extra length is behind the latch plate.
