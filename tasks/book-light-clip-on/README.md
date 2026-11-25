# Clip a book light onto an open hardback's cover with no pages caught, then bend its neck to light the page

A clip-on book light goes on a book that is propped open for reading, and the light's spring clip goes on the top edge of the cover, never on the pages, so the pages can still turn. A 6 x 9 in trade hardback stands open on a bamboo book stand that leans back 25 degrees. The stand's back is shorter than the book, so the head edge of the right board stands free, and the right half's leaves lie back under their own weight on the free endpaper, which lies flat on the board: there is no gap between board and endpaper. A Gritin 9 LED clip-on book light, switched on, lies loose on its side on the desk in front of the book, to the right, its neck folded; its LEDs are a warm-white cone light that lights whatever the head faces.

The right hand pinches the clip's tails, which opens the jaws, lifts the light off the desk and turns it upright, and brings it over the book from above. Over the red mark it eases its squeeze until the front jaw stands only about 2 degrees open, squares and leans the clip with the board, and slides the jaws down over the board's head edge: the front jaw's thin, tapered tip runs down the board's face, wedges in under the endpaper and lifts the endpaper and the leaves aside. It then lets go, so the spring shuts the jaws on the board alone, and moves clear. The left hand takes the light's head from the left, level, across its width, and bends the gooseneck down with a turn of the wrist until the LEDs face the right page, then lets go and backs off along the head toward the reader. The clip's spring (about 0.31 N m on the board) is about three times the neck joint's breakaway torque (0.10 N m), and the collar holds to 0.40 N m, so the bend does not lever the clip off. Both hands then stay clear, and the light must stay clipped and aimed for 2 s.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,noaim,held,beside,slack,endpaper,overpages,reversed,none
python demokit.py task.py OUT --no-record --mode tip05   # each range mode alone: tip05, tip10, spring_lo, spring_hi, mu_lo, mu_hi, neck_lo, neck_hi, collar_lo, collar_hi
```

The harness's instruction (Dream-1 run): `--instruction "Clip the book light onto the top edge of the open book's right cover at the mark, no pages in the clip, then bend its neck to light the right page."`

`BOOK_TRACE=1` (the default) prints the scorer's terms every 20 steps.

**Result** (scripted, GPU physics, 20 Hz): 1.0: the light clipped 15.9 mm deep on the board at the mark, the jaws shut on it, no endpaper or leaf caught; the LEDs within 10.2 degrees of the line to the page centre, 9.0 cm from it, holding after the hand let go; both hands clear; 950 steps (47.5 s)

**Dream-1 run** (the harness's own policy, 2,400 steps, one trial): 0.0. It reached no tier before the step limit.

## Scoring, and the routes that score less

The score is read from the state at the end of the run. Every tier is read from the state now, so a step undone loses its credit.

- **0.33, clipped on the cover:** the board's edge 10 to 24 mm up into the jaws (the jaws are 22 mm deep), the clip within 30 mm of the mark along the edge and within 15 degrees of the board's plane, its back pad on the board's outer face (within 3 mm), the sprung jaw within 3 degrees of shut on the 2.5 mm board (with nothing between them the spring shuts it to -8 degrees; held open by a hand it reads more), and neither the endpaper nor the leaves between the jaws (each one's face at the clip lies outside the front pad). Real trial: photograph along the board's head edge from the fore-edge side, camera axis in the board's plane, at the clip's height: only the board is between the jaws, no page, and the jaws touch the board with no visible gap; a steel rule held against the clip's side in the same photo reads how far the board's edge is into the jaws (10 to 24 mm). A second photo from in front, with the rule along the head edge, reads the clip's centre within 30 mm of the mark.
- **0.67, aimed at the page:** clipped as above, and the LED face's normal within 15 degrees of the line from the LEDs to the right page's centre, the LEDs at least 6 cm from it. Real trial: in a photo from in front, square to the page, with a printed circle at the right page's centre of radius 0.27 x the LEDs' distance from that centre (2.4 cm at 9.0 cm); the spot's centre inside it.
- **1.0, let go and held:** aimed as above, and every part of both grippers (a point grid over each finger plate, and the palm and housing) at least 5 cm from the light and from the book, for 2 s running (a count that restarts if any of these breaks).

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | as above | 1.0 |
| noaim | clips the light on and lets go, but does not bend the neck | 0.33 |
| held | as full, but the left hand keeps hold of the head | 0.67 (clipped and aimed, but the hand is not clear) |
| beside | as full, but the clip goes on the board's edge 45 mm from the mark | 0.0 (the clip does not seat there and the light falls off the book) |
| slack | as full, but the clip goes on only 6 mm deep | 0.0 (6 mm is too shallow to hold: the light falls off the book) |
| endpaper | as full, but the tip goes between the endpaper and the leaves | 0.0 (the clip does not seat past the endpaper and the light falls) |
| overpages | the jaws, wide open, go down in front of the leaves to take board and leaves together | 0.0 (the light does not seat on board and leaves and falls) |
| reversed | the light turned half a turn, so its back jaw would go between board and leaves | 0.0 (the clip cannot lean with the board this way round and falls) |
| none | nothing | 0.0 |
| tip05, tip10 | as full, the front jaw's tip 0.5 mm, 1.0 mm | 1.0, 1.0 |
| spring_lo, spring_hi | as full, the jaws opening at 10.6 N, 12.9 N | 0.0 (seats 17 degrees tilted, then levers off in the bend; below the accepted window), 1.0 |
| mu_lo, mu_hi | as full, the front jaw on the book at friction 0.4, 0.6 and the back pad at 0.55, 0.75 | 1.0, 1.0 |
| neck_lo, neck_hi | as full, the neck's breakaway 0.07, 0.13 N m | 1.0, 0.0 (the clip tips over on the board's edge on the way down; above the accepted window) |
| collar_lo, collar_hi | as full, the collar's breakaway 0.35, 0.50 N m | 1.0, 1.0 |

## Real items

- Gritin 9 LED rechargeable clip-on book light, model G1330: https://www.amazon.com/dp/B08GG42WXY (63 g, gooseneck 19.5 cm, head 82.7 x 40.3 mm, clip 59.7 mm).
- wishacc bamboo book stand, 11.0 x 8.1 in size: https://www.amazon.com/dp/B07459P7BF (a 279 x 206 mm bamboo plate with a full-width ledge hinged at its foot, about 2 in deep folded, a wire prop and a slotted rack with five angles; $19). Its plate is shorter than a 6 x 9 in book, so the book's head edge stands free above it.
- A 6 x 9 in trade hardback (standard trim size), any publisher.

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Book | trade hardback, 6 x 9 in trim: boards 152 x 229 x 2.5 mm (cloth), each half's page block 11 mm (200 g), the boards' squares 3 mm, opened at its middle; the right half's leaves one rigid body on a hinge at the gutter, lying under their weight on a free endpaper 0.3 mm thick, hinged at the gutter, which lies on the board (0.05 mm of solver clearance between them; real gap none); friction 0.5 static, 0.4 sliding | 6 x 9 in is a standard US trim | estimates for the boards, leaves, endpaper and friction |
| Book stand | wishacc bamboo book stand, 11.0 x 8.1 in: plate 279 x 206 x 8 mm leaning back 25 degrees, full-width ledge 50 x 279 x 12 mm at its foot, a prop and a rack 140 x 152 mm; the book's bottom on the ledge, its head edge about 35 mm above the plate; placed at the book's origin (0.40, 0.10) m | plate 11.0 x 8.1 in (279 x 206 mm), ledge about 2 in deep (the listing's folded depth) | plate from the listing's size; ledge thickness, prop and rack estimated from the product photos |
| Clip | two rigid jaws 28 mm wide, 22 mm deep, on a hinge 24 mm above the tips; the front jaw 4 mm thick, its last 16 mm tapered to a 0.6 mm tip; the back jaw's tip rounded on the inside; the front jaw on a torsion spring (0.5 N m per radian, preloaded 35 degrees past shut: about 0.31 N m on a 2.5 mm board, 0.48 N m with the jaws 20 degrees open); tails to 55 mm; hard plastic, friction 0.5 static, 0.4 sliding, but for the back jaw, which is rubber-padded (0.8, 0.7); the housing the neck leaves from to 80 mm | clip about 60 mm long, rubber-padded jaws and jaws open to 3 cm (maker, via manuals.plus video) | Gritin G1330 listing for length, maker's video for the pad and opening; tip thickness and spring estimates (Real trial measures both) |
| Gooseneck | a collar 75 mm and a neck 120 mm (19.5 cm in all), on two friction joints (breakaway about 0.40 and 0.10 N m, about ten and four times the moments of the weight they carry, so any bent pose holds) | gooseneck 19.5 cm | Gritin G1330 listing, full length; breakaway an estimate |
| Head | 82.7 x 40.3 x 12 mm, LED face 70 x 30 mm | 82.7 x 40.3 mm | Gritin G1330 listing; thickness and LED face estimated |
| Light, whole | 63 g | 63 g | Gritin G1330 listing |

Positions in metres; x forward, y to the robot's left, z up; origin midway between the arm bases (0, +/-0.305); the desk is the bench top (z = 0.7475).

## Colliders

| Body | Colliders | Free or fixed |
|---|---|---|
| Book | both boards as one box; the left page block as a box; the right endpaper and the right leaves, each a hinged convex hull | fixed in the stand (see below); endpaper and leaves hinged |
| Book stand | back, ledge and strut as convex hulls (the base, flat on the table, has none: nothing reaches it) | fixed |
| Light | back jaw, bridge, back tail and housing; front jaw, knee and tail (hinged); collar; neck and head (hinged) | free, 63 g |

## Robot figures the demo keeps within

The YAM's jaws open to 94 mm and grip about 40 N a finger. The demo squeezes the clip's tails with up to 30 N, 26 mm above the hinge, which opens the jaws about 27 degrees, then eases until the front jaw is about 2 degrees open to go down onto the board, and takes the head across its 40 mm width with 30 N.

## Real trial

Stand the book open on the stand at its marks, with a small red sticker at the middle of the right board's head edge. Switch the light on, fold its neck and lay it on its side on the desk at its mark. After the run, photograph along the board's head edge from the fore-edge side (camera axis in the board's plane, at the clip's height) to see between the jaws for caught leaves, and photograph the lit page from in front, square to the page, with a printed circle at the right page's centre of radius 0.27 x the LEDs' distance from it (2.4 cm at 9.0 cm). Film both hands with a phone camera on a tripod about 1.5 m in front of the desk, framing both arms, the light and the book: each at least 5 cm from the light and the book for 2 s.

| Real-trial kit | Use | Price (about) |
|---|---|---|
| Gritin 9 LED rechargeable book light (G1330) | the light | $10 |
| A 6 x 9 in hardback | the book | $15 |
| wishacc bamboo book stand, 11.0 x 8.1 in | the stand | $19 |
| Total | | about $45 |

## Safety for a real trial

The light runs on a small built-in lithium battery at USB voltage; unplug its cable and switch it on before the run.

## What is modelled, and what is not

- The book is fixed in its stand, not free. A real book rests on the ledge and leans on the back plate under its own weight; the clip goes on by pushing down the board's plane, which presses the book onto the ledge, and the bend pulls on the clip at the head edge, about 0.1 N m against a book of a few hundred grams leaning 25 degrees, so the real book does not move either.
- The leaves of the right half are one rigid body on a hinge, under their own weight, not separate pages: they cannot bend, so the jaw's tip lifts the whole block aside, where real pages would bend at the tip. The endpaper is one rigid leaf on its own hinge, lying on the board (0.05 mm of solver clearance, where the real gap is none: kept this small so the two do not start in contact, which the solver would push apart), and the leaves lie on it, so the 0.6 mm tip must wedge in and lift both. In the colliders the endpaper's top edge is rounded to its 0.3 mm thickness and the page block's to 1 mm, as paper edges give under a tip: a square corner stops a thin tip dead in the solver. Real paper bends and gives far more than this rounding, so getting the tip under the endpaper is if anything harder in the model.
- The gooseneck is two friction joints, not a continuous bendable neck: each is a stiff drive capped at its breakaway torque whose set point follows the joint once it has slipped 2 degrees.
- The clip's spring, the two breakaway torques, the friction and the front jaw's tip thickness are estimates, not published. Before a real trial, measure each and accept the light only within the range below; each accepted range's two ends score 1.0 in the model (range modes, above).
  - Spring. Bound: the jaws must hold the 63 g light on the board, a few newtons at most, and open under a light pinch, below 4 lb (17.8 N), the pinch force that the Washington State caution-zone checklist (WAC 296-62-05174) flags as a hazard. Measure with a small spring scale pulled square to the clip's tail 26 mm from the hinge: accept a light whose jaws start to open at 11.7 to 12.9 N (0.30 to 0.34 N m; model 11.7 N, the full run, which is the window's lower edge; spring_hi 12.9 N), well inside that bound. At 10.6 N (spring_lo) the model scores 0, so a softer clip is not used.
  - Gooseneck. Bound: each joint must hold the weight it carries, or a bent pose sags (weight moments about 0.025 N m at the neck, 0.040 N m at the collar). Pulled square to the head 120 mm out, accept a neck that starts to bend at 0.58 to 0.83 N (0.07 to 0.10 N m; model 0.83 N, the full run, which is the window's top edge; neck_lo 0.07 N m; at 0.13 N m, neck_hi, the model scores 0, so a stiffer neck is not used); pulled square to the neck's foot 75 mm above the clip, the collar starts to bend at 4.7 to 6.7 N (0.35 to 0.50 N m; model 5.3 N; collar_lo, collar_hi). Upper bound: the neck must give before the clip does, so its 0.10 N m at most stays under a third of the clip's 0.31 N m hold on the board, and bending it cannot lever the clip off; the collar's 6.7 N at most is under the 17.8 N pinch bound above, so a hand can still set it.
  - Friction of the clip on the book. Not published for this pair. The front jaw is smooth moulded plastic, a few tenths on paper; the back jaw carries a rubber pad on the cover, and rubber on cardboard is 0.5 to 0.8 (Engineering ToolBox friction table). Measure each with a tilt test: lay the book flat, set the clip on it on that jaw's face, tilt until it slides; the friction is the tangent of that angle. Accept the front jaw on a page at 0.4 to 0.6 (22 to 31 degrees; model 0.5) and the back pad on the cover at 0.55 to 0.75 (29 to 37 degrees; model 0.65). mu_lo and mu_hi set the book's side so both contacts sit at the low ends, then at the high ends.
  - Front jaw tip. Bound: moulded ABS walls are 1.14 mm or more (Protolabs' design guide), thin-wall moulding goes down to 0.5 mm (Stratasys' guide), and a tapered edge ends thinner than the wall. So the tip's end lies between 0.5 and about 1.1 mm. Measure it with callipers and accept 0.5 to 1.0 mm (model 0.6 mm; tip05, tip10).
- The neck and head are the Gritin's full size (19.5 cm, 82.7 x 40.3 mm); the stand and the light are set so that the bent head is within the arms' reach.
- The nine LEDs are one warm-white cone light (45 degrees half-angle) at the head, its brightness set so the lit page reads on camera; the aim is read from the LED face's normal.
- The demo reads the light's and the book's poses (a policy would see them from its cameras).

## Sources

- Gritin 9 LED rechargeable book light, G1330 (ASIN B08GG42WXY): 63 g, gooseneck 19.5 cm, head 82.7 x 40.3 mm, clip 59.7 mm (retail listings: https://www.amazon.com/dp/B08GG42WXY); jaws open to 3 cm (maker, via manuals.plus).
- Protolabs, injection moulding design guidelines: recommended ABS wall 0.045 to 0.140 in (1.14 to 3.56 mm). https://www.protolabs.com/resources/design-tips/improving-part-design-with-uniform-wall-thickness/
- Stratasys, Injection Molding Design Guidelines (2017): "Thin wall injection molding can produce walls as thin as ... (.020 inch)", 0.5 mm. https://stratasys.com/siteassets/sdm/resources/design-guidelines/injection-molding/injection_molding_design_guidelines_2017.pdf
- Engineering ToolBox, Friction - Coefficients for Common Materials and Surfaces: rubber on cardboard, clean and dry, 0.5 to 0.8. https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
- Washington State Department of Labor and Industries, Caution Zone Checklist (WAC 296-62-05174): pinching with a force of 4 lb or more per hand. https://ergo-plus.com/wp-content/uploads/Caution-Zone-Checklist.pdf
- Pew Research Center, 2026-04-09: 64% of US adults read a print book in the past year. https://www.pewresearch.org/short-reads/2026/04/09/americans-still-opt-for-print-books-over-digital-or-audio-versions-few-are-in-book-clubs/
- American Academy of Sleep Medicine, 2023 Bedtime Routines Survey (2,005 US adults): 29% read before bed. https://aasm.org/baths-books-and-sex-survey-explores-americans-regular-bedtime-routines/
- Allied Market Research, U.S. Book Light Market, 2022-2031: clip-on lights held 62.6% of the US book-light market in 2021. https://www.alliedmarketresearch.com/us-book-light-market-A110798

Gritin is named for identification only; this task is not affiliated with it.

## Licence

MIT
