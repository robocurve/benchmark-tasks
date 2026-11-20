# Real-trial kit: no custom fixture

Nothing in this task is a custom-built mechanism. The shoe, the insole and the shoe box are bought items. This folder
holds the parts list, the measurements to take on the real parts before the first trial, and `fit_stiffness.py`,
which turns the insole's 5-minute cantilever test into the task's stiffness parameter.

## Parts (per rig)
| Item | What to buy | Notes |
|---|---|---|
| Shoe | Brooks Ghost 18, men's, US 9, medium (D) width, any colourway | An everyday lace-up running shoe with a removable OrthoLite X-60 sockliner (5.0 mm) and a non-gusseted padded tongue. Take its own insole out and keep it for tracing. Lace it through all six eyelet pairs and tie a bow at a fixed lacing (mark the laces at the top eyelets) so the opening is the same in every trial. |
| Insole | Spenco PolySorb Walker/Runner replacement insole, Cat. No. 38-385, size 3 (US men's 8/9), one of the pair | Polyurethane foam with a fabric top cover; heel 7.50 mm, forefoot 6.80 mm (maker's sheet). Trim it to the shoe's own insole, as the maker's instructions say ("Use the original insole as a guide to trace and trim"), so it is 3 to 5 mm shorter than the inside. |
| Shoe box | The shoe's own cardboard box (about 30 x 19 x 12 cm) with its lid on | It stands beside the shoe; the insole lies on the lid. |
| Tools | 300 mm steel ruler, kitchen scale (1 g), fine marker, a pencil with an eraser end, phone camera, masking tape | For setup, the stiffness test and scoring. |

## Published numbers the model uses
| Quantity | Value | Source |
|---|---|---|
| Inside length | 271.0 mm | RunRepeat lab review of the Ghost 18 (runrepeat.com/brooks-ghost-18), "Internal length" |
| Heel / forefoot stack, drop | 36 / 26 mm, 10 mm | Running Warehouse product page ("Heel Stack: 36 mm", "Forefoot Stack: 26 mm", "Heel-Toe Offset: 10 mm") |
| Original insole thickness | 5.0 mm | RunRepeat: "We measured the Ortholite X-60 insole at 5.0 mm thick." |
| Toe box height | 25.8 mm | RunRepeat, "Toebox height" |
| Tongue padding, gusset | 9.0 mm, none | RunRepeat: "We measured 9.0 mm of tongue thickness"; "Non-gusseted tongue" |
| Midsole width | 116 mm forefoot, 96 mm heel | RunRepeat, "Midsole width" |
| Weight, men's 9 | 283 g (with laces and insole) | Running Warehouse: "10.0 oz \| 283 g" |

So the model's bed (the floor under the insole) is 36 - 5 = 31 mm above the table at the heel and 26 - 5 = 21 mm at
the ball. RunRepeat does not state the size of the pair it measured; the lab heights (37.3 / 25.1 mm, 12.2 mm drop)
are close to the published ones.

## Before the first trial: measure and write down (to measure)
Put each value into `isaac/geom.py` (the name in brackets) and rerun the demo if it differs by more than a few mm.
1. Inside length on the bed, heel counter to toe box end (`LI`, 271 mm): slide the steel ruler in along the bed's
   centre line until it touches the toe box end, read it at the heel counter's lining.
2. Bed height and drop (`B_HEEL`, `B_FORE`, 31 / 21 mm): stand the ruler upright inside the shoe on the bed at the heel
   and at the ball (through the opening), and read the height of the bed above the table on a straightedge laid
   across the outsole's outer edges; or cut-and-measure a worn pair.
3. Heel cup (`CUP_RISE`, 8 mm at the sides): ruler across the heel seat from side to side; read how far the bed's
   edge rises above its centre.
4. Toe spring (`TS`, 10 mm at the outside tip): stand the shoe on the table and measure the gap under the outsole's
   toe tip.
5. Collar opening length (`THROAT`, 130 mm, heel counter to the top eyelets with the laces tied), heel counter height
   (`H_COUNTER`, 65 mm), collar height at the ankle (`H_SIDE`, 50 mm): ruler along and against the inside.
6. Vamp heights above the bed (`ROOF`: 58 mm at the top eyelets, 46 mm at 185 mm, 31 mm at 230 mm): a ruler held
   upright through the opening at the top eyelets; further forward, a thin dowel slid along the bed until it
   touches the vamp's underside, with a pen mark on the dowel at the eyelet edge.
7. Tongue (`TONGUE_FREE` 50 mm above the top lace, `TONGUE_W` 52 mm, `TONGUE_REST_DEG` 60 degrees above the
   horizontal at rest, `TONGUE_K`): measure its free length and width with the ruler; photograph it from the side to
   read its rest angle; push its tip forward until it lies on the laces and feel that it springs back.
8. Shoe mass without its insole (`SHOE_MASS`, 0.27 kg): kitchen scale.
9. The trimmed insole's length and mass (`L` 267 mm, `MASS` 40 g), and its outline widths (`W_W`).
10. The insole's bending stiffness (next section).
11. Pen marks for scoring, all on the insole's top on its centre line: a short line 20 mm from its toe tip; dots at
    30, 100 and 185 mm from its heel end (heel, arch, ball); a line across it 150 mm from its toe tip. Tape ticks on
    the table at the shoe's heel and toe and at the shoe box's corners.

## Measure the insole's stiffness (5 minutes)
No published bending stiffness for a foam insole was found, so the task keeps it as a named, unmeasured parameter,
the foam modulus `E` in `isaac/geom.py` (4 MPa, so the whole insole's bending stiffness EI is about 8.1e-3 N m^2).
1. Lay the insole top cover up on a table, its toe end under a heavy book, and slide it out so the heel end
   overhangs the table edge by 80 mm (mark the edge on the insole with the ruler).
2. With the steel ruler held upright against the table edge, read how far the heel tip hangs below the table top.
3. Repeat at 120 mm and 160 mm overhang. Optionally tape a coin of known mass at the heel tip (a US nickel is
   5.0 g) and repeat.
4. Weigh the insole. Then run, for example, `python fit_stiffness.py --mass 40 --length 267 80:0.8 120:4.5 160:12.9`.
   It prints the modulus E that matches the drops with the task's outline and thickness. Put it in `geom.E`.

With the task's E = 4 MPa and 40 g, the expected drops are about 0.8 / 4.5 / 12.9 mm at 80 / 120 / 160 mm (no coin) and 1.9 / 8.4 / 21.2 mm with a 5.0 g coin.
A drop several times larger means a floppier insole; a much smaller one means a stiffer insole. Both change how hard
it is to push the insole home from the heel, so rerun the demo with the measured value (the demo was checked at half
and twice the value, README).

## Reset (about 1 minute)
Pull the insole out of the shoe by its heel end, lay it top up on the shoe box lid with its heel end 4 cm over the
lid's near edge, toe away from the robot, put the shoe back on its tape marks with its toe away from the robot, check
the bow is at its lacing marks, and push the tongue back so it leans over the opening. The insole is reusable.
Replace it when its top cover is torn or the foam is creased.
