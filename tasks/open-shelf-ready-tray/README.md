# Open a shelf-ready tray of cans: peel the front panel out and down so the perforation tears (a straight lift holds)

A brown corrugated shelf-ready tray of twelve food cans stands on the table, front toward the robot. Its front panel
comes off along a printed perforation, and the opened tray goes straight onto the shelf as the display.

The instruction is: "Open this shelf-ready tray so it can go straight onto the shelf: take its front panel off along the
perforation and leave every can standing in the tray."

Stockers and order fillers open shelf-ready trays all shift when they restock. The panel comes off by hand, with no
knife: the perforation tears when the panel is peeled out and down from it, and holds when it is just pulled straight up
or sideways. A panel torn off the line, a tray dragged off its spot or a can knocked over leaves a tray that can't be
shelved as it is.

What makes it hard for a robot is the peel itself. The cans stand touching, with the front row 8 mm behind the panel,
so a finger has to go down behind the panel into the 3 cm gap between two cans without knocking either. The panel then
has to be turned out and down about the lip, because pulled straight up or sideways it holds (the pull spreads over
every tie), and the lip then has to be ripped from one end. Every pull goes through a 5 kg tray standing loose on the
table, which must not be dragged more than 2 cm.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left); heights above the table. The layout is
fixed (no jitter); both arms start in their ready pose.

| Object | Description |
|---|---|
| Tray | An open-top half-slotted case (FEFCO 0200: a regular slotted case without top flaps) of brown 44 ECT single-wall B-flute board, 3 mm thick, about 260 g, shrink film already removed. Outside 31.0 wide (y) x 24.1 deep (x) x 11.5 high. Its front face at x 25, centred on y 0, square to the table edge. The panel is printed with a green band and a label (decoration only) |
| Removable part | The front wall above a 4.0 cm lip, with the front part of each side wall above a diagonal tear line (die-line below) |
| Tear line | An easy-open perforation: cut through the board except at 15 short ties, 9 along the lip line and 3 down each diagonal, marked with printed dashes. Drawn 1:1 in `tray_template.svg` |
| Cans | Twelve 15 oz steel food cans, 7.5 cm across x 11.1 cm, 425 g, paper labels, standing upright on the tray floor 4 across x 3 deep, touching (7.55 cm apart, centre to centre), the front row 8 mm behind the front panel |

### Die-line of the removable part

```
   side wall (left), seen from outside            front wall, seen from the robot             side wall (right)
   back                         front                                                    front                 back
    |       top edge 11.5    |   |<-------------------- 31.0 -------------------->|   |      top edge       |
    |      9.0    ____.......|   +------------------------------------------------+   |.......____    9.0   |
    |        ____/ .  panel  |   |                removable panel                 |   | panel . \____       |
    |   ____/  . diagonal    |   |                (7.5 high)                      |   |  diagonal  . \____  |
    |  /  . tear line        |   |- - - - - - - - - tear line - - - - - - - - - - |   |      tear line  .  \|
    | . lip top at the front |   |                lip (4.0 high, stays)           |   |                     |
    +------------------------+   +------------------------------------------------+   +---------------------+
```

Each diagonal runs straight from the side wall's top edge, 9.0 cm back from the tray's front face, down to the lip's top
(4.0 cm up), 3 mm behind the front face. The front wall's line runs the full width at 4.0 cm. The front corners are plain
folds, not cut. The ties sit where the sim's ties are: on the lip line at 0, ±3.5, ±7.0, ±10.5 and ±14.0 cm from the
middle, and on each diagonal at a fifth, half and four fifths of its length from the lip (2.3, 5.7 and 9.2 cm).

## The real tray: where it comes from and how to make it

Every trial tears one tray, so trials need a supply of identical trays. They are made from a stock case and the
template in this folder, and each batch is checked with a pull test.

1. **Cases.** Plain FEFCO 0200 half-slotted cases, outside 310 x 241 x 115 mm (inside about 304 x 235 x 109 mm), 44 ECT
   single-wall B-flute kraft board, with the glued joint at a back corner. FEFCO 0200 is a standard style that any
   corrugated box maker makes to size; they arrive flat.
2. **The tear line (about 5 minutes a tray by machine, 10 by hand).** Cut it from `tray_template.svg` (red: cut
   through; the gaps are the ties, with a black tick at each tie's two edges; grey and blue are reference only). Ties this short are what die-makers call nicks: uncut gaps left in a cut line,
   like the ones that hold a die-cut part in its sheet until it is pushed out by hand. The file gives each tie's finished
   width. Cut it by machine (box maker or laser: the default, for the most even ties), or by hand as a fallback:
   - *Box maker:* have it cut into the blanks on its CAD sample-cutting table before the case is glued (the usual way
     shelf-ready samples are made), with the ties set as nicks of the file's widths.
   - *In house, laser cutter* (40 W or more CO2, cuts 3 mm B-flute in one pass, bed at least 600 x 300 mm): slide a
     3 mm MDF sheet inside the flat case so only the top layer is cut, line the template up with the case's folds,
     and cut the front wall and the side wall lying flat beside it; flip the case and cut the other side wall. Finish
     the few millimetres of line that cross the flat case's folded corner with a craft knife (there's no tie there).
     The beam burns away about its own width (the kerf, typically 0.1 to 0.3 mm in board) at each end of every gap, so
     measure the kerf on a scrap cut and regenerate the file with each tie widened by it (`python template.py <diagonal
     tie + kerf> <lip tie + kerf>`). Check a few ties on the test tray with a loupe.
   - *By hand, no laser (fallback)* (about 10 minutes a tray): print the file at 100 % (tiled on A4 or Letter; the scale bar must
     measure 100 mm), cut out the three walls' strips and tape each on its wall of the erected tray, lined up with the
     folds and the bottom edge. Hold a wooden block inside, behind the line, and cut along the red line with a sharp
     18 mm snap-off craft knife against a steel rule, two or three passes through the 3 mm board, stopping and
     restarting at the black ticks so each tie is left standing. Hand-cut ties land within about 0.2 mm, narrower than
     the flute pitch, so their strength scatters more than machine-cut ties: test more spares (step 3).
   Then draw the printed dashes along the line with a fine marker.
3. **Batch pull test (two spare trays per batch, 2 minutes each).** Erect the spare, tape its bottom flaps, and load it with
   the 12 cans. Clip a 32 mm binder clip on the middle of the panel's top edge and hook a digital luggage scale reading
   to 10 g to it.
   - *Lift:* pull straight up to 2.0 kg (20 N) and hold 3 s. No tie may break.
   - *Peel:* pull horizontally toward you, square to the panel, at about 1 cm/s. The first tie must break between 450
     and 900 g (4.5 to 9 N).
   The sim gives about 43 N for the lift and 6.7 N for this peel (`test_task.py`). The peel band brackets the sim's
   6.7 N by about a factor of 1.4 either way, about the scatter of ties cut by hand to 0.2 mm. The lift is held to 20 N: over
   twice the band's top, and under half the sim's 43 N, so a passing tray holds any straight pull a pinch can apply
   (about 7.5 N) with a wide margin. The template's starting tie widths are 0.6 mm on the diagonals and 1.2 mm on the
   lip line, since the sim's lip ties are twice as strong. If the peel needs more than 900 g, narrow all ties by the
   same factor; if the lift breaks a tie or the peel needs less than 450 g, widen them. Regenerate the file with `python template.py <diagonal tie mm> <lip tie mm>`, re-cut and
   re-test. Settle the widths on a test tray before cutting a batch, then test two spares from every batch of up to 10
   trays cut with the same settings (three if cut by hand), and every new delivery of cases. If a spare fails, re-test
   two more; reject the batch if either of those fails.
4. **Each trial (about 2 minutes).** Erect a fresh tray, tape the bottom flaps (48 mm tape along the seam), stand the 12
   cans in it 4 x 3 with the front row 8 mm behind the front wall (an 8 mm spacer), and set the tray on its four
   corner tape marks.

## Tear force

The two pulls in the batch test pin the real tray to the sim. In the sim, the ties' strengths and positions set them.
Both strengths are one calibration factor, `TIE_K = 3` in `task.py`, times base values of 0.6 N (diagonal tie) and
1.2 N (lip tie): 1.8 N and 3.6 N. Pulled square to the panel at its top edge, the tear starts at about 6.7 N (about
9 N at the pinch, 2 cm lower), and the lip holds meanwhile; a straight lift spreads over all 15 ties and holds to about
43 N. A tray that passes the test tears the same way. If a real shelf-ready tray is measured with the same two pulls,
`TIE_K` is the one number to change (the template's tie widths follow it; the bands, these numbers and the runs need
redoing).

For comparison, published tests on die-cut shelf-ready perforations in E-flute board (320 to 410 g/m², five patterns from
1 mm cut / 1 mm tie to 10 mm cut / 3 mm tie; Šarčević et al., 2022) found a tensile index of about 5 to 13 N m/g across
the line and a tear index of 0.6 to 1.0 mN m²/g along it. That is about 2 to 5 N per mm of line pulled straight across,
and 0.2 to 0.4 N torn along the line. So a die-cut line holds far more pulled straight than it takes to tear along it,
which is the behaviour this task rests on. Its straight-pull strength along a 31 cm lip would be hundreds of newtons. This
tray's line is cut through between 15 short ties instead, an easy-open line that holds about 43 N in a straight pull.
The same paper notes that there is no public standard for these perforations: each maker sets its own pattern by
trial and error. We found no published measurement of the hand force needed to peel a shelf-ready front, so the batch
test is the check.

The format itself is a real one. Alfthan et al. tested B-flute shelf-ready cases (3 mm) whose front panel tears away
along perforations together with the front corners of the side walls, as here.

## Model

- **Board:** the tray is one rigid body. The removable part is three rigid pieces of board: the front panel and the two
  triangular side flaps, joined at the front corners by the board's folds (hinges about the vertical, sprung straight,
  0.002 N m per degree).
- **Tear line:** modelled tie for tie, as on the template: 15 ties. Nine sit along the lip, and three sit down each
  diagonal at a fifth, half and four fifths of its length. Each is a joint that holds only against the line being
  pulled open (a one-sided limit) and against shear out of the board's plane, and carries no moment, like a short tie
  of board. It breaks for good past its strength: 1.8 N on the diagonals, 3.6 N on the lip (`TIE_K` 3 times 0.6 N and
  1.2 N). The lip ties sit on the panel's outer bottom edge, where a fold of board pivots.
  - The panel and flaps collide with the tray, 0.1 mm off the board below the line. So a push that closes the line (the
    panel or a flap pushed down) bears on the board, as on a real tray, and never loads a tie: the ties hold no
    compression, and the diagonal ties let a flap slide along its line on the side wall's edge.
  - Peeling the panel out and down about the lip loads the ties farthest from the lip most. So the tops of the
    diagonals tear first (with about 9 N at the pinch, 2 cm below the panel's top edge), and the tear then runs down
    them, each next tie needing less.
  - Once both diagonals are torn, the panel hangs on the lip ties. Pulled at one end, in the panel's plane, the lip
    tears from that end, each next tie needing less (about 10 N for the first), so once the end tie goes the tear
    runs the whole line at once. Pulled at the middle, all nine share it (about 32 N).
  - A straight lift or a sideways pull spreads over every tie and holds (about 43 N).
  - The real trays' tie widths are set by the batch pull test above, so that they tear like these.
- **Torn:** a tie counts as torn once its two ends are more than 2 mm apart.
- **Not modelled:** the board bending, and tears off the line. The real trial checks for those tears, and any such tear
  scores 0.

## Scoring

`score()` judges the end of the trial:

- **0, whatever else:** the tray ever tipped more than 10 degrees (and, on a real rig, the board torn more than 5 mm off
  the perforation line anywhere).
- **1.0:** the panel off (every tie torn) and not in or on the tray (its pieces' middles outside the tray's outline, or
  more than 5 cm above its rim); the tray moved no more than 2 cm at any corner; all 12 cans upright (within 10 degrees)
  in the tray; both grippers' TCPs 3 cm or more from the tray, the panel and the flaps; and everything still (under 1
  cm/s) for 1 s.
- **0.5:** the panel off, but not all of that.
- **0.25:** the tear started (at least one tie torn), the panel still on.
- **0:** nothing torn.

On a real rig: the video, tape marks at the tray's corners, and a look at the torn edges.

## Run a trial

From this folder, with Inspect Robots and the `yam_arena` embodiment installed:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --policy <policy> --max-steps 2400
```

`--policy yam_scripted` runs the scripted demo (`demo()`), and `DEMO_VARIANT` picks a variant (below). The layout is
fixed, so there is no seed. Runtime per trial (one 48 GB NVIDIA GPU, including about 30 s of start-up): about 6
minutes for the full 2,400 steps, under 2 minutes when the trial ends early on success.

Offline checks (no simulator): `PYTHONPATH=<yam-isaac-lab-arena> python -m pytest -q test_task.py`. They cover:
- the geometry and the finger's gap;
- the tear's force balance: a peel tears with about 9 N, the lip holds meanwhile, the tear runs on once started, and a
  straight lift needs about five times the peel; and the peel command's lag builds that much before the peel ends (a
  static model of the arm's joint stiffness);
- the batch pull test's bands against the sim's own numbers, and that `tray_template.svg` has the sim's ties in the
  sim's places and is up to date;
- the ties: a push down closes every tie and is square to every locked axis, the peel turns the panel about the lip
  ties' own line and opens the diagonals, and the panel's and flaps' collision faces stand 0.1 mm off the board;
  `push_down` presses firmly (about 10 N) and its fingertips miss the cans;
- the corner pull: the lip tears from the left end, each tie needing less (about 10 N for the first); its command
  matches the arm's static compliance; the fingers stay clear of the tray and the left flap;
- `hand_over_tray` ends with the TCP inside the let-go distance of the tray, its fingertips 5 mm over the wall,
  touching nothing;
- scoring;
- that every demo waypoint is within reach.

## Scripted demo

1. The left hand turns its fingers to close along x.
2. It lowers them over the middle of the front panel's top edge, 6 mm either side of the board: the back finger goes
   down behind the panel into the gap between the two middle front cans (3 cm wide there), the front finger in front.
3. It pinches the top 2 cm of the panel and peels it out and down about the lip to about 55 degrees, turning the
   wrist with it. The diagonals tear, top ties first.
4. It eases the pinch so the panel rests on the front pad, slides along the top edge to the panel's left end, and
   pinches there.
5. It pulls that end away from the lip line, along the panel, ramped to a commanded 16 N (the arm gives about 1 mm per
   newton there), then lifts it up and out, away from the cans. As it lifts, the lip tears from the left end and the
   tear runs the whole line within one control step.
6. The panel comes free at the edge of the pinch and drops onto the table in front of the tray, lying out flat from
   the lip (about 105 degrees). The hand carries on empty: its scripted carry to the left, put-down and release pass
   with nothing in it, and it parks.

The right hand stays parked. Step 4 is why the lip comes off at all. Pulled straight out at the middle, the panel loads
all nine lip ties at once, about 32 N, and the pinch only holds an in-plane pull by friction (about 7.5 N: 7.5 N of
squeeze on each side of the 3 mm board), so it slips. Pulled at its end, the pull's moment loads the end tie about three
times as hard, and each next tie needs less: the way people rip a panel off from its corner.

| Variant (`DEMO_VARIANT`) | What it does | Expected | Result |
|---|---|---|---|
| `full` | peel the panel, rip the lip from its left end (the panel drops free in front of the tray), park | 1.0 | 1.0 (the six diagonal ties torn at step 149, the nine lip ties at step 406; the trial ended on success at step 664) |
| `nothing` | arms stay parked | 0 | 0.0 |
| `lift` | pinch the panel's middle and pull it straight up (in its own plane), aimed 8 mm up | 0 (holds) | 0.0 (no tie torn) |
| `start_only` | peel the panel down to 55 degrees (the diagonals tear), then hold it there without pulling it off | 0.25 | 0.25 (the six diagonal ties torn at step 149, the lip whole) |
| `hand_over_tray` | the same tear; then the hand comes back and stays with its closed fingertips 5 mm over the tray's left wall until the end, never clear of the tray | 0.5 | 0.5 (all 15 ties torn by step 406; the hand ends 1.8 cm over the rim) |
| `push_down` | press straight down with the closed fingers, aimed 2.5 cm under each edge (about 10 N): the panel's top edge near its left end, the left flap's top edge, the panel's top edge at its middle | 0 (nothing torn) | 0.0 (no tie torn) |

The Dream-1 run scored 0.0 (no tie torn in 2,400 steps).

## Revision (after TK-BY9XSL)

| Review point | Change |
|---|---|
| The README put the tray front at x 20 cm; the setup and code use x 25 | The Scene table now says x 25, as do the setup and `task.py` (`XF = 0.25`). Every other number in task.json, this README and the code was re-checked against `task.py`: tray 31.0 x 24.1 x 11.5 cm, 3 mm board, 260 g; lip 4.0 cm; diagonal tops 9.0 cm back; cans 7.5 x 11.1 cm, 425 g, 3 x 4 at 7.55 cm, front row 8 mm back; 2,400 steps at 20 Hz; scores. The offline-check note on the lift now says five times the peel (it said twice) |
| Check the simulated tear force against the selected real tray, and update the fidelity note | New section "Tear force". The ties are now three times stronger, set by one factor (`TIE_K` 3: 1.8 N diagonal, 3.6 N lip ties). The real tray's tear line is specified tie for tie as in the sim, and each batch must pass two pulls that bracket the sim's own numbers (peel 6.7 N, lift 43 N). Published perforation data are cited for comparison. The fidelity note now says this. With the stronger ties the lip no longer tears when the panel is pulled straight out at its middle, so the demo rips it from the panel's corner |
| (Found in our own Dream-1 run) A press straight down on a flap tore the line in the sim | The ties now hold only against being pulled open or sheared out of plane, and the panel and flaps bear on the board below the line, so a push goes through contact, as on a real tray. New variant `push_down` scores 0.0 |
| Clarity: each trial uses a custom-perforated tray, with no repeatable source or quick preparation method | New section "The real tray". Stock FEFCO 0200 cases (size and board grade given) with the tear line cut from `tray_template.svg`, a 1:1 file made from `task.py` and checked by `test_task.py`. It is cut by a box maker's CAD table or a laser cutter in about 5 minutes a tray, or by hand from the printed template with a craft knife and a steel rule in about 10. There is a batch pull test with a luggage scale and a per-trial reset. The setup and reset notes now say this |

The title now says how the panel comes off. The instruction is unchanged. The tie strengths changed, so the demo, every
variant and the Dream-1 run are new runs, and the videos and screenshot are new uploads.

## Sources

- I. Šarčević, D. Gregor-Svetec, D. Banić, "Validation of Elmendorf method for testing perforated corrugated
  cardboard", *Tehnički vjesnik* 29(6), 1883-1888 (2022), https://doi.org/10.17559/TV-20211126090308 (tensile and tear
  indices of five shelf-ready perforation patterns in E-flute board; no public standard for these patterns).
- I. Šarčević, D. Gregor-Svetec, D. Banić, "Effect of perforations on the loss of corrugated cardboard bending
  stiffness", GRID 2022, 73-79, https://doi.org/10.24867/GRID-2022-p6 (perforations turn the transport case into the
  shelf tray; shelf-ready packs must open easily yet survive transport).
- J. Alfthan, O. Tillander, T. Trost, A. Tysén, A. Kihlstedt, "Failure due to perforation in corrugated board boxes",
  Innventia AB, https://www.diva-portal.org/smash/get/diva2:1511240/FULLTEXT01.pdf (B-flute shelf-ready cases whose
  front and front side corners tear away along perforations).
- FEFCO Code of Designs, style 0200 (half-slotted case), https://www.fefco.org/sites/default/files/documents/FEFCO_Codes_of_Designs.pdf

## Files

- `task.py`: the scene, the tear line's ties, scoring and the scripted demo.
- `bodykit.py`: multi-shape rigid bodies and joints for yam_arena task files (this team's part library). This copy adds
  convex-prism shapes and break forces on its ball and fixed joints.
- `template.py` and `tray_template.svg`: the real tray's tear line, 1:1, drawn from the sim's ties.
- `test_task.py`: offline checks.

No third-party assets: every part is built from boxes, cylinders and prisms in `task.py`.
