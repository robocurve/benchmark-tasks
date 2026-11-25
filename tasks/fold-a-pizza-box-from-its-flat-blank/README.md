# Fold a pizza box from its flat blank

A 10 in corrugated pizza box, still the flat die-cut blank it ships as, lies inside-up on a length of 2x8 board with its
front toward the robot. The instruction is "Fold this pizza box from its flat blank and close it: lock the front wall
over the side walls' tabs, then shut the lid with all three of its flaps inside. Leave the closed box on the board and
let go."

Everything is at full size. The board stands in for the edge of a prep counter: it is narrower than the blank, so the
walls and flaps hang over its edges and a hand can get under them.

## Why it matters

Pizza is one of the most familiar take-out foods. USDA's national dietary survey found that "About 1 in 8 Americans
consumed pizza on any given day", and take-out and delivery pizza travels in a box like this one. Most people have
opened and closed a pizza box at home, and the stack of flat blanks being folded behind a pizza shop's counter is a
common sight. The result is easy for anyone to judge: either it is a box, with its walls up, its front locked and its
lid shut, or it is not.

A pizza box reaches the shop as a flat blank. The listing for the box modelled here says "It ships flat to maximize
storage space and can easily be assembled by folding along perforated edges and locking the ends." A shop folds a stack
of them before every rush. The folding is part of packaging food to go, which O*NET lists for counter workers ("Wrap
menu items such as sandwiches, hot entrees, and desserts for serving or for takeout.") and fast food cooks ("Cook and
package batches of food, such as hamburgers or fried chicken, prepared to order or kept warm until sold."); the task is
submitted as an everyday activity, not as a claim about those occupations' numbers.

## What the task asks of a robot

- **Nothing holds until the next fold is made.** A wall folded up to upright and let go springs most of the way back
  down. The front wall stays up only once it is rolled over the side walls' tabs and its two feet are in the base's
  slots. The lid stays shut only with its flaps inside the walls. With two grippers there is no spare hand to hold a
  wall up while another is folded.
- **Breaking the scores in.** The way round that is the one box makers and box folders use: fold a wall on past
  upright, in over the base, and its score is broken in; it then comes back only to about upright and stands by
  itself. The demo does this to both side walls before anything is locked. A control run that folds the walls just
  to upright ends with no lock (see Verification).
- **A roll-over lock.** The front wall has three parts: the wall, a narrow cap strip, and an inner leaf with two feet.
  The side walls' tabs are turned in along the front; the front wall comes up outside them; the cap goes over the top
  and the leaf down behind the tabs, and the feet go into two slots in the base. A tab left outside the roll-over
  gives a box that looks folded and opens when it is picked up; that is scored as not locked.
- **Thin panels that move when pushed.** Every part is 3.2 mm board hinged to its neighbour. Pushing one panel moves
  the others and can slide or lift the whole blank (119 g). One hand presses the base to the board while the other
  folds.
- **Tucking flaps.** The lid has a flap on each side and one at the front. All three have to end up inside the walls,
  which means the side flaps are folded before the lid comes over and the front flap is turned under while the lid is
  still part open.
- **A back wall that cannot be broken in.** Once the side walls are up, the back wall cannot be folded past upright:
  their rear ends are in the way. Raised and let go it leans back, propped by the lid trailing behind it; lifting the
  lid carries it up against the side walls' rear ends, and once the lid is shut the lid's weight on the walls' tops
  keeps it there.
- **Order.** Side walls before tabs, tabs before the front wall, the lock before the lid, the lid's side flaps before
  the lid comes over, the front flap before the lid is down.

The demo below is one way to do it with two parallel grippers used as pushers. The task does not claim it is the only
way.

## Files

| File | What it is |
|---|---|
| `task.py` | the scene, the instruction and the scoring rule |
| `boxkit.py` | the blank: 13 panels, 12 scores, their springs and friction, the crease-set rule, the box's geometry |
| `demo.py` | the scripted demo and the control runs |
| `test_scoring.py` | 37 offline checks of the scoring rule, the geometry and the crease rule |
| `task.json` | the task's details for the board |

## Run a trial

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fold this pizza box from its flat blank and close it: lock the front wall over the side walls' tabs, then shut the lid with all three of its flaps inside. Leave the closed box on the board and let go." \
      --scorer task_score --max-steps 4000

Layout B: set `BOX_LOT=B` in the environment (the blank and the board 6 mm to the robot's right).

## Scripted demo

    python demo.py task.py out_dir                  # the full fold, layout A
    BOX_LOT=B python demo.py task.py out_dir        # layout B (this script reaches 0.75 there)
    python demo.py task.py out_dir --mode still     # control: nothing moves
    python demo.py task.py out_dir --mode noprebreak   # control: walls folded just to upright
    python demo.py task.py out_dir --mode lock      # control: stops after the lock (lid left open)

What the demo does, in order (one hand presses the base to the board whenever the other folds something attached to
it):

1. **Side walls.** The left hand, leaning in from the left with the flat of its closed fingers under the left wall,
   folds the wall up and on over the base to 125 degrees, holds it there, and lets it come back (it rests leaning in
   a little). From inside the box the same hand pushes the wall out to upright, then sweeps the tab at the wall's front
   end round along the front. The right hand does the same on the right.
2. **Front wall.** The right hand, leaning in from the right with its fingertips under the front wall's right end
   (where the wall hangs over the board), lifts the wall to upright outside the two tabs and stays against it.
3. **Roll-over.** The left hand pushes the inner leaf over the top of the front wall and down inside. Then it opens,
   comes down astride the front wall (one finger outside it, one inside behind the leaf) and closes, squeezing wall,
   tabs and leaf together. The right hand presses the cap down until the leaf's feet are in the base's slots. Both
   let go: the front is locked.
4. **To a known place.** The lock's folds slide the box about 2.5 cm sideways and half a centimetre away, and turn it
   a little. The left hand pushes it back sideways by the foot of its left wall until the hand is at a fixed place;
   then the right hand at the foot of the right wall near the front, and the left hand at the foot of the left wall
   near the back, push in together to fixed places, which turns the box square. Last, both hands come down astride the
   locked front wall, one finger outside it and one inside behind the leaf, and close on it at a fixed place: the
   fingers stop 10.0 mm apart, and wall, tab and leaf pressed together are 9.5 mm thick, so the wall is centred between
   them without being squeezed. The box is then at a distance known from the hands' own positions.
5. **Lid side flaps.** With the lid still lying flat behind the box, each hand, leaning well over from its side,
   slides its fingertips in under the free edge of a side flap and turns the flap up and in.
6. **Back wall and lid.** The right hand presses the base. The left hand, leaning in from the left behind the box,
   lifts the back wall by the end that hangs over the board until it stands (the lid, trailing from its top to the
   board, props it at about 80 degrees), then moves back under the lid and lifts it until it stands nearly upright
   by itself. The right hand leaves the base and waits in front, lying nearly flat, fingertips where the lid's front
   right corner will land. The left hand, still behind the lid, pushes it on over until it falls forward onto
   those fingertips, about 40 degrees open.
7. **Front flap.** The left hand pushes the lid's front flap down and under. The right hand lowers the lid, so the
   front flap goes down inside the front wall and the side flaps inside the side walls, and slides out.
8. **Press and let go.** The left hand presses the lid's front down, and both hands go back to rest.

The demo is open loop. It reads nothing about the box from the simulator: not where the box is, not the angle of any
fold. After the lock it does not know where the box has slid to, so it puts it in a known place (step 4), and the
lid's moves are laid out from there and from three fixed angles found in trial runs (the raised back wall settles at
about 80 degrees; lifting the lid carries it to 94.6; the lid comes to rest on the supporting fingers 41.3 degrees
open). The only readings it uses are the arms' own joint positions, to correct a hand's sag under load. Apart from
the roll-over squeeze and the centring pinch on the front wall, every move is a push with closed fingertips.

Limits of the demo, found while building it: the lid's moves are at the end of the left arm's reach behind the box,
and work with the base within about 2 mm of its place, which is why step 4 exists. The lock stage works with the base's
centre 36.0 cm from the robot and failed in six runs with the blank started 4 to 20 mm nearer (the box tipped when the
hands swapped holds), so the second layout moves the blank sideways, not nearer. **The demo reaches full score on
layout A only.** On layout B the same script locks the front and shuts the lid but leaves the lid's front flap outside
the front wall, which scores 0.75: in step 7 the fingertip comes down on the lid a few millimetres from where it does on
layout A, presses the lid down instead of turning the flap, and the box slides 4.5 cm. Two other sideways shifts (6 mm
left, 3 mm right) also ended at 0.75, with a lid flap left outside. These are limits of this scripted demo, not of the task.

## Scoring

Scored once, at the end of the trial, from how the box stands then. A ruler, a protractor or a phone's level, and a
photo are enough.

| Score | What is true at the end |
|---|---|
| 1.0 | The front is locked; the lid is shut (its front edge within 6 mm of the height of the walls' tops, level within 5 degrees, the back wall within 10 degrees of upright); all three lid flaps are inside the walls; both grippers are at least 3 cm from the box; nothing is moving. |
| 0.75 | Locked, and the lid is down (its front edge within 2 cm of the height of the walls' tops) but one of the conditions for 1.0 fails: the lid is not shut, or a flap is outside, or a gripper is within 3 cm of the box, or something still moves. |
| 0.5 | The front is locked and holds by itself: both tabs under the roll-over, both feet in their slots (each foot's end at least 1 mm below the base's top face), both side walls and the front wall within 8 degrees of upright, no gripper within 3 cm of them. |
| 0.25 | Both side walls are up (within 30 degrees of upright) and the front wall's inner leaf is rolled over at least one tab (the tab's free end lies between the front wall and the leaf). |
| 0 | Anything else: nothing done, walls up but no tab under the roll-over, a box that ends off the board. |

Sliding, turning or tipping the box on the board costs nothing; only where its parts are relative to each other counts
(and that it is still on the board). "Locked" is a statement about geometry that can be checked by looking: tabs
between the wall and the leaf, feet through the slots, walls plumb, hands away. On a real box the same check can be
made by lifting the box 5 cm by its front wall: a locked front comes up with its walls standing.

## Verification

Scores are read from each run's `result.json` (the scorer's own report at the end of the run).

| Run | Score | What happened |
|---|---|---|
| Scripted demo, layout A | 1.0 | full score at step 3740 of 4,000; side walls 2.4 and 2.4 degrees off plumb, front 1.5, back 4.9; lid's front edge -1.6 mm from the walls' tops, 0.3 degrees off level; flaps inside: 3 of 3 |
| The same script, layout B (blank and board 6 mm to the right) | 0.75 | not full score: ran to the limit of 4,000 steps; front locked, lid shut, the lid's front flap left outside the front wall (box slid 4.5 cm when the flap was pushed); side walls 2.6 and 2.6 degrees off plumb, front 1.5, back 5.2; lid's front edge +2.3 mm from the walls' tops, 0.6 degrees off level; flaps inside: 2 of 3 |
| Control: nothing moves | 0.0 | blank flat, as it started |
| Control: side walls folded just to upright (88 degrees), no pre-break; the rest of the lock as in the demo | 0.0 | tabs under the roll-over: 0 of 2; feet in slots: 0 of 2; walls at 62, 87 and front -35 degrees |
| Control: side walls pre-broken, their tabs left unfolded, front wall rolled over | 0.0 | tabs under the roll-over: 0 of 2; feet in slots: 0 of 2; walls at 89, 96 and front -6 degrees |
| Control: the demo up to the lock, lid left open | 0.5 | tabs under the roll-over: 2 of 2; feet in slots: 2 of 2; walls at 88, 88 and front 89 degrees; lid flat behind the box |
| Control: the whole demo, but the lid's front flap is not folded under | 0.75 | side walls 2.4 and 2.4 degrees off plumb, front 1.5, back 4.9; lid's front edge +2.0 mm from the walls' tops, 0.5 degrees off level; flaps inside: 2 of 3 |
| Dream-1, 4,000 steps | 0.0 | both grippers reached the blank's side panels and moved along them; no wall left standing |

`test_scoring.py` checks, without the simulator: that no two panels overlap in the flat, locked and closed states; that
the feet pass through the slots and the tabs sit in the pocket between wall and leaf when locked; that the lid and its
flaps are inside the walls when closed; each score tier and each way of falling short of it; that sliding or turning the
box changes nothing; the crease rule (no set until a score is folded past upright; a wall folded to 125 degrees rests
about upright and one folded to 98 does not); both layouts; and task.json's fields and length limits.

## Runtime and seeds

The scripted demo reaches full score after 3740 steps (187 s of robot time). The run limit is 4,000 steps (3 minutes 20 seconds at 20 Hz). Nothing is random: `object_jitter=0`, no
seeds.

## The scores, and breaking them in

Each of the blank's 12 fold lines is modelled as a hinge with a spring toward its rest angle and holding friction.

- The five long scores (base to each wall, back wall to lid) are stiff: spring 0.0083 N m/rad, holding friction
  0.0113 N m. Folded to 90 degrees and let go, a wall comes back to about 65 degrees.
- The seven short scores (tabs, cap, leaf, the lid's three flaps) hold by friction: they stay where they are put.
- **The crease-set rule.** The rest angle of a wall's score (side walls, front wall, back wall) moves when the score
  is folded past upright: one degree of set for each degree past 90, up to 90. A wall folded to 125 degrees then rests
  at about 97 to 100; a wall folded to 90 gets no set and rests at about 65. The set does not go away during a trial.

The rule stands in for what corrugated board does when a score is "pre-broken". Box plants do it by machine so that
blanks set up easily: one patent describes "means for prebreaking the main creases in creased carton blanks in order
that the blanks when glued and folded may be readily set up", in which "all four creases are folded flat through 180
and preferably ironed at the creases", "ironing the creases during the prebreaking operation to render the prebreaking
permanent" (US 2,589,944). Another supplies blanks "with the corner scores of the box blank broken, so that said blank
may be more easily folded for use by operatives" (US 2,273,419), and a third notes that otherwise "the shell will retain
the springiness inherent in the score lines immediately after folding and pre-breaking" (US 4,287,704). A folding guide
for pizza boxes tells the person folding to "Apply sufficient pressure when creasing the folds, especially given the
thickness and stiffness of the material."

What is sourced is that folding a score well past its final angle takes the spring out of it. **The amounts are
estimates:** the spring and friction torques, the rest angle of a score that has not been broken in (65 degrees), and
the one-degree-per-degree set were chosen so that a 2 in wall of B-flute board behaves plausibly, not measured on a
blank. On a real blank the pre-break angle that makes a wall stand will differ; the task's scoring does not depend on
it.

## Getting the real parts

- **The box.** A 10 x 10 x 2 in white corrugated pizza box, B flute, sold flat in bundles of 50 (WebstaurantStore
  item 245CPB10150WBULK, about $16 a bundle (price on the listing, November 2026); the listing gives "Length 10 Inches", "Width 10 Inches",
  "Height 2 Inches", "Flute Type B", and "Easy to assemble with pre-notched sides"). The same maker's kraft box is
  described as "Made of B-flute cardboard, spanning 1/8" in thickness", which is the 3.2 mm used here. Any 10 in
  pizza box whose front wall rolls over the side walls' tabs will do. One trial uses one blank: a folded blank's
  scores are broken in.
- **The board.** A 61 cm (24 in) length of 2x8 lumber, 38 x 184 mm actual, clamped to the table.
- **Marks.** Mark the base's four corners on the board so that every blank starts in the same place: the box's front
  crease 1.5 cm in front of the board's near end, the blank centred on the board.

The listing gives the box's outside size, its flute and that it locks at the ends. It does not give a die line. The
tab, slot, cap, foot and flap sizes in `boxkit.py` and in task.json's setup are estimates of a box of this kind, laid
out so that the parts fit (checked by the offline tests). A real blank's will differ by millimetres; the scoring
uses only relations (tab between wall and leaf, foot through slot, flap inside wall) that carry over.

## What the simulation simplifies

- **Rigid panels.** Each panel is a rigid 3.2 mm slab. Panels do not bend, dent or twist; real board skews, and a
  person uses that skew to work a tab into place. Here every part has to be put where it goes by rotation about its
  scores alone.
- **Scores as hinges.** A real score is a crushed line with some width; here it is a pin joint at the middle of the
  board's thickness. Small gaps (0.5 to 2 mm) are left between neighbouring panels so that they can fold without
  overlapping.
- **The crease-set rule** above is an estimate of a real effect.
- **Friction-held short scores.** Tabs, cap, leaf and flaps stay at whatever angle they are left at.
- **Solver aids.** Every panel has at least 3 g of mass (the lightest real ones are about 1 g) and each hinge a little
  rotor inertia (2e-5 kg m2), so that the 13-body chain stays stable at the simulator's time step.
- **Shortened lid and lid flaps.** The lid's side flaps start 3 cm from the lid's hinge and stop 4.5 cm short of its
  front edge, which leaves room for a gripper's fingers behind the back wall and under the lid's front corner, and the
  lid stops 17 mm short of the front crease, so that its front flap clears the roll-over wherever within a few degrees
  of upright the back wall stands. On the real box the flaps run closer to both ends and the lid closer to the front;
  a real trial may need the flaps' ends trimmed the same way, or narrower fingers.
- **Not tried on a real blank.**

## Nearest tasks

On the board, the nearest accepted tasks fold or close packaging without setting a box up from its blank: "Put up a
folding produce crate's four walls until every corner catch clicks" (rigid walls on pins, spring catches), "Assemble a
slotted partition" (two slotted cards pushed together), "Crimp the board lid onto a filled foil take-out pan by folding
its rim down all the way round", "Wrap a burger to go in a foil-laminated sheet: front and back over the bun, then both
sides up", "Snap a sip lid onto a full 16 oz paper hot cup all round the rim, without tipping or squeezing the cup", and
"Close a dozen-egg carton with its locking flap tucked inside the lid" (a container that is already formed).
"Slit a taped carton's 40 cm seam with a box cutter, short of the product 6 mm below, then draw the blade back in"
opens a carton. One open submission shuts a filled take-out box that is already formed. The pilot set's carton-flaps
task (TK-S2WMM8) closes the four flaps of a shipping carton that is already erect.

Nothing on the board sets a die-cut box up from its flat blank; this task starts from the flat blank and ends with the
box closed: back wall, side walls, tabs, the front roll-over, the lid and its three flaps.

In the popular task sets: BEHAVIOR-1K has `dispose_of_a_pizza_box` (a finished box and half a pizza end up inside an
ashcan) and `make_a_christmas_gift_box` (wrapping paper laid over a packing box with a bell inside); neither folds a
blank, and neither models a score that springs back. dexhand's Paper Half Fold folds one sheet in half; no fold there
has to lock another. No task in LIBERO, RoboCasa, SimplerEnv, DataCenterBench or
RoboDojo folds a box from a blank, as far as their task lists show.

## Sources

- USDA Agricultural Research Service, Food Surveys Research Group, Dietary Data Brief No. 11, "Consumption of Pizza:
  What We Eat in America, NHANES 2007-2010" (February 2014): "About 1 in 8 Americans consumed pizza on any given day."
  https://www.ars.usda.gov/ARSUserFiles/80400530/pdf/DBrief/11_consumption_of_pizza_0710.pdf
- O*NET OnLine, 35-3023.00 Fast Food and Counter Workers: task quoted above.
  https://www.onetonline.org/link/details/35-3023.00
- O*NET OnLine, 35-2011.00 Cooks, Fast Food: task quoted above.
  https://www.onetonline.org/link/details/35-2011.00
- WebstaurantStore, "Choice 10" x 10" x 2" White Corrugated Plain Pizza Box Bulk Pack - 50/Bundle", item
  245CPB10150WBULK: size, flute, "It ships flat ...", "Easy to assemble with pre-notched sides".
  https://www.webstaurantstore.com/choice-10-x-10-x-2-white-corrugated-plain-pizza-box-bulk-pack-bundle/245CPB10150WBULK.html
- The same seller's kraft printed 10 in box (item 245CPB10PRINT): "Made of B-flute cardboard, spanning 1/8" in
  thickness".
- US 2,589,944, "Machine for prebreaking, gluing, folding, delivering, and stacking creased carton blanks" (quotes above). https://patents.google.com/patent/US2589944A/en
- US 2,273,419, "Means for breaking the four scored corners of collapsible paper boxes" (quote above). https://patents.google.com/patent/US2273419A/en
- US 4,287,704, "Folding and gluing machine" (quote above). https://patents.google.com/patent/US4287704A/en
- KimEcopak, "Pizza Box Folding Made Easy: Step-by-Step Folding Instructions": "Fold the Side Panels Upward: Bend the side panels upward along the pre-scored
  lines."; "Secure the Flaps: Insert the smaller tabs into the designated slots on each side."; "Complete the Lid: Fold
  the lid over the top, aligning its edges with the base."; and the line on creasing quoted above.
  https://www.kimecopak.ca/blogs/news/pizza-box-folding
- Wikipedia, "Lumber", table of North American softwood dimensional lumber sizes: a nominal 2 x 8 is 1 1/2 x 7 1/4 in,
  38 x 184 mm. https://en.wikipedia.org/wiki/Lumber
- BEHAVIOR-1K activity definitions for `dispose_of_a_pizza_box` and `make_a_christmas_gift_box`.
  https://behavior.stanford.edu/knowledgebase/tasks/ (task pages `dispose_of_a_pizza_box-0` and `make_a_christmas_gift_box-0`)

## Third-party assets

None. The blank and the board are built from primitive boxes in `boxkit.py` and `task.py`.
