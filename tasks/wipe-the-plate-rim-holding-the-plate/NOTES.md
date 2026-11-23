# Notes: wipe the plate rim, holding the plate

Detail kept out of `README.md` so that it stays short. `task.json` keeps condensed versions of the context and the
fidelity note (the task board limits their length); the full texts are here.

## Context in full

Restaurant cooks (O*NET 35-2014.00, about 1.4 million US workers) list among their core tasks "Portion, arrange, and
garnish food, and serve food to waiters or patrons". BLS OEWS (May 2022) counts 988,060 restaurant cooks in
full-service restaurants (NAICS 722511), where plates go out to seated guests. Wiping the rim is the last step of
plating: Culinary Arts Switzerland says to wipe the rim with a damp cloth before serving, Chefitt to wipe the plate edge
before it leaves the kitchen, and the caterer Personal Touch Dining to wipe every rim before sending a plate out (there
one cook polishes, wiping off drips). The arranged food in the middle must stay untouched while the rim is wiped. Here
the plate stands on a wet stainless-steel pass, and wet porcelain on wet steel slides when a towel is dragged across the
rim: with the same wiping and nobody holding the plate, the recorded run scored 0 because the plate slid away under the
towel. So one arm has to hold the plate still by its rim while the other presses the towel and moves it over each whole
smudge, keeping clear of the food. The instruction says only to clean the rim with the towel pad; it does not say to
hold the plate or to keep off the food. Done by: restaurant and catering cooks at the pass, before each plate goes out.

## Simulation fidelity in full

Rigid bodies: boxes, so the plate is square, the towel pad is a rigid block and the muffin is a disc of fifteen box
slats (their corners reach 5.07 cm from its centre, against 4.75 cm for the real muffin's edge); contact forces are read
from the simulator each step. The simulated plate weighs 0.45 kg; a real 24 cm square porcelain plate is heavier
(roughly 0.65-1.2 kg, an estimate: catalogue weights of 24 cm round porcelain plates of about 0.52-0.96 kg, scaled by
the square's 27% larger area), so the real trial weighs it, and its slide check confirms that it still slides on the wet
pass (a lighter plate within the tolerances is used if it does not). Liquids are not simulated. The wet pass and the
damp towel are friction settings (0.02 on the steel, 0.10 on the plate's base, 0.8 on the towel pad), so a towel pressed
and dragged across the rim pushes an unheld plate along the pass.

The sauce does not smear or soak into the towel: each smudge is drawn as 113 tiles of 2.5 mm over a 3 cm disc, one over
each of the scorer's sample points, and a tile disappears once its point is wiped (a smudge's last tiles once it counts
as clean), so the cameras see the rim come clean where the pad has wiped it. The scorer stands in for cleaning with
these 113 sample points per smudge, each wiped once the pad, pressed on the plate with at least 1.2 N and moving, has
slid 1 cm over it, and a smudge clean when 95% of its points are wiped: up to 5 of its 113 points may stay unwiped,
and their tiles then vanish with the rest, whereas the real rule allows no mark of 2 mm or more.

The 1.2 N threshold is the press the scripted wipe holds: its force controller targets 1.2 N, and while rubbing the
contact force had a median of 1.30-1.45 N; it is the firm contact a rigid pad needs in simulation. Pressing much harder
is a rigid-pad artefact here: when the first version's demo pressed at 4-6 N, the rigid pad dragged the held plate round
5.5 degrees. Real trials score the sauce actually removed, whatever force that takes. The rigid pad is exactly as wide
as a smudge, so it must pass over the whole smudge centred across the rim to within about 2 mm; a soft real pad is more
forgiving there.

The near and right smudges lie on the rim band (between 6.5 and 10.2 cm from the plate's centre, inside the lip). The
far-right smudge's centre is at (5.9, -5.9) cm from the plate's centre, inside the band's inner edge, so it sits at the
band's inner far-right corner, about 45% of it on the band and the rest on the plate's flat middle; band and middle are
one flat porcelain surface. The muffin is a 57 g rigid disc resting loose on the plate, so anything that touches it
pushes it and the contact force is read; the real muffin is soft and crumbly, so a touch leaves a mark.

## Changes in this revision, in detail

- Wiping is scored by area. Each 3 cm smudge is 113 sample points, and a smudge is clean only when 95% of them have had
  the pressed towel slide at least 1 cm over them. The first version counted 10 steps of rubbing with the towel's
  centre within 2 cm of the smudge's centre, which short strokes over one side meet: in `partial` (near 54.0%, right
  36.3%, far-right 46.9% of points wiped, score 0.0) that rule would have counted all three smudges clean.
- A successful trial ends only once all three smudges are clean with the plate within its limits and the towel has
  then been off the plate (contact under 0.05 N, its underside at least 2 cm above the rim band) for 1 s; `score()` is
  unchanged and counts clean smudges. The full demo therefore runs on while it lifts the towel off the plate: it ends
  at step 709 (it ended at step 632, with the towel still on the far-right smudge), and its last frames show the whole
  rim clean.
- The press threshold is 1.2 N (it was 0.8 N): the force the scripted wipe's controller targets, against a median of
  1.30-1.45 N measured while rubbing in the full run.
- The food is a collidable stand-in, a whole English muffin 9.5 cm across, 3 cm tall and 57 g, resting loose on the
  plate. It replaces visual-only boxes that reached 4.75 cm from the plate's centre while the scored food box reached
  3.5 cm. The food rule reads the contact force between the muffin and the towel and five links of each arm, and keeps
  finger pads and the towel 1 cm from every part of it.
- The smudges are 3 cm discs (they were 3 cm squares), matching the 3 cm stencil of the real trials. Each is drawn as
  113 tiles of 2.5 mm, one over each scoring point. A tile disappears from the cameras' view once its point counts as
  wiped, and a smudge's remaining tiles once the smudge counts as clean, so the robot can see which sauce is left and
  the demo's last frames show a clean rim (in the first version the smudges never changed on camera).
- New variants `partial` and `food`; the scripted wipe now uses the towel's measured position and eight strokes per
  smudge.
- The context names the occupation, restaurant cooks (O*NET 35-2014.00), its core task, and the 988,060 who work in
  full-service restaurants (BLS OEWS, May 2022).
- `scorer_check.py` adds eleven edge cases fed straight to the scorer.
- The real trials now give the sauce, its amount, temperature and handling, and the template, as a printable 1:1
  file (`template.svg`); the wetting of the pass and the towel pad, with a slide check, its stop rule and a lighter
  plate as the way out; the food stand-in, its handling and its placement by ruler; tolerances for the plate, the pass
  height and the towel stand, and the plate's mass logged; corner marks and a ruler for the slide and turn; how a
  trial ends (the robot lifts the towel off, and the after photo follows once it has been off for 1 s); a numeric
  clean rule, with any tint judged against the clean plate surface or a white card; the food and lift rules (the plate
  is wiped where it stands, ready for pick-up); and how the grippers are kept clean and dry.
- `kitchenkit.py` records the peak contact force per step, can watch contacts with robot links, and skips a contact pair
  it cannot read instead of stopping all contact readings.

## `report()` fields

`report()` records the towel-plate contact force and the towel's speed, the number of each smudge's points under the
pad, the share of each smudge's points wiped and the clean smudges, the plate's largest move, slide and turn, whether it
was lifted, whether the food rule fired (with the first step and the reasons), the largest contact force on the food
from each body that touched it, the closest any finger pad or the towel came to the food (with which and when), any
contact pair that could not be read, how many sauce tiles each smudge has hidden, any error in hiding them, and how
long the towel has been off the plate (`towel_off_s`).
`old_rules_not_scored` is a diagnostic only: what the first version's rules would have credited (a smudge clean after 10
rubbing steps at 0.8 N with the towel's centre within 2 cm of its centre; a food check on a 7 x 7 x 4 cm box with
finger pads only).

## `scorer_check.py` cases

Set poses and contact forces fed straight to the scorer. Eleven cases, all as expected: the whole smudges wiped (1.0);
one smudge left (0.67); short strokes over one side of each smudge (0.0, which the first version counted clean); the
same strokes as the full wipe pressed with 1.0 N, below the 1.2 N threshold (0.0); the towel pressed on each smudge and
nudged 1.5 mm (0.0, a dab is not a wipe); strokes 2 mm above the rim (0.0); after a full wipe, the right gripper body or
the left arm's finger brushing the food at 0.05 N (0.0); the towel's side 8 mm from the food or a finger pad 7.5 mm from
it (0.0, which the first version's food box missed); and a 0.01 N contact, below the threshold (1.0).

## Variant end states in full

From `demo_results.json`. "Points wiped" is the share of each smudge's 113 sample points wiped. The plate's movement is
its largest distance from its start; its slide and turn are the largest since the towel first touched it.

- `full` (709 steps, **1.0**): success at step 709, with the towel off the plate for the last 1.95 s; points wiped
  99.1% on all three; the plate moved up to 1.5 cm from its start, and after the towel's first touch slid up to 0.38 cm
  and turned up to 0.79 degrees; the food was never touched, and the closest any finger pad or the towel came to it was
  1.18 cm (right finger pads, step 652).
- `no_hold` (896 steps, 0.0): after the towel's first touch the plate slid up to 10.34 cm and turned up to 13.44
  degrees, ending up to 10.34 cm from its start: it slid past the 1 cm hold limit at step 230 and past the 3 cm limit at
  step 262, which set the 0; later, with the plate shifted and turned under the towel, a finger pad also came within
  1 cm of the food (step 620); points wiped near 99.1%, right 95.6%, far-right 95.6%, since the towel followed the plate.
- `light` (942 steps, 0.0): points wiped 0% on all three; the plate moved up to 1.6 cm from its start, and after the
  towel's first touch slid up to 0.51 cm and turned up to 0.34 degrees.
- `two` (758 steps, 0.67): points wiped near 99.1%, right 99.1%, far-right 0%; the plate moved up to 1.6 cm, slid up to
  0.53 cm and turned up to 0.63 degrees.
- `partial` (942 steps, 0.0): points wiped near 54.0%, right 36.3%, far-right 46.9%, so no smudge is clean; the plate
  moved up to 1.64 cm, slid up to 0.57 cm and turned up to 0.62 degrees. The first version's rule would have counted all
  three clean: rubbing steps with the towel's centre within 2 cm, near 31, right 31, far-right 35, against the 10 it
  needed.
- `food` (1020 steps, 0.0): the food rule fired at step 506 (right finger pads within 1 cm); the largest contact force
  on the food was the towel's, 0.41 N. Everything else would have scored 1.0: points wiped 99.1% on all three; the plate
  moved up to 1.64 cm, slid up to 0.5 cm and turned up to 0.80 degrees. The first version's food check fired too, at
  step 518 (right finger pads within 1 cm of its 7 x 7 cm box).
