# Set a toilet's two closet bolts: T-heads down the flange slots, slid round to the floor line, retainers pushed down

Set both closet bolts: head down through each slot's wide end, slid round to the yellow line, retainer pushed down onto the ring.

## Scoring

Judged at the end of the run from the top-camera video, a photo and a ruler (in simulation from the poses). A bolt's head is in a slot when its shank stands in that slot (narrow part or wide end) with the head below the ring. A bolt is set when: its shank stands in its slot's narrow part with the shank's centre within 3 mm of the yellow line (lay a straightedge along both line marks); its retainer has been pushed down the thread until its underside is within 2 mm of the ring's top (a 2 mm card does not slide under it); the bolt is within 5 degrees of upright (its tip within 5 mm of a square stood beside it); no gripper is within 3 cm of it, and it has been still for 1 s. A slot is set when a set bolt stands in it. Score 0 if a bolt fell off the bench at any time, or if neither slot holds a bolt's head (doing nothing scores 0). Otherwise the lowest of these numbers that applies: 0.25 if neither slot is set; 0.5 if one slot is set and the other holds no bolt's head; 0.75 if one slot is set and the other is not; 1.0 when both slots are set. Either bolt may go in either slot.

How each check is made on the real rig, from above and with a ruler: the shank stands in the narrow part of the
slot (it cannot stand there unless its head went down through the wide end and under the ring); a straightedge
laid along the two yellow line marks passes within 3 mm of the shank's centre; a 2 mm card will not slide under the
retainer anywhere; a small square stood beside the bolt is within 5 mm of its tip. Nothing in the scoring needs a
force reading or a sub-millimetre measurement. In simulation the same checks are read from the bolts' poses and the
retainer slide's reading: the head's middle below the ring's underside and inside the slot's outline (narrow part or
wide end); the shank's centre where it crosses the ring's top within 3 mm of the line's crossing with the slot; the
retainer's underside within 2 mm of the ring's top; the bolt's axis within 5 degrees of vertical; every gripper's
TCP at least 30 mm from the bolt's bounding box; the bolt moved under 1 mm and turned under 2 degrees over the last
1 s. A bolt below the bench top at any time makes the score 0 for the rest of the run.

In our recorded demonstration (no object jitter) both arms pick their bolts at once; the left arm then holds its bolt out of the way while the right arm sets its bolt (one slot set at 34.2 s), and sets its own after (full credit at 61.6 s; the arms touch if both work at the slots together). At the end both shanks are within 0.6 mm of the line, the retainers 0.6 and 0.7 mm above the ring, both bolts upright to within 0.1 degree. Three trials with every object shifted up to 10 mm: all three reached full credit.

Our wrong-way run slides both bolts to the line and lets go without pushing the retainers down: both bolts stood upright on the line (0.4 and 0.3 mm from it), each retainer still 50.4 mm above the ring: 0.25.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_toilet_closet_bolts_in_the_flange_slots.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Set both closet bolts: head down through each slot's wide end, slid round to the yellow line, retainer pushed down onto the ring." \
  --scorer task_score --max-steps 2400 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. Each trial allows 2,400 control steps (120 s at 20 Hz). No seed is needed
at object_jitter=0; our shifted-object checks used object_jitter=0.01. `pytest` here checks the scorer on named layouts.

## What the robot has to do

Each arm sets the bolt on its own side (each bolt is about 420 mm from the other arm's base, out of its reach); with
both hands at the slots at once the two arms touch, so they take turns there. The bolt stands on its head on its mat; its retainer is started on the tip. Take the bolt, get its T-head
down through the slot's wide end (24 x 14 mm for a 19 x 9.5 mm head) and onto the shelf under the ring, slide it
round the curved slot, out of sight under the ring, until the shank is on the yellow line, then push the retainer
about 51 mm down the thread, against its grip of about 3.5 N, until it bears on the ring, and let go. A bolt left in
the slot without its retainer pushed down is not set: on the job it slides back along the slot or falls over when
the toilet is lowered onto it.

## Real objects (sources fetched; nothing here was measured)

- Flange: Oatey 43495, 3 x 4 in PVC closet flange without test cap, with stainless-steel ring (fits over 3 in or
  inside 4 in Schedule 40 DWV pipe). Oatey technical specification sheet for its Level Fit closet flanges with
  stainless steel ring (product table, 43495: 12 per carton, 9.5 lb):
  https://assets.unilogcorp.com/187/ITEM/DOC/Oatey_100077171_Specification_Sheet.pdf . Retailer listing (7 in
  across, 2 5/16 in tall): https://www.webstaurantstore.com/oatey-43495-level-fit-3-4-pvc-water-closet-flange-with-stainless-steel-ring/83543495.html .
  Retailers also quote Oatey: "bolt slot shelf eliminates bolt fall through".
- Ring sizes: Oatey's submittal for its stainless closet-flange ring for 3 or 4 in flanges (42778): ID 5.08 in, screw
  centre 6.00 in, OD 7.13 in, overall height 0.32 in, six countersunk holes and two closet-bolt slots (its drawing
  shows the two slots 180 degrees apart, each with a wide end):
  https://media.oatey.com/asset/05c634fa-4740-4a35-b0ee-d3a98233f3d0/09_Stainless_Steel_Closet_Flange_Replacement_Ring_SUBMITTAL.pdf ;
  product page (O.D. 7.01 in): https://www.oatey.com/products/oatey-level-fit-closet-flange-with-stainles-steel-replacement-ring-2125802336 .
- Bolts: Danco 88772A closet bolts, 1/4-20 x 2 1/4 in, yellow zinc-plated steel; the pack holds two bolts, two
  plastic retainers, two flat washers and two nuts. https://www.amazon.com/Danco-88772A-Closet-Bolts-2-1/dp/B01ENC9OL6
  (maker page https://www.danco.com/product/14-in-x-2-14-in-closet-bolts/).
- Plywood: 23/32 in subfloor, cut 14 x 14 in; two 2x4s (1 1/2 x 3 1/2 in actual), 14 in long; four 1 1/4 in
  flat-head wood screws; two rubber mats; a yellow lumber crayon. Everything costs under 40 US dollars.
- Occupations: O*NET 47-2152.00 Plumbers, Pipefitters, and Steamfitters (about 510,600 employed; task quoted in the
  context) and 49-9071.00 General Maintenance and Repair Workers.

## Estimates (not published; how to check each in a minute)

T-head 19 x 9.5 x 2.5 mm and bolt mass 16 g (ruler, kitchen scale); retainer 22 mm across, 2 mm thick, 1 g (ruler,
scale); its grip on the thread, about 3.5 N to start it moving and 2.5 N to keep it moving (stand a bolt head-down on a
kitchen scale and push the retainer down with a finger: the scale's reading is the force); slot centre line 75 mm
from the flange's centre, narrow part 7.5 mm wide, wide end 14 x 24 mm, 53 mm end to end (scaled from Oatey's
drawing; ruler); the shelf 3.5 mm below the ring's underside (a ruler through the wide end); ring sheet 1.52 mm
(16 gauge, from a retailer's description); all frictions (steel 0.3, plastic 0.5, rubber 0.9, plywood 0.5).

## Simulation model

- Units metres and kilograms; x forward from the arm bases' midpoint, y left, z up. Floor top 56.4 mm above the
  bench; ring top 8.13 mm above the floor, its underside 6.61 mm, the shelf 3.11 mm.
- `subfloor` (fixed): the plywood as four boxes round the hub's 104 mm square hole, the two 2x4s, the two yellow
  line marks (3 x 57 x 0.2 mm boxes at x = 180 mm).
- `flange` (fixed): PVC hub (44.45-50.8 mm radius, 35 mm below the floor), shelf plate (to 87 mm radius, 3.11 mm
  thick), lip over the ring's inner edge (to 66 mm radius, 0.5 mm above the ring) and the ring's turned-down rim, as
  exact triangle meshes; the ring's top sheet (64.5-89 mm radius, 1.52 mm) as 148 convex pieces that leave the two
  slots open, each slot a 45-degree taper from its wide end into its narrow part; four screw heads.
- `mat_right`, `mat_left` (fixed): 70 x 70 x 3 mm boxes.
- `bolt_right`, `bolt_left` (free mechanisms, 16 g + 1 g): the head as a convex stadium prism, the shank as a
  cylinder; the retainer, a 22 x 2 mm disc, on a prismatic joint along the shank (53.5 mm of travel toward the head,
  stop at the start position) with joint static friction 3.5 N and dynamic friction 2.5 N, damping 0.5, armature
  0.005 kg. Contact offset 1 mm.
- The demo reads the bolts' poses (allowed for scripted demos) to line its hand up; it never moves an object.

## Fidelity

Rigid bodies. The retainer's finger grip on the thread is a slide along the bolt that moves only past about 3.5 N (2.5 N sliding; estimate); the thread is a plain cylinder. Each retainer starts 2 mm onto its bolt's tip, as at reset, because that slide must exist from the start; it cannot come off the tip in simulation. The ring and slots are convex pieces at the published ring sizes; slot, head, shelf and retainer sizes, masses and frictions are estimates.

## Reset

About 2 minutes. Pull each retainer up off its bolt's thread (twist it as you pull), slide each bolt back to its slot's wide end and lift it out. Push a retainer 2 mm onto each bolt's tip by hand and stand each bolt on its head in the middle of its mat, the head's length as in the setup. Check the ring has not turned (the yellow line still crosses each slot 12 mm from its closed end). A retainer whose fingers no longer grip the thread (it slides down under its own weight) is replaced from a new Danco 88772A pack. Return both arms to the ready pose.
