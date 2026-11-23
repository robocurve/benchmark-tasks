# Scrape a bee stinger out of a mannequin hand with a bank card's edge

**Instruction:** "Get the bee stinger out of the back of the mannequin hand using only the bank card, then lay the card
down on the table."

A honeybee's barbed sting stays in the skin with its venom sac attached. First-aid guidance says to remove it quickly
by scraping it sideways with a fingernail or the edge of a card. It also says not to pinch it with tweezers or
fingers, because squeezing can push more venom in:

- NHS, Insect bites and stings: "Brush or scrape the stinger sideways with your fingernail or the edge of a bank
  card." "Do not use tweezers to pull out a stinger as you could squeeze poison out of it."
  https://www.nhs.uk/conditions/insect-bites-and-stings/
- MedlinePlus, Insect bites and stings: "Remove the stinger by scraping the back of a credit card or other
  straight-edged object across the stinger." "Do not use tweezers -- these may squeeze the venom sac and increase the
  amount of venom released." https://medlineplus.gov/ency/article/000033.htm
- St John Ambulance, Insect stings: "brush or scrape it off sideways with something firm like a fingernail, credit
  card, or plastic ruler as soon as possible." https://www.sja.org.uk/first-aid-advice/insect-bites-and-stings/
- Cleveland Clinic, Bee sting: "You can use your fingernail, a credit card or gauze to remove the stinger." "Avoid
  using tweezers since they can inject more venom into your skin when you pinch the stinger."
  https://my.clevelandclinic.org/health/diseases/25093-bee-sting

A field study (Visscher, Vetter and Camazine, Lancet 1996; 348:301-302, summarised at
https://bees.ucr.edu/stings.html) found that how quickly the sting is removed matters more than whether it is scraped
or pinched. The task follows the published guidance above; that the guidance says not to pinch is why a pinch scores 0.

The robot pinches a 0.76 mm card by its top long edge, lifts it out of a place-card holder, sets the lower edge on the
curved back of a mannequin hand behind the stinger, and draws it along the skin. The edge has to run low enough to
catch the 3 mm stub at its base. An edge 2.5 mm up only bends the stinger over and leaves it in. The press has to stay
light, because grippy silicone drags on the edge and twists the card in the two-finger grip. Then the card goes down
on the table. Only the right arm is used; the left arm waits out of the way.

Nearest board task: an accepted task draws a splinter out of a fingertip with tweezers. That is a grasp, with the
tool's tips closed on the splinter and pulled along its axis. This task forbids the grasp: a hand-held straight edge is
drawn sideways along curved skin to lever and drag a barbed stub out.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=scrape_bee_stinger_off_mannequin_hand.py -E object_jitter=0 \
    -E control_hz=20 \
    --instruction "Get the bee stinger out of the back of the mannequin hand using only the bank card, then lay the card down on the table." \
    --policy yam_scripted --scorer task_score --max-steps 1200
```

- Scene: `station`. Control at 20 Hz. Run limit: 1,200 steps (60 s). The scripted demo plan is 24.8 s; it reaches
  success at step 431 (22 s). A trial takes about 2-3 minutes of simulation, about 6 minutes of wall time on a
  shared GPU.
- No object jitter. The hand and holder are fixed and the card starts in one place, so the seed changes nothing.
- `--scorer task_score` gives 1.0 on success and otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_scrape_bee_stinger_off_mannequin_hand.py` (generated scorer tests on the start, goal,
  partial and cheat layouts, and the demo's reach).

## Scene and dimensions

World frame in metres: x forward, y left, z up, origin midway between the arm bases. The table top is z 0.7475.

| Object | Real item | Model | Size and pose |
|---|---|---|---|
| Mannequin hand | Life-size pose-able right silicone mannequin hand, display prop (silicone over a wire armature, about 0.45 kg; listing below), fingers straightened | fixed body: 25 convex pieces of our own (back of the hand, cut wrist, 4 knuckles, 12 finger bones, 4 nails, 2 thumb bones, the weal) | 198 mm wrist crease to middle fingertip; about 78 mm across the knuckles, 120 mm with the thumb; back of the hand 27 mm high at the wrist and 23 mm at the knuckles (superelliptic cross-section, flat palm); cut wrist 62 x 43 mm; palm down, fingers toward +x, thumb toward +y; middle of the knuckle line over (0.270, -0.100); wrist crease at x 0.170 |
| Fixing | Two strips of double-sided carpet tape under the palm | the hand is a fixed body | - |
| Sting site and weal | 12 mm disc of skin-safe red marker | thin red lens, top 0.15 mm proud of the skin | (0.220, -0.100, 0.7724): over the middle-finger bone, 50 mm behind the knuckle line, 24.9 mm above the table; the back of the hand falls 2.4 deg toward the knuckles here |
| Stinger stand-in | 4 mm of 0.25 mm black nylon monofilament with a 1.5 mm black 15/0 seed bead glued on its outer end | two light links (below) | 1.5 mm in the skin, 1.5 mm of shaft out, the bead above: 3.0 mm tall; collides as a 0.8 x 0.8 mm shaft and a 1.5 mm ball |
| Card holder | Beech place-card holder block, taped down | fixed: base and two jaws | 24 (x) x 70 (y) x 16 mm high, centre (0.200, -0.300); 1.2 mm slot along y, floor 6 mm above the table (10 mm deep) |
| Card | CR80 PVC card (ISO/IEC 7810 ID-1), blue | free box, 4.8 g | 85.6 x 54 x 0.76 mm; on its long edge in the slot, long edge along y, centre (0.200, -0.300, 0.7807); top edge 60 mm above the table |

Friction (static/dynamic, estimates, not measured): hand silicone 0.9/0.8, card 0.6/0.5, holder 0.4/0.3, stinger
0.6/0.5.

### The stinger mechanism

The stinger is two links on the hand:

1. `sting_root`: a slide along the skin (x, the hand's long axis), from -12 to +12 mm. It is held by a static
   friction of **0.11 N** (dynamic 0.09 N). This is the barbed shaft's hold.
2. `stinger`: a hinge at the skin about y, from -80 to +80 deg. It carries the shaft and the bead. Static friction is
   2e-5 N m, enough to keep it upright; a touch of about 13 mN at the bead tips it.

What happens depends on where the card's edge meets the stinger:
- An edge that meets the bead or upper shaft only tips the stinger over on its hinge. The edge rides over it and the
  slide never moves.
- An edge on the skin meets the shaft at its base. The push goes into the slide, so once it passes 0.11 N the stinger
  is dragged along the skin out of its hole, falling over as it goes. In the scripted demo it is carried on the
  card's face to the end of its 12 mm track.
- A pinch can also move it. That is caught by the scorer, below.

Sources for the hold force:
- Ling J. et al., "Effect of honeybee stinger and its microstructured barbs on insertion and pull force", J. Mech.
  Behav. Biomed. Mater. 68 (2017) 173-179, doi:10.1016/j.jmbbm.2017.01.040. They measured an average pull-out force of
  **113.50 mN** for a honeybee stinger in rabbit skin, against a penetration force of 5.75 mN.
- The sting is about 2.5 mm long: Ramirez-Esquivel and Ravi, "Functional anatomy of the worker honeybee stinger",
  iScience 26 (2023) 107103, https://pmc.ncbi.nlm.nih.gov/articles/PMC10359947/

The slide's 0.11 N rests on that one pull-out measurement. Pulling along the shaft's axis and dragging along the skin
are taken as needing the same force; that is an estimate. Either way it is far below the robot's sideways push
(about 1-2 N, so 0.4-0.6 N is a comfortable budget). The card's friction on the skin adds about 0.3-0.6 N under the
light press. The tilt hinge's 2e-5 N m is our estimate.

To calibrate a real stand-in:
1. Loop a thread under the bead and hang an 11 g weight from it over a pulley, so the thread pulls along the skin.
2. The stinger should drag out. Choose the needle size and depth so a 9 g weight does not drag it.

## Scoring

The scorer reads the stinger's two joints, the card's pose and box, and the robot's joint state (for the gripper
centre points and openings). It never reads contact forces.

| Rule | Value |
|---|---|
| Card used | The card's lowest point has been more than 5 mm above the holder's top (lifted out) at some time, or the card is no longer in the holder. Without it every rung is 0. |
| Tipped (0.25) | Stinger hinge at 30 deg or more from upright. |
| Out (0.5) | Slide drawn 3 mm or more (twice the 1.5 mm in the skin) **and** the stinger lying over 45 deg or more in the direction it was drawn. Tipped but not drawn, or drawn while upright, is still in. Either direction of scrape counts. |
| Card down (0.75) | Also: the card's lowest point within 3 mm of the table top; its box at least 5 mm clear of the hand's box in plan (not on or against the hand); both grippers' centre points at least 30 mm from the card's box. |
| Full (1.0) | Also: both grippers' centre points at least 30 mm from the hand's box; card and stinger at rest (under 0.5 mm and 0.5 deg change per 50 ms control step) for 1 s. |
| Sticky 0 | The stinger is ever pinched: a gripper's pads closed to 4 mm or less with the bead between them (within 10 mm of the pad centre across the pad width and between the pad tips and 12 mm above the tool point). |
| Success | 1.0, held for 1 s (`SUCCESS_HOLD_S`). |

Scored 0 by the offline tests: doing nothing, a stinger drawn while upright, a card that never left its holder, and a
pinch earlier in the trial. These score below 1.0: a stinger only tipped (0.25), drawn 2 mm (0.25), a card left on the
hand (0.5), and a card dropped against the hand (0.5).

**In a real trial:**
1. After the robot lets go, photograph the back of the hand through a 5x magnifier with a millimetre scale.
2. The stinger is out if it lies loose 3 mm or more from its needle hole, or has gone, and the hole is empty. It is
   tipped if it leans 30 deg or more on a side photo.
3. The card is down if it lies on the table clear of the hand.
4. Check the top and wrist videos for any pinch of the stinger between the pads.

## Scripted demo

The right arm only, all grips straight down with the pads closing along x.
1. Pinch the card's top 18 mm in the holder and lift it 70 mm.
2. Carry it over the back of the hand and lower its edge onto the skin 16 mm behind the sting (toward the wrist),
   commanded 0.8 mm into the skin.
3. Draw it 36 mm toward the knuckles in 3.5 s, following the back of the hand's slope. The arm lags about 7 mm. The
   edge rides on the skin, catches the stinger at its base and drags it out.
4. Lift the card, carry it to the free table beside the hand (around (0.22, -0.20)), set its edge on the table and
   let go. It falls flat.
5. Raise the gripper 75 mm and wait. From there the right wrist camera looks across the back of the hand, so the
   last frame shows the stinger lying out of its weal and the card on the table.

The demo reads no object state: every waypoint is fixed in advance.

GPU runs (station, 20 Hz, no jitter):

| Run | Change | Result |
|---|---|---|
| nominal 1 | - | 1.0, success at step 431 |
| nominal 2 | - | 1.0, success at step 431 |
| timing 0.8x | every waypoint duration x 0.8 | 1.0, success at step 352 |
| timing 1.2x | every waypoint duration x 1.2 | 1.0, success at step 509 |
| control: edge 2.5 mm above the skin | the same plan with the edge held high | 0.25: the stinger tipped 53 deg, drawn only 0.5 mm, still in |
| Dream-1 | the policy, 1,200 steps | 0.0, no refusal: it reached toward the card but never picked it up |

## Real-trial parts (about US$120)

| Part | Listing | Price |
|---|---|---|
| Life-size pose-able right female silicone mannequin hand, about 21 x 11.4 x 6.4 cm | Silicone Sound, https://octopus-dog-rtwe.squarespace.com/shop/p/pose-able-right-female-silicone-mannequin-hand-display-model-prop-lifesize | $64.99 |
| Blank PVC CR80 cards, 30 mil (0.76 mm), pack of 100 (colour one face with a blue paint marker, or use any expired bank or gift card) | J. O'Brien CR8030-91754, https://www.jobrien.com/CR8030-91754-Graphic-Quality-Blank-PVC-Cards-White-Pack-of-100 | $15.99 |
| Wooden place-card holders with a card slot, set of 10 | Wrapables, https://wrapables.com/products/wrapables-rustic-wooden-place-card-holders-with-table-name-place-cards-for-wedding-parties-holidays-special-events-table-decor-set-of-10 | $19.99 |
| Black 0.25 mm nylon monofilament | Beadalon Supplemax 0.25 mm black, https://www.hobbyperline.com/en/stringing-materials/beading-thread/supplemax-beadalon/sm23970-black-supplemax-beading-thread-size-025mm-50mt-spool | about $3 |
| 15/0 opaque black seed beads (1.5 mm outside diameter, 0.7 mm hole) | https://www.gardenofbeadin.com/product/15-0-opaque-black-japanese-seed-bead-401/ | $4.25 |
| Double-sided carpet tape, red skin-safe marker, cyanoacrylate glue, a fine sewing needle, an 11 g and a 9 g weight (coins) with thread, a 5x magnifier | any hardware or craft store | about $12 |

**Making the stinger:**
1. Cut 4 mm of monofilament.
2. Thread a seed bead onto one end so the end is flush with the bead's top, and fix it with a touch of glue.
3. Make a 1.5 mm track in the silicone at the weal's centre with the needle.
4. Push the stand-in in until 1.5 mm of shaft shows under the bead.

**Safety:** a mannequin hand only, no person in the scene. The card edge is blunt. Use the usual robot cell guarding
and an e-stop.

**Reset:**
1. Wipe the back of the hand.
2. Set a fresh stinger in a new needle track inside the weal (re-ink the weal if it has faded).
3. Check the 11 g drag-out on a spare track.
4. Stand the card back in the slot, long edge down.
5. Replace the tape when it stops holding the palm flat.

## Fidelity

- The silicone hand is a rigid body here, taped down in the real trial.
- The stinger links weigh 0.2 g each (a real sting about 5 mg) for solver stability; the stinger's weight still
  does not matter.
- The barbed hold is a stiction on a slide along the skin, and a stinger that is out stays on its 12 mm track rather
  than falling off the hand.
- The card is rigid; a real one flexes a little under the press.

## Files and licences

- `scrape_bee_stinger_off_mannequin_hand.py`: the scene (hand, stinger, holder and card), the scorer and the scripted
  demo. All geometry, including the hand's meshes, is generated by this file from our own code.
- `yam_parts.py`, `yam_joints.py`, `yam_mesh.py`: our parts, joints and procedural-mesh kit, copied unchanged.
- `test_scrape_bee_stinger_off_mannequin_hand.py`: the scorer and reach tests.
- `task.json`.

No third-party models, textures, fonts or materials are used.
