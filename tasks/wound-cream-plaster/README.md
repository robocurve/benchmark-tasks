# Cream a forearm wound and put on a fabric plaster by its liner tabs

A small abrasion on a forearm trainer is first creamed with a sterile cotton-tipped applicator and then covered with a
large fabric plaster. The robot loads the applicator from a dollop of antiseptic ointment in a medicine cup, spreads a thin
film over the wound and a marked 5 mm margin, and bins the applicator; it must not dip it again once it has touched skin.
Then both hands carry the plaster by its two release-liner tabs and lay its pad centred over the creamed wound,
lengthwise along the arm. Each hand pulls its liner out along the strip, bins it and presses its wing down. The grippers
must never touch the wound, the pad's face or the adhesive, and no cream may lie under the adhesive.

## Why it matters

Dressing minor wounds is a core O\*NET task of medical assistants (31-9092.00, "Change dressings on wounds", 833,900
jobs, BLS 2025) and of licensed practical nurses (29-2061.00, "... dressing wounds, treating bedsores ...", 666,900
jobs). Nursing assistants (31-1131.00, "Apply clean dressings ... under direction of nurse") and EMTs (29-2042.00,
"Administer first aid treatment ...") do it as well. A clean, dry, covered wound is the first-aid standard for the
minor cuts and abrasions seen every day in urgent care, care homes, schools, workplaces and ambulances. The common
failures are physical: a smeared or double-dipped applicator, cream under the adhesive (the plaster lifts), a touched
pad, or a wing left unstuck. Each one means another dressing change and another disturbance of the wound. The same
handling (carry a thin adhesive part by its liner, peel the liner in the right direction, press the adhesive down) is
used for ECG electrodes, island dressings, IV securement dressings and transdermal patches.

## The real build

Positions are in metres from the midpoint between the two arm bases: x forward, y to the robot's left, z up from the
table top.

| Part | What, and where |
|---|---|
| Forearm trainer | Nasco Life/form Suture and Stapling Practice Arm LF01028 (soft vinyl over foam). It lies across the table, palm down, hand to the robot's left (+y), with its axis at x 0.30 and its top 75 mm up. It is strapped with two 25 mm hook-and-loop straps to a 180 x 470 mm, 6 mm plywood base centred at (0.30, 0.065) under a disposable underpad. |
| Wound | A 14 x 7 mm abrasion painted on the top of mid-forearm at (0.30, 0.00) with skin-safe moulage paint: red, with a darker 9 x 4 mm centre, its long axis along the arm. A dotted outline in fine blue skin marker is drawn 5 mm out from its edge: the margin to cover. |
| Cream | 0.5 mL of 10 % povidone-iodine ointment (an antiseptic first-aid ointment; dark brown, so that the film shows on the skin in a photo), pre-dispensed as a dollop on the floor of a 30 mL graduated polypropylene medicine cup. The cup stands on a 70 x 70 x 25 mm white HDPE block at (0.30, -0.205). |
| Applicator | A sterile 6 in (152 mm) wooden cotton-tipped applicator (for example Puritan 806-WC), its bud about 5 mm across and 14 mm long. It lies along x in its peel pouch, opened at the handle end, on a 130 x 60 x 25 mm white HDPE tray at (0.16, -0.17), bud toward the robot, with 50 mm of handle out past the tray's far edge. |
| Plaster | Band-Aid Flexible Fabric Extra Large, 1.75 x 4 in (44 x 102 mm; JOJ5685, 10 per box), pad about 38 mm along the strip and across its full width, re-lined before the trial with two end-tab liners (below). It lies along y, liners down, on its opened wrapper on a 70 x 80 x 35 mm white HDPE riser at (0.44, 0.00), so that its wings and tabs stand out past the riser's ends. |
| End-tab liners | The stock liners overlap at the pad's middle and are meant to be pulled from there, from under the strip, which a gripper cannot reach once the strip lies on the skin. So each plaster gets two C-folded liners whose tabs run out past its ends, as on end-tab dressings. Liner kit: 44 x 127 mm strips cut from siliconised release paper (the backing sheet of plain sticky labels, or a roll of silicone-coated release liner), each folded 51 mm from one end, silicone side out. Re-lining (about 60 s, gloved, touching only the paper and the plaster's back): peel one stock liner and lay that half of the adhesive on the 51 mm leaf of a kit liner, its fold at the middle of the pad and its long leaf running out under the strip, 25 mm past the end; repeat on the other side. Each liner then covers one wing and half the pad, and pulling its tab out along the strip peels it from the middle outward. Five plasters take about 5 minutes. Basis: the HSE (Ireland) first-aid page tells a person to "hold it by the protective strips with the pad facing down", "place the pad on the wound", then "pull away the strips and press the edges of the plaster" (https://www.hse.ie/conditions/wounds/apply/), and dressings such as Smith & Nephew PRIMAPORE have a backing paper in two halves, removed one after the other (https://www.smith-nephew.com/en-au/pharmacy/product-range/healthcare-in-the-home/primapore). The end tabs give a stock plaster that two-handed hold, place and pull sequence. The re-lining moves only where the liner is first taken: the liner still peels from the pad's middle outward, it must still be pulled low along the strip (a steep pull lifts the wing), the strip must still be held while it is pulled, and the pad and adhesive must still never be touched. Release check, once per batch of liner paper: stick a kit liner's short leaf on a spare plaster laid on the film and pull its tab back along the strip with a 1 N spring gauge; it should read about 0.2-0.5 N, the range the simulator's peel force (0.27 N, 0.43 N at the start) sits in. |
| Waste pot | A 1 L open bin, 110 mm across and 110 mm high, at (0.15, 0.02). |
| No-touch check | Disposable nitrile finger-cot pad covers on the gripper fingertips, marked with Glo Germ UV gel (not the loose powder), and a 365 nm UV torch. Containment: the gel is dabbed with a swab on the covers' outer pads only, over a lined tray away from the rig, and left 1 minute to dry; the covers are rolled onto the fingertips and, after the trial, rolled off into a bag over the same tray. The torch checks the bare fingers, the gripper bodies, the table and the underpad before each trial (nothing glows) and after it: a glow anywhere but on the covers, the plaster or the parts in the pot means the covers leaked; wipe it with soap and water, note the trial, and repeat the check until nothing glows. |
| Skin protection | A 10 x 12 cm clear film dressing (3M Tegaderm 1626W) over the wound site, renewed each trial, with the moulage and the outline under it: the ointment and the plaster go on the film, so the iodine never stains the vinyl. |

Rough cost: trainer about $350 (reusable); per trial one applicator ($0.05), one plaster ($0.60), ointment ($0.10), a
medicine cup ($0.02) and two liner strips (under $0.05).

## A real trial and its score

Before a trial: a fresh dollop in a fresh cup, a fresh applicator and plaster laid as above, fresh UV-gel-marked
finger-cot covers fitted over the tray (above), the rig checked under the UV torch, the protective film renewed, the wound and its outline renewed if faded. Four reference ticks are drawn on the
film with a fine marker, outside the plaster's footprint: on the arm's axis 60 mm each side of the wound's centre, and
round the arm 30 mm each side of it. A phone on a stand looks straight down on the wound. Start the video and the timer
(4 minutes).

After the trial:

1. Before the plaster went on (the overhead phone video's frame just after the applicator is dropped in the pot): the
   brown film covers at least 90 % of the outlined zone, counted under a transparent sheet printed with the outline and
   a 1 mm grid laid over that frame printed at 1:1 (the ticks set the scale), and no single smear larger than 2 x 2 mm
   lies more than 3 mm past the dotted outline.
2. From the video: the applicator went into the pot and was never dipped again after touching skin; its bud touched
   nothing but the cream in the cup and the skin: not the cup's rim or wall, the plaster or its tabs, either gripper,
   the trays, the riser, the underpad or the table, nor the pot while still held (falling into it as it is let go is
   fine).
3. With a ruler on the final overhead photo, against the ticks: the pad's centre (the middle of the strip, between the
   wings' fold lines) is within 4 mm of the wound's centre (the midpoint of the ticks) and the strip within 10 degrees of
   the line through the axis ticks. The pad (38 x 44 mm) is larger than the outline (24 x 17 mm) by more than that
   tolerance all round, so a pad within it covers the whole outline.
4. Both liners are in the pot. Both wings lie flat and stuck along their whole length: a 0.25 mm feeler blade does not
   enter under any edge.
5. Peel the plaster off slowly: no cream on either wing's adhesive. Under the UV torch, no UV marker on the pad's face, on
   the adhesive, or on the wound.
6. From the video: both grippers end 3 cm or more from the plaster for 1 s.
7. Roll the covers off into the bag over the tray and check the rig under the UV torch (above).

| Outcome | Score |
|---|---|
| A gripper touched the wound, the pad's face or the adhesive (UV marker); cream under the adhesive; the applicator dipped again after touching skin, or its bud touched anything but the cream and the skin (step 2); the plaster dropped off the arm | 0 |
| Cream done (step 1 and the applicator in the pot, step 2) | 0.25 |
| ... and the pad over the whole outline, one liner off and its wing stuck along its whole length | 0.5 |
| ... and both liners off, at least one wing stuck along its whole length | 0.75 |
| Everything in steps 1 to 6, held 1 s | 1.0 |
| Cream short: 75 to 90 % of the outline covered, no smear past it, the applicator in the pot; and the plaster meets the 0.5 row or better | 0.5 |
| Cream short: under 75 % covered or none, no smear past the outline, the applicator in the pot or never taken; and the plaster meets everything in steps 3 to 6, held 1 s | 0.25 |
| Cream short, and the plaster below those | 0 |

## In simulation

```bash
python -m pytest -q test_task.py                       # the model and the scorer, without the simulator
python run_demo.py OUT                                  # the scripted demo through inspect-robots run, frames in OUT
python run_demo.py OUT --no-frames --mode one_liner     # a failure check (face, drop, one_liner, cream_wing, bud_rim, unheld, none)
# a policy (POLICY), as Dream-1 runs it:
inspect-robots run --embodiment yam_arena --policy POLICY -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 4800 \
  --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')"
```

One trial is up to 4,800 control steps (4 minutes at 20 Hz). The demo scores 1.0 and ends after 3,089 steps (154 s simulated), about 15 minutes on one RTX-class GPU. There is no randomness (object jitter 0).

**What is simulated.** The arms, the forearm (a convex collider 1.5 mm inside the drawn skin, for the vinyl's give),
the trays, the cup, the pot and the riser are rigid bodies on GPU physics. So are the three things the hands hold:

- **Applicator:** a loose rigid body (a 152 mm shaft with a 2.3 mm square collider, so it does not roll as the round
  stick would; the bud as three 5 mm spheres). It
  lies on its pouch; a hand lifts, turns and wipes it only by the friction of its fingers, and it falls into the pot by
  gravity when let go (it counts as binned when its middle lies inside the pot).
- **Liner tabs:** each tab is a loose rigid plate (25 x 44 mm, a 1 mm collider), hinged at its root (the strip's end)
  and sprung to lie flat on the opened wrapper, so that a closing jaw turns the flap into line with its pads; once a
  hand holds it, the hinge is let go. A liner that has come off is its tab: carried in the fingers, dropped into the pot
  by gravity, binned when it lies inside the pot.
- **Held** means native PhysX contact: a hand takes a part when both of its fingertip pads press that part's body with
  at least 1 N for 3 control steps, its jaw closed to within 3 mm of the part's thickness; it has let go once either
  pad stays under 0.2 N for 3 control steps. Nothing is attached to a gripper: the part moves only by friction.

The cream, the plaster and the peeling of its liners are a reduced model (`firstaid.py`), advanced every control step
from the grippers' measured poses, the poses of the applicator and tab bodies, and the native contact forces of each
fingertip on the forearm and on those bodies:

- **Cream:** a 0.5 mm height field on the skin, worked by the bud, as in the paste model of an accepted spackle task. The
  bud loads only in the dollop (up to 0.08 mL). Sliding at least 1.5 mm in 0.15 s along the skin, it leaves a 0.12 mm
  film under its contact patch (1.2 mm across lightly, up to 2 mm squashed), filled from its load, and takes up what
  stands thicker. Held still, it leaves a dot. A dip after it has touched skin is a double dip.
- **Plaster:** 35 x 9 nodes. While carried, it is a flat strip stiffened by its liners, hanging between the two tab
  bodies at the places the fingers hold them. Lying on the forearm, it lies on the skin (the forearm is a straight
  elliptic section over the dressing zone, so the strip lies on it without stretch). Let fall from the air, it settles
  on the arm only from within 10 mm of the skin, roughly level and with no adhesive bared; otherwise it has fallen
  (score 0).
- **Liners:** a taut tab pulled at most 60 degrees above the skin peels its liner from the middle outward, at half the
  tab's travel, if the strip is held against the peel force (below). A steeper pull lifts the wing's end instead. Tabs
  pulled apart in the air peel both liners and bare the pad.
- **Peel force on the tab.** The liner resists the pull. A C-folded liner is a 180 degree peel, so the tab feels the
  liner's release force as a 180 degree peel test measures it. Taken here: 0.15 N per 25 mm of width, an easy-to-medium
  siliconised release paper (medical release liners run from about 5 to 200 g/in, Saint-Gobain's guide to medical
  release liners,
  https://www.globalspec.com/SaintGobainSpecialtyFilms/ref/ReleaseLinerSelectionforDrugDeliveryandMedicalDeviceDesign.pdf;
  easy grades about 5-20 and easy-medium 10-50 g per 2 in, Adhesives Research's release liner overview,
  https://www.adhesivesresearch.com/wp-content/uploads/2022/09/Release-Liner-Product-Overview.pdf; the FINAT FTM 10
  release test is a 180 degree peel of 25 mm strips at 300 mm/min). Over the strip's 44.5 mm that is 0.27 N once the peel front is on the
  adhesive, with a start-up peak of 1.6 times that (0.43 N) over the first 3 mm of adhesive; under the pad, which has no
  adhesive, 0.05 N while the liner slides off and unfolds. It is a native PhysX external force on the tab body, set at
  every physics step (240 Hz) from that body's pose, like a taut string: full once the tab is pulled out as far as the
  peel front's travel needs (twice the front's distance from the strip's middle), falling to nothing over the last
  millimetre of slack. The fingers must hold the tab against it by friction alone; a grip too light lets the tab slip
  and the peel stops.
- **Counterforce on the strip.** The same force acts the other way on the strip at the peel front's row of nodes, along
  the skin toward the tab. The strip resists it with friction on the skin (coefficient 0.5) under its own weight
  (0.8 g) and under any fingertip pads pressing on it (their measured contact forces), and with 0.1 N of shear for each
  tacked node (about 17 mm^2 of adhesive). If that holds, the peel front moves on. If not, and the other hand holds the
  other tab taut with its liner on, the rest of the pull passes through the strip onto that tab body, as the same kind
  of native force, and that hand's grip must hold it. Otherwise the strip slides along with the pull (and the tab feels
  only the strip's sliding friction): pulled with nothing holding it, the plaster is dragged off the wound. Carried in
  the air, each tab carries half the strip's weight, and tabs pulled apart pull on each other with one tension.
  Omitted: the release force's change with peel speed and angle (one 180 degree value for any pull up to 60 degrees
  above the skin), stick-slip after the start-up peak, the paper's stretch, the strip turning (it slides as a whole
  along its axis), the other liner's fold opening when its held tab takes the drag, and the torque from where the force
  acts (at the tab body's centre).
- **Tack:** an exposed adhesive node lying on the skin sticks only under a fingertip pad whose own native contact force
  on the forearm is at least 1 N, within 2 mm of that pad. Tack never spreads. An adhesive node that comes down on cream
  (thicker than 0.02 mm within 2 mm) is spoiled.

Not simulated: wrinkles and creases (in the simulator, "flat" means every wing node lying on and stuck to the skin), the
adhesive's peel strength on skin (tack holds only in shear, as above), cream flow, the bud's
bending and the paper's flexibility (the tab bodies are rigid). Masses are rounded up for the solver (applicator 2 g,
tab 1 g). All values are nominal.

What is drawn: the strip's back in tan fabric with the pad's area as a lighter raised panel (so that the plaster reads
against the skin), on its white liners with blue tabs from each end to where its tab body is held; a loose liner as
white paper on its tab body (the waste pot is dark inside, so binned liners show); the applicator's own body; and the
ointment as a brown film on the skin. The margin's outline is drawn as 1.5 mm dots; at a policy's 320 x 176 cameras
they barely resolve. The simulator scores the cream against the zone's geometry, not against the dots; the dots are
there for the real trial's grid count.

**The bud's contacts.** Once the applicator has been lifted from its pouch and until it lies in the pot, each control
step checks the bud's three spheres against every other body: the medicine cup's wall and rim and its bare floor
outside the dollop, the waste pot (while a hand still holds the applicator; falling into it once let go is binning),
the trays, the riser, the underpad, the table, the strip and both liner tabs, and both grippers' collision boxes. A
sphere within 0.5 mm of any of them is a touch (score 0). The skin and the dollop are the only surfaces it may touch.

**The scripted demo** (`demo.py`) is a scripted reference: it reads the exact poses of visible parts (the applicator's
and the tab bodies' poses, the strip's ends), the fixed scene (the forearm, the wound, the cup, the pot) and its own
joints. It never reads who holds what, the cream heights, the tack, the peel fronts, any contact force or the score. A
policy gets only the cameras and the joints.

The sequence:

1. The right hand takes the applicator by its handle and turns it bud-down. It dips the bud into the dollop and stirs
   once. It wipes eight rows along the arm inside the outline, reading again before each stroke where the bud is (the
   stick can turn a little in the fingers), then drops the applicator in the pot.
2. Both hands take the tabs, lift the plaster, carry it over the wound and lower it, each tab held 12 mm out past its
   end and 16 mm up, so that the strip settles on the skin.
3. The right hand pulls its tab out low and slowly along the strip (about 20 mm/s at the tab, 10 mm/s at the peel
   front), pinched shut, while the left hand holds its own tab still: the strip's pull comes onto that tab. It bins
   the liner, then presses the right wing at 17 points (5 along the wing by 3 round the arm, and its two inner corners) with its closed fingers
   tipped 4 degrees, so that one fingertip pad leads and bears the press (at least 1 N on that pad alone).
4. The right hand presses the pad down beside the wound (on its right half, where no left liner is) while the left hand
   pulls its tab the same way; the stuck right wing and the pressing fingertip hold the strip. The right hand moves
   clear; the left bins its liner and presses its wing. Both hands end clear of the arm.

**Failure checks** (`--mode`):

| Mode | What it does | Score |
|---|---|---|
| `full` | the demo | 1.0 |
| `face` | pulls the tabs a little apart in the air (both liners peel back off the middle of the pad), then the right hand lets go and takes the strip by its pad | 0 |
| `drop` | carries the strip over the wound, pulls the tabs right apart (the liners peel off) and lets go: the bared strip falls | 0 |
| `one_liner` | leaves the left liner on | 0.5 |
| `cream_wing` | wipes cream out along the arm where the right wing will lie | 0 |
| `bud_rim` | wipes the excess off the bud on the cup's rim after the dip, then goes on as the demo | 0 |
| `unheld` | lays the plaster, then the left hand lets go of its tab before the right liner is pulled: nothing holds the strip against the peel force, and the pull drags it off the wound | 0.25 (cream only) |
| `none` | does nothing | 0 |

## Files

| File | What it is |
|---|---|
| `task.py` | the task file: scene objects, per-step hooks, score, success and the demo's entry point |
| `scene.py` | positions and sizes, and the forearm's geometry |
| `props.py` | the fixed bodies (forearm trainer, underpad, trays, cup, pot, riser) and the loose ones (applicator, two liner tabs), all generated meshes |
| `firstaid.py` | the cream, applicator, plaster and liner model; no simulator |
| `rules.py` | the score, following task.json's `scoring` |
| `model.py` | couples the model to the simulator (gripper poses, the loose bodies' poses, fingertip contact forces) and draws it |
| `demo.py` | the scripted demo and its failure modes |
| `run_demo.py` | runs the demo as a policy through `inspect-robots run`, with the same guards as any policy |
| `test_task.py` | unit tests of the model and the scorer |
| `bodykit.py`, `demokit.py`, `gripgeo.py`, `steptime.py` | helpers: multi-shape bodies, closed-loop demos, the gripper's collision boxes, step timing |

## Assets and licence

Every mesh is generated by the code in this folder. There are no third-party meshes, scans or textures. The robot and
the station come with the harness.
