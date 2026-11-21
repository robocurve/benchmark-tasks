# Scrape a failed print's cured raft off a Form 4 tank film with the plastic scraper, lift it out on the blade and bin it, without gouging the film

Instruction: *The print failed and its cured raft is stuck to the film in the resin tank. Without touching anything in
the tank with your grippers, clear the raft into the waste tray, leave the film undamaged, and put the tool you used
back in its rest.*

Category: Impact on AI timelines.

## Why it matters

Robot labs print their own grippers, fingers, brackets and housings on resin printers; the Formlabs Form 4 is a
common one. When a print fails, its first layers (the raft) cure onto the film at the bottom of the resin tank, and
nothing more prints until they are off. The film is what the light shines through, so the job is a removal under a
constraint: get the cured resin off without marking the film. The maker's procedure, from Formlabs' support article
"Inspecting and cleaning resin tanks (Form 4 and Form 4L generation)":

- "Use the plastic scraper that comes with the Finish Kit or a similar tool to remove individual pieces of cured
  resin from the resin tank. Apply minimal force to remove the cured resin. Avoid using the corners of the tool to
  dig under the print."
- "Do not use a metal tool to inspect the Form 4 resin tank or remove cured resin from the tank. Metal tools may
  damage the tank film, leading to failed prints or a resin spill."
- "Open the printer cover." "Use the flat edge of the plastic tank scraper to scrape cured resin off of the film."
  "Scrape laterally using the flat side of the tool to get its edge under the cured resin." "Continue scraping or
  twist the tool to lift the resin off of the film."

And from "Resin tank damage and wear (Form 4 generation)": "Failed print stuck to tank film" is the first case it
lists, and "A gouged or punctured tank requires immediate disposal and replacement", with common causes "Using a
sharp tool inside the tank", "Using excessive force to scrape the tank" and "Directly touching the bottom of the
resin tank". The Finish Kit's own parts list: "Plastic tank scraper: Use to inspect and clean the Form 4 resin tank.
Metal scraper: Use to remove parts from the build platform. Do not use this tool to inspect the Form 4 resin tank."

Why this is robot building and not only printer upkeep: in a lab that prints its robots' parts, the printer is part
of the robot-building line, and a failed print stuck in the tank stops it. The maker treats it as routine,
not exceptional: it lists "Failed print stuck to tank film" first among tank faults, gives the Form 4 a dedicated
"Print cleaning sheet" function that is "perfect for removing print failures or hard to find debris", and fits the
Form 4L tank with debris catchers because "Print failures often leave behind small pieces of cured resin that can
easily lead to future print failures". No failure rate is published, so none is claimed here. A robot that can clear
the tank safely keeps a printer turning out robot parts without a person; one that gouges the film writes off a
US$119 tank and risks "a resin spill" onto the light unit. Occupational line: O*NET 17-3024.01 Robotics Technicians,
core task "Build or assemble robotic devices or systems".

What the robot has to do, and why it is hard for a robot: choose the plastic scraper over the metal one that lies
nearer; hold a light flat tool so its edge runs flat on a film inside a tank whose walls stand 32 mm proud, inside a
printer whose deck is 20 cm up (the front wall stops the blade being laid flatter than about 25 degrees where the raft
lies); free the raft without a corner of the edge digging in or the edge chopping down steeply; then get the freed
raft onto the blade, lift it out level and tip it into the tray, and put the tool back. The grippers never touch the
film, or the resin while it is in the tank.

## Scene (true size)

Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left, z up from the bench.

| Item | Product and size | Pose | In the simulator |
|---|---|---|---|
| Printer | Formlabs Form 4: 39.8 W x 36.7 D x 55.4 H cm, 18.3 kg (maker's tech specs). Cover open; build platform, mixer and cartridge taken off | front face x 21.0, centred y 6.0 | fixed: base (deck 20.0 cm up, estimate), rear tower, platform carriage at the top, open cover |
| Resin tank | Formlabs Form 4 Resin Tank, RT-F4-01, US$119; seated and latched, drained | well x 26.5 to 41.5, y -6.5 to 18.5 | fixed: film 20.5 cm up, walls 3.2 cm above the film, well 15.0 x 25.0 cm (estimates; printed area 20.0 x 12.5 cm) |
| Cured raft | a coupon printed in Formlabs Grey V5 standing in for a failed print's raft: 50 x 60 x 8 mm, its film-side edge chamfered 45 degrees for 3 mm all round, 27 g (`fixture/raft_coupon.stl`). Real stuck resin varies in shape; the chamfer gives the blade one defined way in | centre x 37.8, y 12.0, 12 mm in front of the back wall, long side left-right | loose, bonded to the film (below) |
| Plastic tank scraper | Form 4 Finish Kit (US$149 for the kit): white polypropylene, 130 mm long, blade 55 mm wide, 2.5 mm thick with a 0.8 mm edge, handle 58 x 22 x 14 mm, 25 g (sizes are estimates from the maker's photographs) | in its rest at y 12.0, edge at x 18.0, 1 cm up, blade 45 degrees down toward the printer | loose |
| Metal scraper | Form 4 Finish Kit: stainless blade 40 mm wide, 1.2 mm, black handle, 165 mm long, 68 g (estimates) | in its rest at y 23.5, same pose | loose |
| Scraper rests | printed PLA: a 10 x 10 mm rib at 45 degrees on two posts, a lip in front of the edge (`fixture/rest_*.stl`, `fixture/drawings.svg`) | as above | fixed |
| Waste tray | Thermo Scientific Nalgene 6902-1000 autoclavable tray, polypropylene, 260 x 159 x 64 mm, 2 L | centre x 33.5, y 40.0, long side along x | fixed (taped down) |

The layout is the same every trial. The top camera sees the printer, the raft on the film inside the tank, both
scrapers in their rests and the tray beside the printer (the screenshot); the wrist cameras see into the tank as the
hand comes over it.

### The bond (declared model)

The raft is held to the film by one breakable rigid joint at its middle, 5.5 mm above the film. The simulator
releases it for good the first time the load it carries passes 60 N of force or 0.04 N m of torque. A blade whose
edge is under the raft's chamfer lifts that edge, which loads the joint in bending (a peel): the raft lets go with a
few N on the blade. A push low on its side loads it in shear, which it resists far more; a push high on its side tips
it about its far edge and so peels it too, with a harder push. Estimates for a weak bond to a non-stick film: about
0.025 MPa of shear over the coupon's 24 cm2 bonded face, a few N to peel one edge; check them on five real coupons
with a spring scale (side push, one-edge lift). Peel progress, wet resin and the film's give are not modelled. The
scoring does not require the edge to go under: it scores where the raft ends up, the tool used and the film-damage
rules, as the maker's instructions do ("Apply minimal force", no corners, no metal).

## Scoring

Whole-trial zero rules first, then the end-state ladder (the same definitions in simulation and real trials):

- **Score 0** if at any moment a gripper touches the film, or touches the raft while it is in the tank; the metal
  scraper's blade goes below the tank's rim inside the tank; the plastic scraper's edge touches the film tilted 10 degrees or more across its width
  (a corner digging); or its edge is on the film with the blade steeper than 50 degrees. In real trials also 0 for
  any new gouge, dent, crease or puncture in the film (10x loupe with a phone light under the film, against the
  before photo).
- **1.0** raft inside the tray's walls, not on the scraper, scraper back in its rest (edge within 15 mm of its spot,
  turned at most 10 degrees), both grippers 30 mm or more from it, everything still 1 s.
- **0.75** raft in the tray and still 1 s, not being carried on a scraper still in hand; scraper not back.
- **0.5** the same, with the raft out of the tank (middle outside the well seen from above, or more than 1 cm above
  its rim) but not in the tray.
- **0.25** the raft has come free of the film (moved 3 mm or turned 3 degrees from where it was stuck).
- **0** still stuck.

How each is read in a real trial: three phones film every trial at 60 fps: one overhead (tank, rests and tray in
frame); one low at the printer's front looking along x over the tank's front wall, for the edge's tilt across its
width (a corner down shows as the edge line leaning; 10 degrees over the 55 mm edge is 9.6 mm, read against the
blade's width); and one at the tank's right side looking along y over the right wall, for the blade's angle to the
film (a protractor overlay on the paused frame). Videos are stepped frame by frame for those angles and for any
finger contact; photos from above with a steel rule give the end state.

Simulation (task.py): `observe()` runs after every control step. It samples the two grippers' collision hulls
(`link5`, `gripper`, both fingertips, faces every 3 to 5 mm) and zeroes the trial on a point within 2 mm of the raft while
the raft is in the tank (`raft_gap`, `raft_in_tank`) or within 2 mm above the film inside the well; on any of nine points of the metal blade inside the well
below its rim (`metal_in_tank`); on `corner_dig` (one edge corner within 1.5 mm of the film inside the well, the other
9.6 mm or more higher, i.e. 10 degrees over 55 mm); and on `stab` (the lowest of the edge's two corners and middle within
1.5 mm of the film, blade over 50 degrees). It records the raft as freed once it has moved 3 mm or turned 3 degrees, when everything was last
moving (1 mm), when the raft was last moving, and since when every gripper point has been 30 mm or more from the
plastic scraper. `score()` applies the ladder; "not carried on a scraper still in hand" means either the hands have been
30 mm or more from the scraper for 1 s, or every corner and a 5 x 5 grid of the raft's underside is 3 mm or more from
the scraper (`raft_on_scraper`). If the scene or the arms cannot be read at any step the
trial scores 0.

### Shortcuts and what they score

| Strategy | Score | Why |
|---|---|---|
| Do nothing | 0 | still stuck |
| Use the metal scraper (it lies nearer the arm) | 0 | metal in the tank (`RESIN_DEMO=metal` demo) |
| Pick the raft off with the fingers, or push it with them | 0 | gripper on the raft in the tank |
| Steady anything by pressing on the film | 0 | gripper on the film |
| Dig in with a corner | 0 | corner rule (`RESIN_DEMO=corner` demo, edge tilted 12 degrees) |
| Chop down at it with the edge, blade near upright | 0 | stab rule: blade over 50 degrees with any point of its edge on the film |
| Trace round the raft's outline without getting under it | 0 | nothing lifts an edge; the bond holds |
| Drive the blade along about 6 mm up, into the raft's upright side | 0.25 if it is then left in the tank | the push high on the side tipped and freed it (`RESIN_DEMO=shove` demo, below); a sideways shove is not penalised by itself |
| Free it and leave it in the tank | 0.25 | `RESIN_DEMO=leave` demo |
| Tip it out onto the printer or the bench | 0.5 | not in the tray |
| Bin it, keep the scraper or drop it elsewhere | 0.75 | scraper not back |
| Hold the raft on the blade over the tray at the end | 0.25 | carried on a scraper still in hand |
| Chop with the blade steep and one corner down (tilt under 10 degrees) | 0 | stab rule checks the lowest edge point |

One hand: the task is one tool in one hand; the other arm stays at rest in the demo. Nothing in the scoring needs
two hands.

## How this differs from close tasks

- TK-63RTRA (griddle scraping, accepted): a steel scraper on a robust steel plate, many pieces pushed over a lip.
  Here one bonded part on a film that must not be marked, the tool has to be chosen (the metal one is forbidden), the
  blade works inside a 3.2 cm deep tank inside a printer, and the part is lifted out on the blade, not pushed off.
- TK-SANNN9 (gift-card coating, accepted): scratching a coating off with a coin. Here a part is freed by getting an
  edge under it and then carried.
- TK-A8X2KJ (flex a build plate to pop a print, accepted) and TK-XTCCB5 (tree supports off a printed part,
  accepted) and TK-UPP72T (swap a finished print plate, accepted) are the same printer workflow at other steps: the
  plate, the part, the supports. None clears the tank after a failure, and none has a tool used on a part over a
  surface it must not damage.
- TK-LDYESG (gum off a shoe sole with a putty knife, pending): a soft deposit on a tough rubber sole held by the
  other hand.

## Run

From the yam-isaac-lab-arena checkout (the program's YAM embodiment for Isaac Lab-Arena), with this folder as
`tasks/free-the-cured-resin-from-the-form-4-tank`:

    inspect-robots run tasks/free-the-cured-resin-from-the-form-4-tank/task.py \
        --instruction "The print failed and its cured raft is stuck to the film in the resin tank. Without touching anything in the tank with your grippers, clear the raft into the waste tray, leave the film undamaged, and put the tool you used back in its rest." \
        --max-steps 1600 -E object_jitter=0

Add `--policy yam_scripted` for the scripted demo. One trial per launch (`--epochs 1`, the default): the bond is
made when the scene is built and is not re-made inside a run. The scene is deterministic and needs
`object_jitter=0` (jitter would move the raft off its bond). A trial takes about 3 minutes on an L40S.
`RESIN_DEMO=metal`, `RESIN_DEMO=corner`, `RESIN_DEMO=leave` and `RESIN_DEMO=shove` play the shortcut demos above.

Offline checks: `python near_misses.py` (every score level, both sides of each limit, every zero rule).

## Real trials

Equipment: a Form 4 the lab already runs (any Form 4 or 4B), a Form 4 Resin Tank (US$119; a retired one is fine:
only new damage is scored), the Finish Kit's two scrapers, the Nalgene tray, the two printed rests and the placing template
(fixture/), raft coupons printed in Grey V5 (about US$1 of resin each), a 10x loupe, an ULTRAFIRE 501UV-395 PRO UV
torch (395 to 405 nm), nitrile gloves and UV glasses. Reset (about 5 minutes):

1. Take the tank out, drain it into its cartridge, wipe the film with IPA on a lint-free cloth (Formlabs' own
   film-cleaning steps), let it dry.
2. Photograph the film over a phone light through the loupe (the before photo).
3. Push the placing template into the well's back-left corner, put 0.3 mL of fresh Grey V5 in its window (52 x 62 mm,
   1 mm clearance round the coupon), lay a new coupon on it chamfer down, lift the template off, wipe away any resin
   squeezed out round the coupon so none fills the chamfer.
4. Cure the bond from below through the film: tank on two blocks, the torch 30 mm under the coupon for 20 s. Check
   with a spring scale: a 5 N push on the coupon's side does not move it.
5. Re-seat and latch the tank; cover open; platform, mixer and cartridge out.
6. Lean both scrapers in their rests, edges against the lips; empty the tray.

Safety: the tank is drained, so the robot meets only cured resin and a thin cured bond; people wear gloves and UV
glasses during the reset. Nothing is heated or powered during the trial.

Lower-cost rig for a lab without a Form 4: the same tank, scrapers, rests and tray, with the tank taped onto a 12 mm
plywood box of the printer's footprint (398 x 367 mm, top 200 mm up) carrying a 100 mm deep block across its back to
554 mm (the Z tower), the tank at the place the setup gives (about US$40 of plywood). The box keeps what limits the
robot's reach: the deck height, the tank's walls and the tower behind; the trial is otherwise the same.

## Fidelity and estimates

Rigid bodies in Isaac Sim (PhysX, 20 Hz control). Estimates, each with a one-minute check on real hardware: deck and
film height (20.0 and 20.5 cm: tape measure), well size and wall height (15.0 x 25.0 cm, 3.2 cm: rule), the
scrapers' sizes and masses (calipers, kitchen scale), the bond's strength (a spring scale on a coupon's side, then
under one edge). The film is rigid in simulation, so film damage is judged there by the corner and angle rules and in
real trials by the loupe. Friction: raft and scraper on the film 0.30 static, 0.25 sliding; raft on the plastic blade
0.35/0.30 (estimates). The tank is drained; the real job is often done with resin in it, which is not simulated.

## Results

Final runs on task.py sha256 65f3a29c (the shortcut traces below ran on the same scene and scoring before an
unused code path was removed), Isaac Sim on an L40S, `object_jitter=0`, one trial each.

- Scripted demo (`--policy yam_scripted`): 1.0 in 869 steps (43.5 s) of the 1,600-step limit. The raft did not move
  (0.0 mm) while the scene settled and the arm fetched the scraper. The blade held at about 25 degrees in the grip, its
  edge level across its width within 2.5 mm (the corner rule is at 9.6 mm). The raft came free when the edge was about
  4 mm in under its chamfer, was pushed 12 mm to the back wall, rode onto the blade, was lifted out at 7 degrees,
  carried over the tank's left wall and tipped into the tray (it landed upside down). The scraper went back in its rest
  2 mm from its marked spot.
- Dream-1 (same instruction, 1,600 steps): 0.0. It did not refuse and ran the full 1,600 steps (the run log shows 32
  expired requests to the model that were re-asked). The arms moved about near their rest poses and toward the
  scrapers but picked neither up; the raft stayed stuck.
- Shortcut demos (GPU traces of the same scene): `RESIN_DEMO=metal` 0.0: the metal scraper freed the raft and the raft
  even ended in the tray, but the metal blade went into the tank. `RESIN_DEMO=corner` (edge tilted 12 degrees) 0.0: a
  corner on the film with the other corner 9.6 mm up. `RESIN_DEMO=shove`, edge aimed 5 mm up (2 to 4 mm once the arm
  sagged): it still caught the chamfer, freed the raft and then left it in the tank, 0.25 (the "free it and leave it"
  outcome). Edge aimed 8.5 mm up (5 to 7 mm actual, into the raft's upright side): the push tipped the raft about its
  far edge and freed it too, then left it in the tank, 0.25. So in this model a hard push high on the side also frees
  the raft; the score does not depend on how it came free, only on where it ends and on the damage rules.

## Files

- `task.py`: scene, the bond, scoring, the step hook and the scripted demo.
- `parts.py`: multi-shape rigid parts (convex pieces, turned boxes, per-shape friction). `tilt.py`: tilted grips for
  the scripted demo only. Each patches the stock embodiment in the task's own process; nothing is changed on disk,
  and `tilt.py` never affects Dream-1 or scoring.
- `near_misses.py`: scoring counterexamples and step-by-step tests of every rule.
- `fixture/`: STL files (millimetres) for the raft coupon, the two scraper rests and the placing template, and
  `drawings.svg`.

Licences: the scene is built from primitives in `task.py`; the YAM robot and rig come from yam-isaac-lab-arena under
its own licence. No other third-party assets are used.
