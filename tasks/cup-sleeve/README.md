# Put a corrugated sleeve on a full, lidded hot cup

A bimanual YAM task in the station scene. On the counter stand a full 16 oz single-wall paper hot cup with its sip lid
on, and a clear acrylic sleeve organizer holding one flat-packed kraft corrugated cup sleeve on its narrow edge. The
robot must open the sleeve into a ring, get the cup into it narrow end first and bring the sleeve up the cup's taper
until it fits snugly, then leave the cup standing on the counter and let go, never squeezing the full cup across its
wall.

At coffee bars that serve hot drinks in single-wall cups, the barista slips a sleeve on before handing the drink
over. The sleeve comes flat, folded along two creases; it is opened into a ring with both hands, and since the cup is
already full and lidded (the lid is wider than the sleeve's opening), it goes on from the bottom and wedges on the
cup's taper. The cup's thin wall must not be squeezed.

## Why it matters (Societal relatability)

Putting a sleeve on a hot paper cup is something most coffee drinkers have done themselves, not only baristas: hot
coffee to go is a daily habit for a large share of adults, single-wall paper cups are too hot to hold bare, and the
corrugated sleeve is the standard answer, handed over by staff or taken from a dispenser on the self-serve counter.

- **Coffee is drunk daily, and much of it away from home.** "Overall past-day coffee consumption steady at 66% of
  American adults", and "out-of-home consumption among past-day coffee drinkers reached 38%"; also "55% of past-week
  out-of-home coffee buyers ordered at a drive-thru". National Coffee Association, *2026 National Coffee Data Trends
  Fall Report* (Dig Insights, June 8-24, 2026, nationally representative sample of 1,882 US adults), press release of
  15 September 2026: https://www.ncausa.org/newsroom/NCA-Coffee-tops-beverage-charts-again-specialty-coffee-hits-new-high
- **Billions of disposable hot cups a year (UK data).** "Almost half of all hot drinks are now sold in disposable cups. [...] 1 in
  5 people visit a coffee shop on a daily basis. [...] This results in at least 2.5 billion coffee cups being thrown
  away each year in the UK". House of Commons Environmental Audit Committee, *Disposable Packaging: Coffee Cups*, Second
  Report of Session 2017-19, HC 657, published 5 January 2018:
  https://publications.parliament.uk/pa/cm201719/cmselect/cmenvaud/657/657.pdf
- **The sleeve replaced the second cup.** "In the past, two paper cups were frequently nested together for better
  insulation. Then in January 1997, Starbucks introduced a corrugated paper sleeve to be used as an insulating layer in
  place of a second cup." The same report says "the majority of Starbucks' beverage business is take-out" and that
  before the sleeve "Starbucks was double-cupping approximately 60 percent of its drinks". *Report of the Starbucks
  Coffee Company/Alliance for Environmental Innovation Joint Task Force*, 15 April 2000:
  https://library.edf.org/AssetLink/477d3gh10kfo1f6kfnapt48fil82v728.PDF
- **Why sleeves exist.** The cup sleeve was patented by its inventor as US 5,425,497, "Cup holder" (J. Sorensen, filed
  9 November 1993, granted 20 June 1995): https://patents.google.com/patent/US5425497A/en. Its origin story, from a
  drive-through: "The thin paper cup was way too hot to hold." E. Welles, "To Serve & Protect", *Fortune Small
  Business*, 1 November 2001 (archived):
  https://web.archive.org/web/20131119085418/https://money.cnn.com/magazines/fsb/fsb_archive/2001/11/01/312461/
- **Customers sleeve their own cups.** Sleeve dispensers are sold for the self-serve counter, e.g. the Cal-Mil 2050
  "Java Jacket hot cup sleeve dispenser": "Holds hot cup sleeves (sold separately) to help protect customers from burns
  [...] Ideal for self-serve drink counters" (WebstaurantStore product page, viewed 21 November 2026):
  https://www.webstaurantstore.com/cal-mil-2050-java-jacket-hot-cup-sleeve-dispenser/2112050.html

So the act is familiar from both sides of the counter: baristas sleeve cups before handing them over, and customers
take a flat sleeve from the dispenser, open it and slide it onto their own lidded cup. Chains that use double-wall cups
skip the sleeve.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: objects, per-step model and scorer hooks, `success()`, `score()`, `demo()` |
| `sleevegeom.py` | Cup and sleeve geometry, and the measurements the scorer uses (NumPy only) |
| `sleevekit.py` | The bodies, the sleeve's 16 hinges in a closed loop, the board's set, the contact forces |
| `sleevescore.py` | The scorer (NumPy only): latches, end state, partial credit |
| `sleevedemo.py` | The scripted demo (open-loop waypoints, `TASK_DEMO` selects a variant) |
| `bodykit.py` | Multi-shape rigid bodies and step/reset hooks for task files |
| `robotpts.py` | Sample points of the arms' wrists and fingers, from the robot's meshes (hands-clear check) |
| `steptime.py` | Simulated-time and slow-step records for long runs |
| `closeup.py` | The demo video's close-up camera (only with `TASK_CLOSEUP`) |
| `test_sleeve.py` | Unit tests for the geometry and the scorer: `python -m pytest test_sleeve.py` |

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 --instruction "$(python -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --max-steps 2400
```

`TASK_DEMO` selects the scripted variant: `success` (default), and the scorer checks `notfull` (the sleeve left low and
loose round the cup's base: 0.5), `squeeze` (the cup gripped with 9 N: 0) and `flat` (the sleeve never opened; the cup set down where the ring would be: 0). `TASK_SCORE_DIR` writes a
per-step log (`task_log.json`).

## The scripted demo

Open-loop waypoints from the fixed start layout (`sleevedemo.py`); a few constants were measured in the simulator
(where the opened ring stands, where the fingers first touch, where the held cup hangs). The flat sleeve gapes a little
in the organizer: each hand slips its inner finger into the gap and pinches one layer, the hands lift it out and draw
the layers apart into a ring, set it down standing and press its sides round. The right hand lifts the full cup lightly
by its upper wall just under the lid (about 4 N across the wall), carries it over the ring and lowers it in, narrow end
first, onto the counter. The left hand closes lightly round the sleeve's lower part (under 2 N) and slides it up the
taper until it holds; both arms go back to rest.

## Nearest tasks

- Collapse a travel cup and stow its silicone heat sleeve (TK-UG6D3W): there the sleeve is already fitted and the cup is folded; here a flat-packed paper ring is opened with both hands and wedged up the taper of a full cup.
- Snap a sip lid onto the same full 16 oz cup (TK-KNU8YC): that is a snap-fit lid pressed home round the rim; here a closed band is slid over the cup's taper from below.

## Parts (real-world kit)

| Part | Real item | Size | Material |
|---|---|---|---|
| Cup | 16 oz single-wall poly-coated paper hot cup (Dart Solo 316W type) | 88.9 mm across the lip, 58.4 mm across the bottom, 134.6 mm tall | white poly-coated paperboard |
| Lid | Sip lid for 10-24 oz hot cups (Solo Traveler type) | 93 mm across, 18.5 mm tall | black polystyrene |
| Contents | 400 g room-temperature water (stand-in for coffee) | total 0.43 kg | |
| Sleeve | Universal 12-20 oz kraft corrugated cup sleeve | 64 mm tall, 1.5 mm E-flute board, 4 g; opened 64 mm inside at its lower edge, 79 mm at its upper edge; flat, 101 mm crease to crease at its lower edge and 124 mm at its upper edge | kraft corrugated paperboard |
| Organizer | One open-top compartment of a countertop lid and sleeve organizer | inside 20 x 140 mm, walls 30 mm tall, 3 mm thick | clear acrylic |

The sleeve's cone matches the cup's taper, so it fits with its lower edge about 25 mm above the cup's base.

## Scoring (real trial)

Ruler, 1 mm and 4 mm feelers, a phone photo and the trial video; see `task.json`'s `scoring`. Before the trial, ring
the cup with pencil lines 40 and 90 mm above its base. Squeeze: a dent of 10 mm or more on a side, read on the video
against those lines (by the cup-stiffness figure, about 0.63 N per mm a side, GB/T 27590, that is a 6 N squeeze, which
is the simulator's rule), the lid lifted off the rim anywhere, or water out of the cup scores 0. Snug: mark 8 points
round the sleeve's upper edge; a 1 mm feeler slides in between sleeve and wall at no more than 2 of them, and a 4 mm
feeler at none (in simulation: at least 12 of the 16 strips within 1 mm of the wall, none over 4 mm). Height: the
sleeve's lower edge 15 to 40 mm above the cup's base, measured with the ruler at four places round it (level: within
10 mm of each other).

## Model and fidelity

- Cup and lid: one rigid body (a 48-sided cone hull and the lid's skirt and dome), 0.43 kg, centre of mass 58 mm up.
  The water is a fixed mass (no slosh). The wall does not deform: squeezing is a scored event read from the native
  contact forces of each gripper on the cup, and on sleeve strips that are themselves pressed on the cup.
- Sleeve: 16 rigid strips of 1.5 mm board (each a flat strip between two of the cone's lines) joined into a closed
  loop by PhysX revolute joints along those lines on their inner faces. Two are the factory creases. Each hinge has
  an angular spring whose rest angle is the board's set: it starts at the flat pack (the creases' own set is 40
  degrees short of folded flat, so the flat pack gapes a little when free) and follows the hinge once it is bent more
  than 6 degrees (creases 10) past it. Stiffness 4e-4 N m per degree (creases 2e-4), estimated from the force to
  squeeze an opened sleeve flat (about 1 N). Each strip has a numerical rotational inertia (2e-7 kg m^2) for solver
  stability.
- Frictions (estimates): the sleeve's inner (fluted) face 0.3 against the poly-coated cup (the lower of the pair is
  used), its outer face 1.0 under the fingers (the higher of the pair: rubber on kraft); the cup 1.2 static, 1.0
  dynamic (the harness's value for handled objects), about 0.85 against the fingers; acrylic 0.4. The robot keeps the
  harness's own materials. A fingertip's contact patch on the cup wall has a 20 mm torsional radius (a rubber pad on a
  curved wall), so that a pinch resists the cup turning about it.
- The scorer's hands-clear check samples the arms' wrists, grippers and fingers against points on the cup and sleeve.

## Third-party assets

None. All shapes are generated in code.
