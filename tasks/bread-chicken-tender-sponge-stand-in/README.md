# Bread a chicken tender in flour and lay it on the rack for the fryer, gripping the coated piece as little as possible

Three eighth-size sheet pans stand side by side: a wet chicken tender in the left one, seasoned flour 1 cm deep in the
middle one, a wire rack in the right one. The instruction is "Bread the chicken tender and put it on the rack for the fryer."
Recipes press each face into the flour so the coat adheres ("turning to thoroughly coat and pressing to adhere.
Transfer to prepared rack", America's Test Kitchen, One-Batch Fried Chicken), and the coat sticks to fingers, so the
robot has to press both faces and then turn and move the soft, drooping piece with as few grips as it can. Setup,
scoring and fidelity notes are in `task.json`.

Raw chicken is not practical on shared rigs, so the real trial uses a non-perishable stand-in matched to the raw
tender's size, mass and wet, smooth surface (below), in real flour. The simulation is the raw tender.

## The stand-in: a damp sponge sealed in plastic wrap

- **What it is.** One Scotch-Brite Multi-Purpose (Ocelo) cellulose sponge (Target, 4-pack, no scrubber side, 0.6 in.
  thick), one of the darker colors so the flour shows on it. Cut with scissors to 11.4 x 3.5 x 1.5 cm, soaked,
  squeezed until it stops dripping, sealed in one tight layer of plastic wrap (the ends folded flat like a parcel and
  pressed smooth, clear tape on one long face only) and weighed: about 45 to 50 g (note the weight with the trial). Before each trial it is dipped in water with a
  drop of dish soap per cup, so the water films on the wrap instead of beading, and shaken once.
- **Matched to the tender on.** Size and mass (the simulated tender's body is 11.5 x 3.5 x 1.4 cm and 50 g); a soft
  body that bends when held by one end and has to be turned over by hand; a smooth, non-porous surface wetted with a thin water
  film, so flour sticks where the film takes it rather than lodging in open pores as on a bare sponge. Unlike meat, the
  wrap absorbs no water and is not tacky.
- **Untested.** We found no published data comparing flour adhesion on wet plastic with wet chicken, so how much a
  press sets and a pinch wipes on the wrap, compared with meat, is not measured; nor is how the folded ends and the
  taped face hold flour, or how well the pads grip the wet wrap. Whatever it is, the real trial counts
  the bare area the stand-in actually has; the simulation credits only pressed coat.
- **Reset.** Between trials rinse the flour off under the tap, dip in the soapy water, shake once. After each
  session unwrap it, rinse it and let it dry; soak, squeeze, re-wrap and weigh it at the next (or cut a fresh one).

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/bread-chicken-tender-sponge-stand-in/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Bread the chicken tender and put it on the rack for the fryer." \
      --scorer task_score --max-steps 1600 --store-frames

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. `COAT_DEBUG=1` prints presses, grips and
coverage. Offline checks: `python3 test_task.py` and `python3 test_coatkit.py`.

The demo lays the tender in the flour and presses three spots with its closed fingertips, lifts it by one end, hands it
over in the air to turn it over, presses the other face, then lifts it by the same end, lays it on the rack and lets
go. `DEMO_VARIANT` gives counterfactual runs (never the submitted demo); two are below.

## Scoring

Read once the tender rests on the rack's grid, let go and still for 1 s, by the bare area of its 122.5 cm^2 surface.

| Outcome | Score |
|---|---|
| On the rack, at most 10 cm^2 bare, no hand on it | 1.0 |
| On the rack, at most 25 / 50 / 85 cm^2 bare | 0.75 / 0.5 / 0.25 |
| A hand still on it at the end | capped at 0.5 |
| Breaded (at most 25 cm^2 bare) but left in the breading pan, let go | 0.25 |
| Anywhere else, fallen off the table, or doing nothing | 0 |

- **Why these lines.** Turning the tender and moving it usually take grips. Gripped at one spot each time, the fingertip
  marks stay under 10 cm^2; one extra grip at a new spot costs the top tier. One face left unpressed leaves more than
  50 cm^2 bare.
- **How the simulation decides.** The surface is 2.5 mm cells, bare at the start (`coatkit.py`). A cell is coated
  where the tender is pressed into the flour: the pan pushing up on it several times its weight while a hand pushes it
  down firmly, held briefly, read from PhysX contact forces; only the stretch around that hand is coated. Lying in the
  flour or dropped into it coats nothing. A grip (both fingers pushing from opposite sides, as any pinch that can lift
  it does) wipes the coat under the finger pads; how much comes off is our modeling choice. Coat left touching the
  bare table rubs off. A bare spot pressed again re-coats. Not modeled: a carry without a grip (clamped between two
  fists, or scooped on open fingers) takes no coat off.
- **Real trial.** In diffuse light, no flash, photograph the top, both long sides and both ends with a card ruled in 5 mm squares beside the sponge (nothing
  touches the coat), then lift the rack out by its edges and photograph the underside from below through the grid. A
  square more than half bare (the wrapped sponge's color showing) is bare (0.25 cm^2); a square partly hidden by a wire is judged by its visible
  part, one fully hidden counts as bare. The video shows where it ended and that it was let go.

Counterfactual runs (`DEMO_VARIANT`, same task.py):

| Variant | What changes | Score |
|---|---|---|
| `no_press` | same moves, no presses | 0 (nothing coated) |
| `grip_everywhere` | also gripped at the middle and far end before the carry | 0.75 (19 cm^2 bare) |

## Real-world build

A low-cost setup: three aluminum eighth sheet pans (Nordic Ware, Target; about 9.6 x 6.6 x 1 in.) on strips of
non-slip shelf liner, an eighth-size wire cooling rack, all-purpose flour with 2 % paprika by weight, the wrapped sponge above
(plastic wrap, clear tape, dish soap), and kitchen tools (scissors, scale, sieve, ruler, tongs, a pastry brush, a
printed 5 mm grid card). Positions are in
`task.json` (setup). Nothing hot, sharp or raw is on the rig.

## Runtime and seeds

- Demo: a 75 s plan, run with `--max-steps 1600`; it scores 1.0 on this task.py, succeeding at step 1,444 with 8.8 cm^2
  bare (92.8 % coated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seeds: none. The layout is fixed (`object_jitter=0`).

## Files

- `task.py`: the scene, `score()`, `success()` and the scripted `demo()`.
- `coatkit.py`: the breading model (cells coated by a press, wiped by a grip, drawn as crumbs).
- `yamkit.py`: spawns the modeled props. `guardkit.py`: the grippers' own shapes (contact checks, finger pads).
- `test_task.py`, `test_coatkit.py`: offline checks.
