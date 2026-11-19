# Hold each blister card over a medicine cup and press the due tablets through its foil, none dropped and none extra

In US long-term care (nursing homes, assisted living), pharmacies send each resident's tablets in 30-day blister
cards: one tablet per cell, sealed with foil on the back. A medication pass is a core, several-times-a-day job of
licensed practical nurses, registered nurses and medication aides: for each resident, read the medication
administration record (MAR), find the drugs due at this hour, and push the right number of tablets out of the right
cards into a paper medicine cup, without dropping any and without giving a drug that is not due. A tablet comes out
by pressing its bubble while the card is held over the cup: the tablet tears through the foil and drops out below.
This takes **two hands**: one holds the card up over the cup, bracing it against the press and leaving free space
under the foil, while the other presses the bubble.

The task is the task file `blister_med_pass.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1), with the cards' code in `blister/` and `yampatch.py` beside
it (see [Implementation](#implementation)). It needs no other files or packages to be installed.

**Instruction:** "It is 08:00. Give the resident's 08:00 medications into the medicine cup, as the medication
record says."

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

| Object | Size | Material | Where |
|---|---|---|---|
| 3 blister cards, loose | 27 x 8 cm, 4 mm thick (card and blister layer), 30 g each | white printed card, clear plastic bubbles, foil back | centred at (27, 10), (27, 20), (27, 30) cm, long side along x, label at the near (-x) end |
| 3 card stands, fixed | 20 x 5 x 1.5 cm (x by y by z) | wood | one under each card, centred under it; the card's long edges overhang by 1.5 cm |
| 14 tablets, loose | 9 mm round, 6 mm thick, 1 g | rigid | in the cards' full cells |
| medicine cup, loose | 1 oz paper soufflé cup: 4.4 cm inside diameter, 3 cm tall, 1 mm wall, 4 g | white paper | standing at (30, -8) cm |
| medication record (MAR), flat | US letter page in landscape, 21.6 x 27.9 cm | white paper, black print | centred at (12, -7) cm, text upright from the arms' side |

**The cards.** Each card has 30 cells, 12 mm square openings in 10 rows of 3 at a pitch of 20 mm (along the card)
by 22 mm (across). The first row's centre is 7 cm from the label end; cells are numbered 1 to 30, three to a row,
left to right as seen from the arms and row by row away from the label (the numbers are printed beside the cells).
A full cell has a clear bubble standing 5 mm proud of the card over its tablet, and foil underneath; an empty cell
(already given) is an open hole. The 5.5 cm pharmacy label at the near end has a colour band with the resident's
name and room (M. OKAFOR, Room 14B), the drug, its strength and its directions:

| Card (from the left as seen from the arms: y = 30, 20, 10 cm) | Band | Tablets | Directions | Full cells |
|---|---|---|---|---|
| ATORVASTATIN 20 mg tablet, at y 30 cm | green | pale yellow | Give 1 tablet by mouth at bedtime | 27 to 30 |
| METOPROLOL TARTRATE 25 mg tablet, at y 20 cm | red | pink | Give 1 tablet by mouth twice daily | 27 to 30 |
| FUROSEMIDE 40 mg tablet, at y 10 cm | blue | white | Give 2 tablets by mouth every morning | 25 to 30 |

**The medication record** reads MEDICATION ADMINISTRATION RECORD, M. OKAFOR Room 14B, and a table with the columns
MEDICATION, DOSE, TIME:

| MEDICATION | DOSE | TIME |
|---|---|---|
| FUROSEMIDE, 40 mg tablet, by mouth | 2 tab | 08:00 |
| METOPROLOL TARTRATE, 25 mg tablet, by mouth | 1 tab | 08:00, 20:00 |
| ATORVASTATIN, 20 mg tablet, by mouth | 1 tab | 21:00 |

So at 08:00, 2 furosemide and 1 metoprolol are due, and atorvastatin is not. The print is large (drug names about
1 cm, doses and times 1.1 cm tall), so a wrist camera held over the sheet reads it, as a nurse leans over the record.

**Building it for a real station:** three empty 30-cell blister cards cut to 27 x 8 cm (or narrow 30-day strip
cards), filled with 9 mm placebo tablets (white, pink and pale yellow) and sealed with blister foil, used cells
pushed out beforehand; printed labels; three 20 x 5 x 1.5 cm wooden blocks taped to the table; a 1 oz paper
soufflé cup; the MAR printed on a letter page.

## Scoring

A tablet counts as **out** when it is more than 8 mm from its cell's centre, and as **in the cup** when it is
inside the cup (within its inner radius of the cup's axis, between its floor and rim); an out tablet not in the cup
is **dropped**. A tablet that has come to rest outside the cup (on the table, a block or the cup's outside, below its
rim) stays dropped even if it is put into the cup afterwards: a dropped tablet is discarded, not given.

- **1/3 for each due tablet in the cup** (3 are due: 2 furosemide, 1 metoprolol), minus **1/3 for each dropped
  tablet**, at least 0.
- **0** if the cup holds any tablet that is not due: any atorvastatin, a third furosemide or a second metoprolol.
- **1.0 (success, ends the trial):** the cup holds exactly the due tablets, none is dropped, and all tablets are at
  rest.

On a real station the score is read off the cup and the table (count and colour of tablets, a photo). In
simulation the tablets' poses are read every control step; run with `--scorer task_score`. `BLISTER_DEBUG=1` prints
each foil tear.

## Running

Needs Linux with an NVIDIA RTX GPU (Isaac Sim needs RT cores) and about 200 GB of disk. Install the pinned
adapter, which installs Isaac Lab-Arena and Inspect Robots (from the root of this repository):

```bash
mkdir -p .deps
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git .deps/yam-isaac-lab-arena
git -C .deps/yam-isaac-lab-arena checkout 0059d02a1a91bc6834c1d0846022891df0c78730
ARENA_DIR="$PWD/.deps/IsaacLab-Arena" .deps/yam-isaac-lab-arena/setup.sh
```

In each shell:

```bash
source .deps/IsaacLab-Arena/yam.env
export ISAACSIM_ASSET_ROOT=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0
```

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/blister_med_pass/blister_med_pass.py \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "It is 08:00. Give the resident's 08:00 medications into the medicine cup, as the medication record says." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/blister-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted -E object_jitter=0` (and
`--log-dir runs/blister-demo`). `inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz), about 3 times what the scripted demo needs.
- **Seeds and layout:** one fixed layout. The cards are articulations and the stands and the record are fixed, so
  seeded resets move none of them; `-E object_jitter` would move the loose cup and tablets (the tablets are put
  back in their cells at every reset), so the scripted demo, which is open-loop, is run with `-E object_jitter=0`.
  Each reset restores every card's pose, its foil and bubbles, and every tablet.
- **Runtime:** the scripted demo succeeds after about 828 steps (41 s simulated) with `task_score` 1.0; the run took
  about 5.5 minutes wall-clock on one RTX 5090 shared with another simulation job, including Isaac Sim start-up.

The demo: the right arm first holds its wrist camera 22 cm above the medication record for 2 s to read it (the whole
sheet is in view and legible at 320 x 176). Then, for each due tablet, the left arm pinches the card by its long
edges over the label area, lifts it off its stand, turns it a quarter turn and sets it on the cup's rim, the cell
over the cup and the rim under both halves of the card; the right arm presses the bubble with its closed fingers
until the tablet tears through the foil and drops into the cup. The press aims well below the card and a few
millimetres beside the bubble, as the arm's position control is soft and the fingertip drifts while it pushes (a
teleoperator pressing firmly does the same with the leader arm); it reaches 12 to 16 N on the bubble. Furosemide cells
25 and 26 are given first, then metoprolol cell 27; each card goes back on its stand.
Atorvastatin is left alone.

## Simulation notes

- Each card is one rigid 4 mm slab (card and blister layer together) with the cell openings cut out; it is an
  articulation with two moving parts per full cell. The **foil** is a 2 mm flap under the cell, hinged at one edge
  and held flat by a spring, holding the tablet; the **bubble** is an 11 mm, 3 mm thick pressing face 2 mm above the
  tablet that slides down into the opening on a stiff spring (8,000 N/m, 6 mm travel), so the press on the bubble is
  measured as its travel. The foil **tears** once the bubble is pressed with 10 N (1.25 mm down): the torn flap swings
  open and stays open for the episode, and the tablet falls out below. On a stand or on the table, the support under
  the cell keeps the tablet in.
- The foil flap is 2 mm thick and the tablets are rigid 1 g cylinders, so the contacts stay stable at 240 Hz. Card
  friction uses a fixed 5 mm torsional patch, as a rubber pad on cardboard.
- The cup is rigid: 20 g, a disc floor and 16 wall staves 3 mm thick, with 32 solver iterations, so it takes the
  card pressed onto its rim. The record is a thin fixed box with the print as a texture.
- The stands are part of the setup because a parallel gripper cannot pick a 4 mm card up off a flat table.

## Implementation

| Path | What |
|---|---|
| `blister_med_pass.py` | the task file: the layout, scoring, the scripted demo and the text an agent reads |
| `blister/design.py` | dimensions, the cards' contents, the medication record and which tablets are due (pure Python) |
| `blister/usd.py` | the cards' articulated USD with printed labels, the cup and the record (`pxr`, PIL) |
| `blister/sim.py` | the cards as Arena articulations, the foil's tearing every control step and the episode reset |
| `yampatch.py` | adds to the task-file format at runtime (without changing the adapter's files): spawning custom Arena objects (the cards) and USD files (the cup and the record), a per-step hook (`on_step`) and a reset hook (`on_reset`); it wraps `yam_arena.scene.task_object`, `yam_arena.scene.object_states` and `YamArenaEmbodiment.reset` of the pinned adapter |

The USD files and textures are built at start-up into the system's temporary folder (`yam_tasks/blister`).

## Media

Rendered with `inspect-robots video <log>.json` (top camera, 320 x 176):

- `media/setup.png`: the setup, the demo run's first top-camera frame.
- `media/demo.mp4`: the scripted demo, full score (1.0).
- `media/dream.mp4`: the Dream-1 run at Dream's settings (320 x 176).

## Licences

`assets/fonts/`: DejaVu Sans Bold and DejaVu Sans Mono Bold, Bitstream Vera licence (see `assets/fonts/LICENSE`).
Everything else (cards, cup, record) is built by the code in this folder.
