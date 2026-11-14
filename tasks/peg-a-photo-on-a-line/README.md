# Clip a hanging photo's free corner to a photo wire with a soft-grip clothes peg

A photo print hangs from a taut photo-display wire by one wooden clothes peg at its far top corner. Spare pegs are
clipped on the rim of a peg basket beside it. The robot has to peg the photo's other top corner to the wire too.

**Instruction:** "The photo hangs from the wire by one peg. Peg its other top corner to the wire too."

A clothes peg is two wooden legs on a pivot with a torsion spring: squeezing its handles together opens its jaws, and
letting go closes them on whatever is between them. The task needs the whole cycle with millimetre placement:

- squeeze a spare peg to unclip it from the basket's rim, and keep it squeezed while carrying it (relax the squeeze
  and it snaps shut);
- bring it down so its open jaws pass either side of the 1.2 mm wire and the 0.3 mm photo behind it together, at the
  photo's corner (the jaws open in a V from the pivot, so the wire and photo must enter low in the jaws, where the
  opening is widest);
- let go so the spring clamps both, without knocking the photo, which hangs from a single peg, off the wire.

## Where this comes up

- **Home and classroom displays.** Hanging photos, drawings and cards on a wire or string with pegs is a common way
  to show them at home, in classrooms, nurseries, cafés and shops.
- **Laundry.** The same peg squeeze-and-clip, with the same spring, is how washing is pegged out on a line.
- **Labs and workshops.** Spring clips hold drying prints, gels and labels on lines in darkrooms and labs.

## Potential impact

- **A spring mechanism worked by the gripper.** The robot has to squeeze, hold and release a sprung tool rather
  than just grasp a rigid object, and the tool's state (open or shut) depends on how hard it is held.
- **Thin-object alignment.** The jaws have to straddle a 1.2 mm wire and a 0.3 mm print at once, with under 1 mm to
  spare at the depth where the print enters.
- **Not disturbing what is already there.** The photo hangs from one peg; a clumsy approach knocks it off, which
  scores 0.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: scene, the pegs' state, scoring, the instruction |
| `pegkit.py` | Builds the clothes pegs (an articulation each: two legs, a sprung pivot, soft rubber grips), the photo print and the fixed parts |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline tests of the scoring rule and the peg geometry (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The photo hangs from the wire by one peg. Peg its other top corner to the wire too." \
  --scorer task_score --max-steps 1200
```

Run limit: 1,200 steps (1 minute at 20 Hz). Second layout: set `PEG_LOT=B` in the environment (default `A`).

## Scripted demo

```bash
python demo.py task.py out_dir                  # layout A; PEG_LOT=B for layout B
```

1. The right hand comes straight down on a spare peg on the basket's rim, its fingers on the peg's soft grips, and
   squeezes (to a 5 mm finger gap): the jaws open and let go of the rim. It lifts the peg off.
2. It carries the peg, still squeezed open, to just short of the photo's near end above the wire, then over the
   corner, and down so that the open jaws pass either side of the wire and the photo's edge together.
3. It lets go: the jaws close on both. It rises clear.

The script is open loop: every pose comes from the layout constants in `task.py` (and one measured constant, where a
squeezed peg sits in the fingers), and object states are read only for the result file.

## Scoring

Tools: the trial video and a ruler.

- **0 for the whole trial** if at any time the photo is knocked off the wire (its top edge drops 1 cm or more below
  where it hung).
- Otherwise:
  - **1.0** once a second peg holds the photo's near top corner on the wire, let go and still, for 2 seconds: the
    peg's jaws closed on the wire, the peg within 2 cm of the photo's near end, the photo's top edge between its
    legs, both grippers more than 4 cm from it, and the photo still;
  - **0.5** if a peg is closed on the wire, let go and still (anywhere along it);
  - **0.25** if a peg has been taken off the basket (moved 2 cm from its place);
  - **0** otherwise.

In code: `_watch()` and `score()` in `task.py`.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Second peg on the near corner; full score at step 285 (14 s) |
| Scripted demo, layout B | 1.0 | Same script; step 283 |
| Dream-1, 1,200 steps | 0.0 | The right gripper went to the peg basket several times and hovered there; no peg was taken |
| Arms still | 0.0 | The photo hangs from its first peg; nothing moves |
| `pytest test_scoring.py` | 8 passed | Score tiers, the knocked-off gate, which peg positions count, the peg's opening, the layout |

## Runtime and seeds

About 2.5 minutes per demo run and 5 minutes per Dream-1 run on one L40S, including about a minute of start-up.
`object_jitter=0`; the layout is fixed per lot (A or B), and there is no randomness in the task.

## Fidelity

- Pegs, photo and wire are rigid. A peg is two 72 mm wooden legs (5 x 10 mm) on a pivot 30 mm from the jaw tips, with
  a torsion spring (0.25 N m/rad, preloaded so the closed jaws press at about 3.6 N), handles splayed 8 degrees, a
  groove for the wire, and rubber grip pads on the handles. Wood and paper do not deform.
- The wire is taut and modelled as a rigid 1.2 mm rod between fixed posts; a real wire sags a little under a photo.
- The print is a rigid 0.3 mm card. The pegs are 6 g, the print 2.8 g.

## Real-world build

A 1.2 mm stainless photo-display wire stretched taut between two posts 32 cm apart, 8.5 cm above the table, running
straight out from the arms. A 5 x 3.5 in (127 x 89 mm) photo print hung beside it by one soft-grip wooden clothes peg
at its far top corner, its top edge 1.2 cm above the wire. A blue plastic basket (12 cm deep, rim 9 cm tall) with two
more soft-grip pegs clipped on its near rim.

## Third-party assets

None: every object is built from primitive shapes in `pegkit.py` and `task.py`. The photo's picture is drawn from
flat shapes; no image file is used.
