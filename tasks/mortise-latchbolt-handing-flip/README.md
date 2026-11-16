# Reverse the handing of a mortise lock's latchbolt so the jamb's strike latches on it

A grade-1 commercial mortise lock case (`113 x 154 x 25 mm`) stands on its back edge in a padded bench holder at
`(32.0, -15.0) cm` from the midpoint between the arm bases (`x` forward, `y` left), faceplate up. Its stainless
armored front (`203 x 32 x 3.2 mm`) has a `29.5 x 17.5 mm` latchbolt opening. The spring-loaded latchbolt head
(`28 x 16 mm`, brass with a satin chrome finish, anti-friction tongue in the bevel) stands out `19 mm` at full throw
with its bevel facing `+x`; it turns only when pulled out far enough that its tail clears the square opening.

Above it runs the strike side: an oak jamb section (`70 x 260 x 25 mm`) with a stainless strike (`124 x 32 mm` plate,
`20 x 32 mm` opening into a dark strike box, lip ahead of the opening), face down `3 mm` above the faceplate (the
door-edge clearance), on a linear guide along `x` that stands in for the door's swing over the last `9 cm` at the
latch. It starts open (`9 cm` toward `-x`); a light spring (`~1 N`, the door seal's push) returns it there. Closing it
(`+x`) brings the strike lip onto the latchbolt's `-x` face. A push block stands on its `-x` end over the guide.
Two visual-only hand tools (end-cutting pliers, claw hammer; no collider) lie flat on the bench.

This is why handing matters. As delivered, the latch's square back faces the strike: the lip stops on it (the jamb
jams `42 mm` short of its stop) and the door cannot latch. Reversed, the bevel faces the lip: closing the jamb cams the
latch in, and at the stop it springs out into the strike opening, where its square back holds the jamb against the
return spring.

Goal (the maker's field change, L/LV9000 installation instructions 108367 p.1: "If necessary to change handing:
1. Pull out the latch and rotate 180°"): the latchbolt reversed (bevel `-x`) and the jamb closed and held by it, hands
off. The instruction states only this end state.

## Scoring

End state only (`score()` reads the latchbolt's turn and projection, the jamb's position and the grippers' positions
at the step being scored; nothing earlier in the run counts):

| Score | Condition at the end | Check on a real bench |
|---|---|---|
| `1.0` | bevel faces `-x` (turned `180 +- 10 deg` from the start); jamb within `3 mm` of its stop with the latchbolt standing `19 +- 1 mm` out into the strike opening (latched; with the jamb closed it can stand there only inside the opening); each gripper's tool-centre point outside the box made of the faceplate and the jamb over its whole travel (`x 19.4..35.4`, `y -25.2..+3.5 cm`) grown by `5 cm` on every side, from the bench up to `22.6 cm`; the run ends as a success once this has held `1 s` | protractor on the bevel, ruler at the jamb stop, photo |
| `0.7` | reversed and latched as for `1.0`, but a gripper's tool-centre point is inside that box | photo |
| `0.4` | reversed, but not latched: jamb open, sprung back or short of its stop, or the latchbolt held in | protractor, ruler |
| `0` | not reversed (untouched, pulled and let go, left at `90 deg`, turned back), whatever the jamb does | protractor |

Possible: `0, 0.4, 0.7, 1.0` (the same text is task.json `scoring`). Why the hands-off term: a gripper on the jamb
would hide whether the latch alone holds it against the return spring.
Shortcuts, measured with the MuJoCo pre-check on this code (`mj_check` SHORTCUTS and a control run):
doing nothing `0.0`; sweeping the arms across `0.0`; stopping at half the demo `0.4` (turned, jamb still open);
finishing and then replaying the arm path back (undo) `0.7` (the replayed push cannot pull a latched jamb open; the
hand ends on the jamb); closing the jamb without reversing the latch `0.0` (the lip stops on the square back at
`-42 mm`, and the jamb springs back to `-90 mm` when let go).

## Sub-action evidence

The scripted demo (`demo()`, open loop): the right arm pinches the head's end faces, pulls it `13 mm` past the throw,
waits until the tail is `6.5 mm` clear of the opening, turns it in two `90 deg` wrist strokes with the head resting on
the faceplate in between, lets it seat and parks; then the left arm, fingers closed, pushes the jamb's back end `9 cm`
onto the latch (the lip cams the reversed bevel in; the latch springs into the strike opening at the stop), backs off
and parks. `_watch` prints `SUBACTION_END {...}` on every step of the `1 s` success hold. Last line of each:

Isaac column: official `1280 x 704` demo run of this code, `task_score: 1`, success hold ending at step `443`.
`STATE` lines (observe-only, every `2 s`) log the jamb, bolt and turn through the run.

| sub-action | quantity | target | MuJoCo (current code) | Isaac 1280 x 704 (current code) |
|---|---|---|---|---|
| pull to the handing position | max `bolt` during the run | `>= 0.007 m` | `0.013121 m` | `0.012652 m` |
| turn 180 deg | `turn` | `180 +- 10 deg` | `180.0 deg` | `180.0 deg` |
| seat at the rated throw | `bolt` (0 = 19 mm projection) | `0 +- 0.001 m` | `-0.000006 m` | `0.000103 m` |
| close the jamb onto the strike | `jamb.close` (0 = at the stop) | `>= -0.003 m` | `-0.001003 m` | `-0.001 m` |

Visible end state: in the official top camera at `1280 x 704` the jamb is about `220 x 50 px` and its `9 cm` travel
moves it about `67 px`, so open, jammed (`-42 mm`) and closed read at a glance; the closed jamb sits over the latch,
and an unlatched jamb is visibly back at its open end. The flip itself shows in the head turning between its bright flat back and its bevel with the dark tongue slot.

## Run a trial

From this folder, on a machine with the official simulator (`yam-isaac-lab-arena` `setup.sh`, or the Robocurve Brev launchable):

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/mortise_latchbolt_handing_flip.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mortise-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/mortise_latchbolt_handing_flip.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mortise-demo

for log in ~/yam-eval/mortise-*/*.json; do inspect-robots video "$log"; done
```

Runtime: the demo ends as a success after `445` steps in the MuJoCo pre-check and `444` in Isaac (`22.2 s` at `20 Hz`); Dream-1 runs the
full `2,400` steps. Seeds: none (the layout is fixed and `object_jitter=0`, no loose objects; runs are deterministic up
to the simulator). Offline tests: `pytest test_mortise_latchbolt_handing_flip.py` (official runtime on `PYTHONPATH`). `rehearsal.py` and `screen.py` are
builder pre-checks that need a separate MuJoCo checking tool (not included); they are not needed to run the task.
Media (setup image, demo and Dream-1 videos) are attached by the official upload tool at submission, which writes the
media ID into task.json.

## Fidelity

- Case, armored front, throw: `1:1` from the maker's documents. Head size (`28 x 16 mm`), the `6.5 mm` key
  engagement (head length inside the opening at full throw) and the spring rate (`0.2 N/mm`: `3.8 N` flush, `2.6 N`
  at the handing pull) are estimates (not published).
- Strike: the standard `4-7/8 x 1-1/4 in` mortise-lock strike (ANSI/BHMA A115.1 frame preparation) (`124 x 32 mm`) strike outline; its opening (`20 x 32 mm`), the lip and the
  strike box depth are estimates. The jamb runs on a linear guide instead of hinges: over the last `9 cm` at the latch
  a door's arc is nearly straight (on a `0.9 m` door, `9 cm` of arc moves the latch edge `4.5 mm` sideways). Its
  `~1 N` return spring stands in for the door seal's push (estimate).
- Latch bevel: one flat bevel from `2 mm` above the faceplate to a `3.5 mm` land (real bevels are slightly curved).
- One linear spring returns the latch from both sides of full throw (real locks preload the latch against a stop).
- The case and holder are fixed; the holder is a shop-made padded fixture (no product).
- Step 2 of the maker's procedure (moving the handing screw, which sets which lever locks) applies to locking
  functions and is out of scope.
- Guide friction `0.3` (lubricated brass in steel).

## Assets and provenance

| object | source | revision / sha256 | licence |
|---|---|---|---|
| lock case, armored front, latchbolt, holder, jamb with strike, linear guide (`assets/visual/*.usdc`) | builder-made true-to-size model, `assets_build/build_visuals.py` (Blender 5.x); dimensions from the maker's installation instructions 108367 (https://us.allegion.com/content/dam/allegion-us-2/web-files/schlage/installation-documents/Schlage_L_LV9000-Series_Lever_and_Rose_Installation_Instructions_108367.PDF, sha256 `d97fdd7783f3e09b...`) and L-Series catalogue 102899 (https://www.andersonlock.com/wp-content/uploads/2018/05/Schlage_L_Series_Catalog_102899.pdf, sha256 `79309e38f04e0138...`, p.9 features, p.31 armored fronts); reference photos: catalogue p.9 lock-case cutaway and p.31 armored fronts | `lockcase.usdc` `19fb092e...`, `latch_head.usdc` `2992cbfa...` | task licence |
| end-cutting pliers (`assets/props/pliers/`) | NVIDIA SimReady `tool_pliers_a01` (rev `7879e0e5`) | `8d2e0915...` | CC-BY-4.0 |
| claw hammer (`assets/props/hammer/`) | NVIDIA SimReady `tool_hammer_a01` (rev `7879e0e5`) | `4e0a0151...` | CC-BY-4.0 |
| prop layout (`assets/props/bench_props.usda`) | builder-made placement file | -- | task licence |
| close-up still (`closeup-head-start-end.png`) | cropped from the `1280 x 704` demo run of this task | -- | task licence |

Asset search (public 3D libraries first): no library model of a mortise lock, door latch or lock case was found,
so the lock is a builder-made true-to-size model. The hand tools are NVIDIA SimReady models; tools whose geometry
carries an engraved or embossed maker mark were left out. The library's printed label and engraving texels on the
hammer and pliers textures (albedo, normal, ORM) are blanked by `assets_build/blank_marks.py`.

No maker names, word marks or part numbers appear on any model in the scene (the source documents are cited above
for provenance only).
