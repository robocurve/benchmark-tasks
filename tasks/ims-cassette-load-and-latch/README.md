# Fix an instrument misloaded across a sterilization cassette, seat both loose ones, close the lid and hook the latch

A stainless 5-instrument exam cassette at real size (`76 x 203 x 32 mm` closed) sits in a low bench holder at
`(30.0, 0.0) cm` from the midpoint between the arm bases (`x` forward, `y` left), long side toward the robot. Its two
blue silicone rails, `12 cm` apart (the `13 cm` handles reach `5 mm` past each rail's centre), run across the `76 mm` width; each has five notches at `12.7 mm` pitch, so each
instrument lies along the `203 mm` length, down in one notch of each rail. A sprung silicone lip at each notch mouth
narrows it to `8.0 mm`: a `9.5 mm` handle set on the mouth rests on the lips until it is pressed past them, and is then
held under them. The lid (modelled solid; the real lid is perforated), hinged on the back long edge, leans open at `120 deg`; a blue press pad under
it clears a seated handle by `0.8 mm`. The red latch clip pivots on the lid's front top edge and leans back over the lid
(open, over centre: gravity holds it there); swung down, its hook tab catches under a steel keeper lip on the front
wall. Three loose instruments are already snapped into slots 1, 3 and 5, held only by the rail lips; slots 2
and 4 are free. The misload: an explorer (`16.5 cm`, `9.5 mm` round handle) lies across the cassette, handle along `x`,
centre `(27.0, 4.7) cm`, on the front rim and over the seated handles, so the lid cannot come down. A mouth mirror lies
on a tray in front of the cassette, in its cradle at about `(23.4-24.0, 0.0) cm` (in Isaac it settles against the
cradle ridge at `23.4`), handle along `y`.

Why it matters (maker's catalog, troubleshooting table p.55): "overloading / improperly placed instruments" cause
broken hinges, "instruments should not protrude from cassette", "only light force is needed to close the cassette".
The trial starts from that error. The robot has to find the instrument that is wrong, lift it out, turn it a quarter
turn and push it past both rail lips into a free slot, load the mirror the same way, and only then close the lid and
push the clip forward over centre so its hook drops under the keeper lip. Left across the cassette, the explorer holds
the lid about `1 cm` open and the clip cannot hook.

Who does it (BLS employment figures as shown on O*NET OnLine; O*NET task statements, both rated Core, incumbent data 08/2021):
- dental assistants, O*NET 31-9091.00, 388,600 US workers (2025), task 550: 'Prepare patient, sterilize or disinfect instruments,
  set up instrument trays, prepare materials, or assist dentist during dental procedures.'
- dental hygienists, O*NET 29-1292.00, 225,300 US workers (2025), task 18538: 'Maintain dental equipment and sharpen and sterilize
  dental instruments.'

Together 388,600 + 225,300 = 613,900; neither alone reaches 500,000. Both occupations list instrument sterilization as a
Core task, not a supplemental one. In a cassette system the used instruments go back into the cassette's rails after each
patient and the closed, locked cassette goes through the washer and autoclave, so sterilizing the instruments includes
loading the cassette and latching it, the step this task covers.

How this differs from nearby tasks:
- #977 (accepted, forceps into a sterilizer basket): there is no misload to find, no retention, no lid and no latch. Here an instrument lies across the cassette and holds the lid about 1 cm open, so the robot must lift it, turn it a quarter turn, snap it past the sprung rail lips into a free slot, seat a second loose one, and only then can the lid close flat; the latch is a single final tier.
- #1734 (final): fastening a draw-latched case. Here no draw latch or secondary catch is scored; the scored core is correcting the misload and the snap seating, and the occupations are dental assistants and dental hygienists (O*NET 31-9091.00, 29-1292.00 task 18538), not packers or material movers.
- #1612 was closed by its author the same minute #1734 was submitted and has no ruling.

Goal (instruction states only this): all five instruments snapped down, each into a slot of its own, lid closed flat, latch clip
hooked, grippers off the cassette.

## Scoring

End state only, readable after the run with a photo, a 1 mm feeler gauge and a ruler (open the lid after the run to
check the handles). `score()` reads the instruments' poses, the lid and clip angles and the grippers' positions at the
step being scored; nothing earlier in the run counts.

| Score | Condition at the end | Check on a real bench |
|---|---|---|
| `+0.2` each | the mirror / the explorer seated in a free slot: handle along the slot, centre within `3 mm` of the slot line, within `8 mm` of the middle along the slot (both handle ends at least `1 mm` on their rails) and no more than `1 mm` above its seated height (reached only past the rail lips); two in one slot count once | after the run: open the lid, each handle by eye and ruler below its lip tops |
| `+0.3` | lid closed flat: front gap `<= 1 mm` (counts only if all five instruments are seated) | 1 mm feeler gauge |
| `+0.3` | latch clip swung down `>= 88 deg` with its hook tab under the keeper lip (tab top edge overlapping the lip by `>= 0.5 mm`, at most `2 mm` below it) on a flat lid, both grippers off: each tool-centre point outside the cassette + tray outline (`x 19.0..35.8`, `y -10.2..+10.2 cm`) grown by `5 cm` on every side, up to `14.9 cm` above the bench (counts only if all five are seated) | final photo: clip hanging over the front, hook under the keeper lip; grippers clear |

Possible: `0, 0.2, 0.4, 0.7, 1.0` (same text as task.json `scoring`). The run ends as a success after `1 s` at `1.0`.
Start jitter in the MuJoCo pre-check (`+-5 mm`, `+-3 deg`): pre-seated instruments start in their notches and move with
the cassette under start jitter (the cassette is fixed in its holder, so they keep their seats; each is held down under its two sprung rail lips, which a
handle only passes when pressed past them -- the same lips the robot presses the explorer and the mirror past); the misloaded explorer
and the mirror each get their own jitter. Robustness: 5 jitter runs and friction `x0.7` / `x1.3` all score `1.0`.
Shortcuts and controls, measured with the MuJoCo pre-check on this code (`mj_check` SHORTCUTS and control runs):
doing nothing `0.0`; sweeping the arms across `0.0`; lift-and-drop `0.0`; stopping at half the demo `0.2` (explorer seated,
mirror not yet); replaying the arm path back after the end (undo) `0.7`; the explorer left across the cassette and the
mirror loaded, then the lid closed and the clip pushed `0.2` (lid stays `49 deg` open on the explorer, so the lid and
latch points never count); the explorer left across and nothing else done `0.0`; the clip
never pushed over centre `0.7` (lid flat, clip still leaning back at `125 deg`).
Latch pull test (diagnostic run, `IMS_PULL_TEST=1`, MuJoCo): `2 N` upward on the lid -- unhooked (a `0.2 s` pulse, clip
still up) the lid lifts `10.1 deg`; hooked (held `1 s` after the clip is down) it lifts at most `0.76 deg` (the
`0.9 mm` gap under the keeper lip) and the score stays `1.0`. Isaac (same test, 1280 x 704 diagnostic run): unhooked `10.96 deg`, hooked at most `0.67 deg`, score `1.0`.

## Sub-action evidence

The scripted demo (`demo()`, open loop, both arms): the left arm closes its jaws across the misloaded explorer (wrist
`0 deg`), lifts it out of the cassette, turns it a quarter turn in the air (wrist `-90 deg`) so it lies along slot 4,
sets it on slot 4's notch mouths and lets go -- it rests on the rail lips -- then presses it down at its middle with
closed finger tips past the lips until it bottoms; it then presses each of the three pre-seated handles (slots 5, 3, 1)
home the same way, since lifting the explorer off them can leave one riding on its lips. The right arm then picks the mirror off the tray (jaws across the
handle) and seats it in slot 2 the same way. The left arm then
comes round the cassette's far end and pushes the open lid from behind past upright; it swings shut on its own weight
onto its pads. The right arm pushes the clip forward past upright, moves out of its way while it swings down, and
presses it home so its hook sits under the keeper lip. Both park.
`_watch` prints `SUBACTION_END {...}` on every step of the `1 s` success hold and an observe-only `STATE` line every
`2 s`. Last line of each:

| sub-action | quantity | target | MuJoCo (current code) | Isaac official run, 320 x 176 (current code) |
|---|---|---|---|---|
| seat the mirror (inst2) | slot it is seated in | `2` (any free slot) | `2` | `2` |
| seat the explorer (inst4) | slot it is seated in | `4` (any free slot) | `4` | `4` |
| close the lid flat | `lid` angle | `<= 0.75 deg` | `-0.04 deg` | `-0.0 deg` |
| hook the latch clip | `clip` angle (and hook under the keeper) | `<= -88 deg` | `-89.67 deg` (hooked) | `-89.72 deg` (hooked) |

Visible start and end state: in the official top camera at `1280 x 704` the misloaded explorer shows at the start as a
bright line about `100 px` long lying across the cassette's rails and over its front edge, at right angles to the seated
handles. The closed cassette is about `160 x 55 px`; the open lid (blue pads) and the five handles in their notches read at a glance before closing, and the closed lid shows as one flat grey top. The locked clip hangs down the front face, so from the top it is only a small red mark (about `4 x 3 px`) at the middle of the front edge; the side and wrist cameras show it swing down.

## Run a trial

From this folder, on a machine with the official simulator (`yam-isaac-lab-arena` `setup.sh`, or the Robocurve Brev launchable):

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/ims_cassette_load_and_latch.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cassette-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/ims_cassette_load_and_latch.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cassette-demo

for log in ~/yam-eval/cassette-*/*.json; do inspect-robots video "$log"; done
```

Runtime: the demo ends as a success after `987` steps in the MuJoCo pre-check and `987` in Isaac (`49.3 s` at `20 Hz`; `270 s` wall clock on one A10 including start-up). Dream-1 runs the full `2,400` steps. Seeds: none (fixed layout, `object_jitter=0`; runs are
deterministic up to the simulator). Offline tests: `pytest test_ims_cassette_load_and_latch.py` (official runtime on
`PYTHONPATH`). `screen.py` is a builder reach pre-check; it is not needed to run the task. Media (setup image, demo and
Dream-1 videos) are attached by the official upload tool at submission, which writes the media ID into task.json.

## Fidelity

- Cassette size (`76 x 203 x 32 mm`), the rail layout (two rails across the width, instruments along the length), the
  lid hinged on a long edge and the pivoting clip at the middle of the front edge: from the maker's catalog (p.5-6).
- Estimates (not published): `9.5 mm` handles, `12.7 mm` notch pitch, rails `12 cm` apart, pad height, clip size
  (`23 x 12 mm`), lip stiffness, instrument mass (`20 g`), lid mass (`120 g`).
- Rail grip: one sprung silicone lip per rail at every notch (`3 kN/m`, `2.5 mm` into the mouth,
  estimate) stands for the silicone rail's grip; all five instruments are loose bodies held only by those lips. The clip is over
  centre: open, gravity holds it leaning back on its stop; locked, its hook tab hangs `0.9 mm` under the keeper lip,
  so the lid's front edge cannot lift more than that (2 N pull test above). The keeper lip is its own fixed body: the
  clip is a link of the cassette articulation, and links of one articulation do not collide with each other.
  The cassette sits in a bench holder (in practice it lies on the counter).
- Contact friction `0.5` (stainless on silicone and on stainless).

## Assets and provenance

| object | source | licence |
|---|---|---|
| cassette, lid, clip, holder, tray, instruments (`assets/visual/*.usdc`) | builder-made true-to-size models, `assets_build/build_visuals.py` (Blender 5.x); cassette dimensions and layout from the maker's IMS catalog 2020 (https://www.hufriedygroup.com/sites/default/files/2020-07/IMS-Catalog-IMS-120-2006-v1.pdf, sha256 `e33652d9e0298925...`, p.5 latch, p.6 exam cassette 5 instruments, p.55 troubleshooting) | task licence |

Asset search (public 3D libraries first, our index of NVIDIA SimReady, Objaverse and Isaac Sim assets): no model of an
instrument cassette with rails; the closest instrument is a SimReady mouth mirror (`instrument_mirror_a01`, 186 mm, thinner
handle than the 9.5 mm handle the notch geometry is built around), and the other matches (forceps, scissors, scalpel
handle) are not exam instruments that lie in cassette rails. So all objects are builder-made true-to-size models. No maker names, word marks
or part numbers appear on any model (the catalog is cited for provenance only).
