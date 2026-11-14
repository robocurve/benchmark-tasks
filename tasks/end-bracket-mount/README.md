# Close off a terminal strip: fit and tighten an end bracket (two arms, loose DIN rail)

The step that finishes a terminal strip: an end bracket is clamped to the rail against the last terminal block so the
blocks cannot creep along the rail. A loose 24 cm TS35 x 7.5 DIN rail lies on the table with a row of four beige terminal
blocks mounted on it. A screw-fixed end bracket and a flat-blade screwdriver lie on the table. One arm must hold
the rail; the other puts the bracket on the rail (slides it on from the rail's free end, its hook under one lip), pushes it
flush against the last terminal block of the row and tightens its clamping screw with the screwdriver to 0.5 Nm.

All parts are our own simplified designs (no manufacturer CAD; see *How the parts are built*). The screwdriver is a
0.8 x 4.0 mm flat blade (a stubby, 85 mm long), the size for the bracket's M4 slotted screw. Tightening takes 1.6-1.8 turns
from backed out, so the hand must reset the screwdriver between wrist strokes (the YAM wrist turns +-120 degrees).

**Status:** the scripted two-arm demo reaches the full score (1.0) on CPU PhysX; the variants score as the ladder
says. Dream-1 (one trial through `inspect-robots run`, the command above) scores 0.0: see *Dream-1 run*.

## Files

| file | what it is |
| --- | --- |
| `task.py` | the yam_arena task file: scene (`OBJECTS`), `success`, `score`, `distance`; docstring = what is on the table |
| `eb_parts.py` | the custom parts (rail with the row of mounted blocks, the bracket with its screw mechanism, the screwdriver): geometry, USD writer, and the hook that lets yam_arena spawn them |
| `demo.py` | scripted two-arm demo, closed loop on the simulated object poses, and its variants |
| `bracket_bench.py` | test bench for the bracket's mechanism alone: turns the screw, pushes, pulls and loosens seated brackets on fixed rails with measured forces (the table under *Mechanism bench*) |
| `runs/dream1/` | the Dream-1 run: `model_run.mp4` (the three cameras in one video), one video per camera, `eval_log.json` (the inspect-robots log) |
| `runs/full_video/` | the full demo run: `demo.mp4` (the three cameras in one video), one video per camera, `setup.png` (the starting layout), `result.json` |
| `task.json` | submission metadata |

## Run and score a trial

Environment: yam_arena (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, PhysX at 240 Hz). Run at 20 Hz control with object
jitter off (one fixed layout, no seeds needed):

    inspect-robots run --embodiment yam_arena -E task=task.py -E control_hz=20 -E object_jitter=0 -E device=cpu \
        --policy <policy> --scorer task_score --max-steps 3600 \
        --instruction "Hold the DIN rail down with one arm. With the other, put the end bracket on the rail against the right end of the row of beige terminal blocks, push it flush, and tighten its screw with the screwdriver to 0.5 Nm."

**CPU physics (`-E device=cpu`) is required:** on GPU PhysX the sub-millimetre contact between the screwdriver's
0.8 mm blade and the screw's slot doesn't pass torque to the screw (the blade sits in the slot, but the screw never
turns). CPU PhysX handles it, and for this one-environment scene it is about 3x faster; the cameras still render on
the GPU.

`task_score` gives 1.0 when the episode ends in success, otherwise `score()` at the last step. Runtime per trial: about 30 s of simulator
start-up, then about 8 control steps per second on CPU PhysX with the three cameras rendering at 320 x 176 on the GPU
(the scripted demo: 2,247 steps in 205 s with 640 x 360 video); a full 3,600-step trial is about 8 minutes.

Scripted demo (drives the same embodiment and task-file scoring directly; CPU physics by default):

    python demo.py --variant full --out demo_runs/full      # result.json, steps.jsonl, one video per camera
    python demo.py --variant all --no-video                 # full, no_hold, no_screw, no_push in one session

Mechanism bench (about 4 minutes): `python bracket_bench.py [--device cpu]`, writes `bench_runs/bracket_bench.json`.

## Scoring

All measured in the rail's own frame, at the end of the episode:

| credit | condition |
| --- | --- |
| 1.0 | success: the bracket **flush** and **tight**, and still, held for 1 s in a row, with the rail in place |
| 0.75 | **flush**: seated, and the gap to the last beige block's face 0.5 mm or less at four points on the bracket's facing side (15 mm either side of its centre, 0.5 mm and 38 mm above the lips) |
| 0.5 | **seated**: on the rail to the right of the row, its seat on the lips at both lip edges (between 0.5 mm below and 1.0 mm above the lip tops), centred within 1 mm, square (yaw) and level (tilt) within 3 degrees |
| 0.25 | **lifted**: the bracket's lowest point 25 mm or more above the table |
| 0 | otherwise |

- **Tight:** in a real trial, the screw takes at least 0.5 Nm to turn further (the bracket's rated tightening
  torque in this task), checked with a torque screwdriver set to 0.5 Nm. In the simulation: the screw turned at least 1.6 turns
  from backed out (0.25 turn past the jaw's first contact at 1.37) with the jaw pressing on the far lip (2 N or more):
  the simulation's stand-in, see Fidelity.
- **Still:** under 10 mm/s and 0.5 rad/s, from the bracket's pose change over the last control step.
- **Rail in place:** the rail's origin moved less than 10 mm across the table and it turned less than 3 degrees from
  where it started. Otherwise success is impossible and the score is capped at 0.25.
- Measurable with a ruler, a feeler gauge, a torque screwdriver and a photo. Either face of the bracket may face the
  block, and it may sit either way round (hook on the near or the far lip).

No unintended strategy scores: set straight down on the rail, the hook lands on top of a lip and holds that side
2.8 mm up (not seated); a bracket left of the row or off the rail's end earns no rail credit; tightening a
bracket that is not flush scores only as seated; the beige blocks are part of the rail and cannot be pushed; moving
the rail caps the score.

## How the parts are built

yam_arena's task files only offer single primitives and library assets, and yam_arena has no extension point for
jointed parts. `eb_parts.py` is the task's own minimal mechanism; nothing in yam_arena is modified. A `Part`
subclasses yam_arena's `Box`; `task.py` calls `eb_parts.install()`, which wraps `yam_arena.scene.task_object` and
`object_states` when that module is imported, and spawns the parts from USD files it writes to a temporary folder.

**Geometry and licensing:** every part's geometry was created by us for this task and is written into `eb_parts.py`
as coordinates (a few dozen points per outline); no manufacturer CAD, meshes or other third-party assets are used.
The designs follow public standards: the TS35 x 7.5 rail profile (EN 60715) and the M4 slotted cheese-head screw
(DIN 84). The robot, table and cameras are yam_arena's own, unchanged.

- **Rail** (one free rigid body, 130 g with its blocks, steel friction 0.3): TS35 x 7.5, 1 mm web and walls, lips 2 mm thick with
  outer edges at +-17.5 mm. A row of four beige terminal blocks is part of the same body: our own simplified design of
  a generic 2.5 mm2 screw-clamp feed-through block (5.2 mm thick along the rail, 52 mm across, 40 mm tall; two wire-entry
  shoulders, two screws in wells on top with a marker groove between them, latch feet beside the lips; 7.5 g each,
  estimated).
- **End bracket** (an articulation of four bodies, 14 g, estimated for a cored polyamide moulding): our own design,
  8 x 56 x 46 mm, its section extruded through the 8 mm thickness; it collides as 14 convex pieces plus two guides and the hook.
  - *What fixes its shape:* the seat on the lip tops (0.2 mm wider than the lips each side), skirts straddling the
    lips down to 0.5 mm above the table with chamfered inner faces, and a 10 mm wide screw recess in the middle of the
    top, its floor 32 mm above the seat. The rest (straight sides with a waist, chamfered top corners) is plain styling.
  - *The mechanism* (our design; a common principle for screw-fixed end brackets): a fixed hook under one lip (1.5 mm
    reach, 0.8 mm play); on the other side a steel jaw driven by the screw through a wedge (1.3 mm along a 20 degree
    line per turn: inward and up under the lip); guide faces 0.2 mm outside both lip edges.
  - Screw and jaw are coupled by a PhysX mimic joint (rotation drives the jaw; no scripted motion). The jaw sits on a
    100 N/mm spring (the housing's and wedge's compliance). The screw is an M4 slotted cheese head (DIN 84: 7 mm head,
    1.2 mm slot).
  - The screw turns against a constant 0.03 Nm friction (thread friction in the plastic), which also holds it once
    tightened, and stops it as soon as the screwdriver lets go (static and while turning). Tight = turned 1.6 turns
    or more with the jaw pressing on the lip.
- **Screwdriver** (free rigid body, 30 g): a stubby, 0.8 x 4.0 mm blade, 4 mm shaft 25 mm long, 26 x 50 mm handle,
  85 mm overall (friction 0.9,
  a two-component grip).

### Mechanism bench

The screw is turned by a torque-limited "virtual screwdriver" that presses 10 N into the slot, as a hand does:

| measurement | result |
| --- | --- |
| free running torque | 0.03 Nm |
| jaw touches the lip | 1.37 turns from backed out |
| tight (1.6 turns, jaw pressing) | 1.61 turns, jaw force 17 N, the bracket level within 0.11 degrees |
| tight screw left alone | stays (friction 0.03 Nm; the jaw pushes back with 0.003 Nm) |
| loose bracket pushed along the rail | slides at 0.1 N |
| tightened bracket pushed along the rail | slides at 1.9 N (loose: 0.1 N); see Known weak points |
| tightened bracket pulled straight up at 60 N | stays on (moved 0.4 mm) |
| screw turned back out one turn | the jaw releases fully |

## The demo

Left arm holds the rail, leaned 30 degrees along it with its wrist away from the middle (upright, the two arms' wrist
housings meet above the work). Right arm picks the standing bracket up from above, pinched across its 8 mm
thickness, lines it up with the rail's free right end (0.4 mm above the lips, its hook under the near lip, aligned
on its measured pose), slides it on and along the rail, correcting its position, yaw and roll as it goes, to 15 mm
short of the row, lets go, and pushes it flush with its closed fingers, the hand leaned back 25 degrees so only
the pad tips touch. Measured on GPU: lined up within 0.07 mm, flush with 0.0 mm gaps, square and level, rail moved
under 1 mm.

Right arm then picks the screwdriver up across its handle and holds it upright over the screw; the left arm takes it
from above (coaxially, which is what lets the wrist turn it) while the right arm moves to hold the rail. The left arm
centres the blade over the slot on the measured blade tip (settling between corrections: the YAM wrist is soft and
lags its command by 1-2 mm), lowers it in, and tightens in wrist strokes, pressing down; between strokes it lifts the
blade out, turns back in the air and re-engages, choosing the blade's way round that gives the longest next stroke.
Meanwhile the right arm holds the rail down right behind the bracket, its pads a stop that keeps the bracket square
against the block (the screw's friction twists a loose bracket within its guides' play otherwise). When the bracket
is tight the left arm lifts the screwdriver out and lays it on the table. Measured (CPU PhysX, 20 Hz control, jitter 0): **success at step 2,187** of 3,600 (score 1.0); 4 strokes, no
slips, 1.75 turns, jaw force 15 N; the bracket flush (gaps 0.05 mm or less), square (yaw 0.0 degrees) and level (0.35
degrees); the rail moved 1.7 mm and turned 0.2 degrees. The video runs 3 s past success (2,247 steps).

Why these choices (found on the GPU): the YAM finger pads are 27 mm wide, and the only flat parallel faces of the
bracket that wide are its two big faces, so it stands on its feet and is pinched across its thickness; tilting it on
about its hook doesn't fit rigid-body physics (its skirts reach to 0.5 mm above the table), so it slides on from
the rail's free end, the other way an electrician fits one; pointing straight down over the rail the arms reach only
12-16 cm above the table, so the screwdriver is a stubby (a standard 100 mm one puts the hand at 21-23 cm).

Variants proving the ladder (CPU PhysX):

| variant | what the arms do | score | notes |
| --- | --- | --- | --- |
| `full` | as above | **1.0** (success) | success at step 2,187 |
| `no_hold` | the rail is never held | **0.0** | sliding the bracket onto the rail's end shoves the loose rail 105 mm and turns it 23 degrees; the run stops 1 s after the rail leaves its limits, with the bracket still in the hand at the rail's end (neither seated nor lifted 25 mm). A moved rail caps any score at 0.25 (in an earlier version of the scene the bracket ended up seated on the moved rail: 0.25, capped) |
| `no_screw` | slid on and pushed flush, never tightened | **0.75** | flush, gaps 0.06 mm |
| `no_push` | slid on, stopped 20 mm short of the row, tightened there | **0.5** | seated and tight (1.77 turns, jaw force 11 N), not flush (gaps 19-20 mm) |

## Dream-1 run

One trial with the command under *Run and score a trial* (`--policy dream`, CPU PhysX, cameras 320 x 176,
3,600 steps, 244 s): **task_score 0.0** (hit the step limit). Neither arm holds the rail (it stays in place); the right
arm reaches again and again toward the screwdriver and the bracket and nudges them on the table, but lifts nothing
onto the rail. Videos and log in `runs/dream1/`.

## Fidelity: what is simplified

- Rail lips 2 mm thick (1 mm on a real rail) so contacts don't tunnel at 240 Hz; the bracket's hook and jaw sit
  under the thickened lips.
- The bracket and blocks are simplified designs of our own (see *Geometry and licensing*), not copies of a
  particular product; the bracket's mechanism (hook, wedge jaw, guides) is our design.
- The screw's thread is a mimic-joint coupling and its friction a constant 0.03 Nm. Tightness is judged by the screw's turns
  past the jaw's contact (0.25 turn, with the jaw pressing), not torque: a friction that rises with the clamp would have to be rewritten to PhysX while the
  screw turns, and each rewrite kicks a turning screw (the screwdriver jumps out of the slot). So the simulated
  screwdriver needs far less torque than 0.5 Nm, and a real M4 screw at 0.5 Nm clamps with several hundred newtons.
- The slot's top edges are chamfered 0.3 mm (it lets a blade within about 0.5 mm drop in, for PhysX's ~0.3 mm contact
  error). The head collides as two round half-discs with a thin chamfered cap on top; all of it stays within 3.8 mm
  of the screw's axis, inside the bracket's 8 mm thickness, so the turning head never touches the terminal block.
- The screw head does not sink as it turns; the screwdriver's blade tip is a flat box.
- The four beige blocks are rigidly part of the rail (a real latched row could be pushed along the rail; here the
  row can't move). No end plate, no end bracket at the row's other end.
- Friction: rail steel 0.3, bracket polyamide 0.35, screw slot 0.2, screwdriver handle 0.9, table and robot fingers
  PhysX default 0.5. Contact offsets 0.5 mm on the parts, rest offset 0.

## Known weak points

- CPU physics is required (`-E device=cpu`): on GPU PhysX the blade-in-slot contact doesn't turn the screw.
- A tightened bracket resists only ~2 N along the rail (the jaw's force creeps in PhysX; a real screw-fixed end bracket
  locks the terminal row). The scoring doesn't depend on it (it checks the screw's turns with the jaw pressing), but a policy
  that bumps the bracket after tightening can push it out of flush.
- The scripted demo's screwdriver handling isn't fully robust: the blade sometimes slips out of the slot and the hand
  re-engages (each costs ~10 s); runs vary a little from one to the next.
- The scripted demo runs through `demo.py` (the same `YamArenaEmbodiment` and task-file scoring as `inspect-robots run`),
  not the CLI: `yam_scripted` would need a `demo()` in the task file. The CLI path itself is tested by the Dream-1 run.
- `eb_parts.install()` wraps two functions of `yam_arena.scene`; a yam_arena update that changes them stops the task
  with an error. It must run before `yam_arena.scene` is first imported (true for `-E task=task.py`).
- The scorer cannot tell whether a hand still holds the bracket; a still, flush, tight bracket held by a hand counts.
  The demo lets go before the hold time and checks the bracket stays flush afterwards.
