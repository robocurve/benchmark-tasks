# Fix a DIN rail to a mounting plate with two loose screws (two arms, hex screwdriver)

Every control cabinet starts this way: the mounting plate lies flat on the bench, the DIN rails are laid on it and
screwed down before any terminal block goes on. A galvanised steel mounting plate with two tapped M5 holes 200 mm apart
lies fixed on the table; a loose slotted TS35 rail lies on it, turned a little and with no slot over either hole. Two
M5 socket head cap screws and a hex screwdriver stand in a screw holder beside it. The arms must line the rail up so
a slot is over each hole, hold it there, put a screw through a slot into each hole, start it in its thread, and
tighten both with the screwdriver.

New skills compared with tasks that turn a captive screw: picking a small loose screw (M5, 8.5 mm head) and placing it
upright through a 6.2 mm slot onto a hidden hole; **starting** it in its thread (it only catches when upright and
turned clockwise); lining up a part so that two fastening points match; and two screw-ins in one run.

All parts are our own simplified designs (no manufacturer CAD; see *How the parts are built*).

**Status:** the scripted two-arm demo reaches the full score (1.0, success at step 8,441 of 13,000); the variants
score as the ladder says. Dream-1 (one trial through `inspect-robots run`, the command below) scores 0.0: see
*Dream-1 run*.

## Files

| file | what it is |
| --- | --- |
| `task.py` | the yam_arena task file: scene (`OBJECTS`), `success`, `score`, `distance`; docstring = what is on the table |
| `rp_parts.py` | the custom parts (plate, rail, screws with their thread mechanism, screw holder, screwdriver): geometry, USD writer, and the hook that lets yam_arena spawn them |
| `demo.py` | scripted two-arm demo, closed loop on the simulated object poses, and its variants |
| `rp_bench.py` | test bench for the thread mechanism alone (the table under *Thread bench*) |
| `runs/dream1/` | the Dream-1 run: `eval_log.json.gz` (the inspect-robots log, gzipped); its video is the task's uploaded model-run video |
| `runs/full_video/` | the full demo run: `result.json`, `setup.png` (the starting layout); its video (the three cameras in one) is the task's uploaded demo video |
| `task.json` | submission metadata |

## Run and score a trial

Environment: yam_arena (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, PhysX at 240 Hz). Run at 20 Hz control with object
jitter off (one fixed layout, no seeds needed):

    inspect-robots run --embodiment yam_arena -E task=task.py -E control_hz=20 -E object_jitter=0 -E device=cpu \
        --policy <policy> --scorer task_score --max-steps 13000 \
        --instruction "Fix the DIN rail to the mounting plate with the two screws: line the rail up so a slot is over each hole, hold it, put a screw through a slot into each hole and tighten both with the hex screwdriver."

CPU physics (`-E device=cpu`) is what the task was built and tested on (the screwdriver-in-socket and screw-in-hole
contacts are sub-millimetre; for this one-environment scene CPU PhysX is also faster). The cameras render on the GPU.

`task_score` gives 1.0 when the episode ends in success, otherwise `score()` at the last step. Runtime per trial:
about 30 s of simulator start-up, then about 14 control steps per second on CPU PhysX with the three cameras rendering
(the scripted demo: 8,501 steps in 583 s with 640 x 360 video); a full 13,000-step trial is about 15 minutes.

Scripted demo (drives the same embodiment and task-file scoring directly; CPU physics by default):

    python demo.py --variant full --out demo_runs/full      # result.json, steps.jsonl, one video per camera
    python demo.py --variant one_tight                      # likewise one_tight, no_tighten, one_started

Thread bench (a few minutes): `python rp_bench.py --device cpu`, writes `bench_runs/rp_bench.json`.

## Scoring

Each screw is scored on its own, in either order; the task score is the sum:

| credit per screw | condition |
| --- | --- |
| 0.5 | **tight**: started (below) and tightened: its head down on the rail and turned a quarter turn further |
| 0.25 | **started**: in its hole **through a slot of the rail**, at least 1 full turn into its thread |
| 0 | otherwise |

So 1.0 = both screws tight through the rail; any order climbs 0.25, 0.5, 0.75, 1.0. **Success** (ends the episode
with 1.0): both screws tight and both still for 1 s.

- **Through a slot:** the screw's axis inside one of the rail's 6.2 x 15 mm slots (measured in the rail's frame).
  Both screws need the rail **lying flat on the plate**: its underside within 1 mm of the plate and tilted less than
  2 degrees; otherwise neither screw scores.
- **Started:** in a real trial, the screw stands in its hole by itself and has turned at least one full turn since it
  caught (mark the head; or: it doesn't lift out). In the simulation: the thread has caught and the screw turned 1
  turn or more since.
- **Tight:** in a real trial, the screw doesn't turn further at 2 Nm (a torque screwdriver set to 2 Nm). In the
  simulation, the stand-in: the head has touched the rail (the clamp under it carried 2 N or more for 0.5 s in a row)
  and the screw has turned 0.25 turn or more since, with the head still pressing with 5 N or more. See Fidelity.
- **Still:** under 10 mm/s and 0.5 rad/s.
- Measurable with a ruler, a torque screwdriver, a marker and a photo.

No unintended strategy scores: a screw driven into a hole without the rail over it counts nothing; a screw that
doesn't go through a slot can't reach a hole (the rail's web covers it); a screw only pushed into a hole, or turned
while tilted more than 3 degrees, doesn't catch its thread; a rail lifted, standing on edge or upside down scores
nothing; the plate is fixed. If the first screw is tightened with the rail askew so the second hole is no longer under
a slot, the run stays at 0.5 until the screw is loosened and the rail lined up again (as in real work).

## Setup

All positions from the arm bases' midline (x forward, y to the left); the table top is the reference height.

- **Mounting plate** (fixed): galvanised steel, 300 x 200 x 2 mm on 15 mm folded edges, lengthwise pointing away
  from the arms, centred 0.25 m forward on the midline. Two tapped M5 holes on its centreline 200 mm apart (0.15 m
  and 0.35 m forward), each with a 90 degree countersink to 5.5 mm.
- **DIN rail** (loose, 95 g): slotted steel TS35 x 7.5, 250 mm long, slots 6.2 x 15 mm every 25 mm along the web's
  centreline (the first 12.5 mm from the end). Lies web-down along the plate, centred on it and turned 3 degrees: both
  holes under the web between slots. It lines up with its slots over the holes when shifted 12.5 mm (or 37.5 mm) along.
- **Screw holder** (loose, 0.45 kg): steel block 90 x 24 x 30 mm, 0.25 m forward and 0.29 m left, its length
  pointing away from the arms. Two screw seats (5.6 mm holes with 7 x 7 x 3 mm collars) and a 6.8 mm hole for the screwdriver.
- **Two screws** (2.7 g each): M5 x 6 hexagon socket head cap screws, zinc-plated steel (head 8.5 mm across and
  5 mm high, 4 mm hex socket), standing head up in the holder, 20 mm apart.
- **Hex screwdriver** (35 g): a stubby with a 4 mm hexagon blade 10 mm long, 6 mm round shaft, red 26 mm handle,
  85 mm overall; standing blade down in the holder.

The thread catches when a screw stands upright in a hole and is turned clockwise; from there about 4 turns bring its
head down on the rail, and a quarter turn more tightens it.

## How the parts are built

yam_arena's task files only offer single primitives and library assets, and yam_arena has no extension point for
jointed parts. `rp_parts.py` is the task's own minimal mechanism; nothing in yam_arena is modified. A `Part`
subclasses yam_arena's `Box`; `task.py` calls `rp_parts.install()`, which wraps `yam_arena.scene.task_object` and
`object_states` when that module is imported, and spawns the parts from USD files it writes to a temporary folder.

**Geometry and licensing:** every part's geometry was created by us for this task and is written into `rp_parts.py`
as coordinates; no manufacturer CAD, meshes or other third-party assets are used. The designs follow public standards:
the TS35 x 7.5 rail profile (EN 60715), the socket head cap screw's head (ISO 4762, M5) and the hexagon key size
(ISO 2936, 4 mm). The slot pattern, the plate, the holder and the screwdriver are our own designs. The robot, table and
cameras are yam_arena's own, unchanged.

**The thread.** A real M5 thread (0.8 mm pitch) is far too fine for PhysX, and a loose screw can't carry a
permanent thread joint, so each screw exists twice:
- the **loose screw**, an ordinary rigid body: a 0.6 mm reduced tip, the shank, and the head with its hex socket
  (the socket collides as six convex pieces between its flats and the head's rim, with a 0.25 mm entry chamfer);
- its **threaded twin**, an articulation fixed to the world and parked out of sight: anchor, thread slider coupled
  to the screw's turning by a PhysX mimic joint (0.8 mm per turn), a stiff clamp spring under the head (300 N/mm,
  0.5 mm travel), and the screw turning against a constant 0.03 Nm thread friction.

When the loose screw stands in a hole (its tip within 1.2 mm of the hole's axis, down in the countersink, upright
within 3 degrees) and is turned clockwise 20 degrees, the twin takes its place at the same position and angle: it has
caught its thread. Turned back out a quarter turn past where it caught, it is loose again (it can be lifted out).
The swap runs once per control step, before scoring.

### Thread bench

`rp_bench.py`: the screw is turned by a torque-limited "virtual screwdriver" (1 turn per second, at most 0.5 Nm) that
presses 10 N down once it has caught, as a hand does.

| case | result |
| --- | --- |
| screw held upright in a hole and turned (through the rail: 3 screws; into the bare plate: 1) | catches after the 20 degree turn, all 4 |
| head comes down on the rail / on the bare plate | 3.6-3.9 turns / 6.3 turns after catching (the catch depth varies) |
| tight: 0.25 turn past head contact | clamp 13-16 N on the rail, 62 N on the bare plate; let go, it springs back ~0.02 turn (the demo turns 0.1 further) |
| screw standing 15 mm beside a hole, turned 3 s | doesn't catch (topples) |
| tightened screw turned back out | loose again after a quarter turn past where it caught; lifts out freely |
| screw lying on the plate | rests still |
| unheld rail while its first screw is tightened | turns 0.06-0.1 degrees, moves 0.3 mm or less |
| rail with both screws tight, pushed along its length | slides at 18 N |

The bench holds a loose screw upright with a weak restoring torque and presses it lightly (1 N) until it catches,
as fingers do (pressed with 10 N, the tall socket head screw toppled); in one case (c_plain) the screw still tilted
5.5 degrees and, beyond 3 degrees, didn't catch. Results in `bench_runs/rp_bench.json`.

## The demo

Right arm grips the rail across its lips near its near end, lifts it 6 mm, carries and turns it until a slot is over
each hole (correct, settle, measure on the holes' offsets: the YAM wrist is soft and lags its command by 1-5 mm),
sets it down gently, lines it up again where it lies (still gripped; setting down, it jumps about 1 mm sideways in
the fingers), then lets go and holds it down from above with its closed fingers in the channel. Both holes end within
0.1 mm of their slots' centre lines.

Left arm takes screw 2 from the holder by its head, carries it upright to the far hole with its fingers across the
rail, centres the measured tip over the hole (two heights, correcting tilt about the tip), sets it into the
countersink and turns the hand 30 degrees clockwise about the screw's axis, keeping the screw upright: the thread
catches. It lets go with the fingers opened just clear of the head (wider, they pressed on the rail's sides and moved
the rail). The right arm, no longer needed (the started screw pins the rail), moves back; screw 1 goes into the near
hole the same way.

Left arm then takes the screwdriver from the holder from above (coaxially, the wrist yaw chosen for long strokes),
turns the key to the socket's angle, centres it and lowers it in, and drives each screw home in wrist strokes,
pressing down: between strokes it lifts the key out, turns back in the air and re-engages. It stops when a screw is
tight. Measured (CPU PhysX, 20 Hz control, jitter 0): **success at step 8,441** of 13,000 (score 1.0); screw 2: 18
strokes (9 re-engagements after the key rode out), 0.31 turn past head contact, clamp 14 N; screw 1: 9 strokes, 0.27
turn past contact, clamp 16 N. The video runs 3 s past success.

Variants proving the ladder (CPU PhysX, each variant run from a fresh start):

| variant | what the arms do | score | steps (`full`: at success; others: until the arms are done) |
| --- | --- | --- | --- |
| `full` | as above | **1.0** (success) | 8,441 |
| `one_tight` | both started; screw 2 tightened, screw 1 driven 1.5 turns in | **0.75** | 7,109 |
| `no_tighten` | both started and driven 1.5 turns in, neither tightened | **0.5** | 3,585 |
| `one_started` | only screw 2 placed, driven 1.5 turns in | **0.25** | 2,115 |

## Dream-1 run

One trial with the command under *Run and score a trial* (`--policy dream`, CPU PhysX, cameras 320 x 176,
13,000 steps, 871 s): **task_score 0.0** (hit the step limit). Both arms reach again and again over the screw holder
and the plate, and touch the rail, but pick up neither a screw nor the screwdriver; the rail stays roughly where it
started, and the screws and the screwdriver stay in the holder. Log in `runs/dream1/`; the video is the task's
uploaded model-run video.

## Fidelity: what is simplified

- The thread is the twin swap above: the screw catches at a fixed rule (upright within 3 degrees, tip in the
  countersink, 20 degrees clockwise) rather than by thread contact; cross-threading is not modelled (a tilted screw
  simply doesn't catch).
- Tightness is judged by turns past head contact with the head pressing, not by torque: the thread friction is a
  constant 0.03 Nm and the clamp spring gives 5-20 N, far below a real M5 screw at 2 Nm (several kN). Tightening a
  simulated screw takes much less torque than a real one.
- The rail's web and lips are 2 mm thick (1 mm real) so contacts don't tunnel at 240 Hz. The screw head bears on 12
  small balls on a ring under it (a flat underside straddling the slot sank into the web in PhysX).
- The loose screw's inertia is raised to 5e-7 kg m2 (its real ~3e-8 spins up uncontrollably in one 240 Hz step); its
  mass is real.
- The hex key is 3.9 mm across flats in a 4.0 mm socket (real: ~0.02 mm play) so PhysX contacts don't jam; the
  socket's entry chamfer is 0.25 mm.
- Plate holes are octagonal (4.4 mm across flats, the tapped hole's core) with a 45 degree countersink.
- Friction: steel 0.3, screw 0.2, screwdriver handle 0.9, table and robot fingers PhysX default 0.5. Contact offset
  0.5 mm on the parts, rest offset 0.

## Known weak points

- The scripted demo needs many corrections (the YAM's soft wrist): the key rides out of the socket several times
  while tightening and the hand re-engages. Runs from the start position repeat exactly; the variants were each run
  from a fresh start.
- Once both screws are tight the rail is held only by the simulated clamp (it slides at 18 N when pushed along
  its length); a policy that bumps the rail hard after tightening can move it, though the screws stay through their
  slots.
- The plate lies flat, so the second arm is helpful (it keeps the rail lined up while the screws are placed and
  started) but not strictly needed: an unheld rail barely turns while its first screw is tightened (under 0.1 degree in the
  bench).
- `rp_parts.install()` wraps two functions of `yam_arena.scene`; a yam_arena update that changes them stops the task
  with an error. It must run before `yam_arena.scene` is first imported (true for `-E task=task.py`).
- The scripted demo runs through `demo.py` (the same `YamArenaEmbodiment` and task-file scoring as `inspect-robots
  run`), not the CLI.
