# Secure a cable in a terminal box's cord clamp (two arms)

Every machine or appliance with a fixed cable (a motor's terminal box, a heater, a machine connection box) anchors
the cable inside with a cord clamp (strain relief), so that a pull on the cable is taken by the clamp and doesn't reach
the terminals. Here a cast-aluminium terminal box stands fixed on the table, lid off. A control cable comes in through
its side entry; inside, its sheath is stripped back and its three cores are already connected to the terminal block.
Just inside the entry is the cord clamp: a steel bar over the cable, held by two loose socket screws. The sheath's cut
end lies under the bar's outer half, so the bar is mostly over the bare cores. The arms must slide the cable in until the
**sheath** (not the cores) is under the bar, then tighten the two screws **in turns** with a hex screwdriver so that
the bar stays parallel and grips the sheath, neither too loose (the cable pulls out) nor so tight it crushes the cable.

The judgments tested: where the sheath must sit before clamping (clamping on the cores is a classic inspection fault
and scores 0), how far to tighten, and tightening two screws alternately so the bar doesn't tip.

**Gland or clamp?** The board's cable-gland task fits a cable through a gland in an enclosure's wall: the gland
**seals the cable entry** (the box's protection against water and dust) and anchors the cable from outside. A cord
clamp is the other common anchorage: it **holds the cable inside the device**, after its cores are connected, and
seals nothing. The skills differ: positioning the sheath against already-connected cores, and two screws and a bar
tightened in turns instead of one nut. Nearest tasks on the board: #198 clamp-cable-straight-in-fixture (a cable
clamped straight in a test fixture: no screws, no decision where the sheath goes), #2144 terminal-wire-screw (a wire
under a terminal screw), and the cable gland above.

All parts are our own simplified designs (no manufacturer CAD; see *How the parts are built*).

**Status:** the scripted two-arm demo, one arm holding the cable while the other tightens the screws, reaches the
full score (1.0, success at step 6,595 of 9,000); variants show 0.75, 0.25 and 0 with the arms, and the bench shows
1.0, 0.75, 0.25 and 0. Dream-1 (one trial, 9,000 steps): 0.0; it reached toward the screwdriver and the box but never pushed the cable in.

## Files

| file | what it is |
| --- | --- |
| `task.py` | the yam_arena task file: scene (`OBJECTS`), `success`, `score`, `distance`; docstring = what is on the table |
| `cc_parts.py` | the custom parts (box with saddle and terminal block, clamp screws, bar, cable, screwdriver and holder): geometry, USD writer, the per-step update (head contact, crushing, cores clamped, pull test) and the hook that lets yam_arena spawn them |
| `demo.py` | scripted two-arm demo, closed loop on the simulated object poses, and its variants |
| `cc_bench.py` | test bench for the clamp alone, the screws turned by a "virtual hand" (the table under *Clamp bench*) |
| `preview.py` | renders the starting layout from the station cameras |
| `runs/full_video/` | the full demo run: `result.json`, `setup.png` (the starting layout); its video (the three cameras in one) is the task's uploaded demo video |
| `runs/uneven_video/` | the `uneven` variant's run with video (0.75): `result.json` |
| `task.json` | submission metadata |

## Run and score a trial

Environment: yam_arena (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, PhysX at 240 Hz). Run at 20 Hz control with object
jitter off (one fixed layout, no seeds needed):

    inspect-robots run --embodiment yam_arena -E task=task.py -E control_hz=20 -E object_jitter=0 -E device=cpu \
        --policy <policy> --scorer task_score --max-steps 9000 \
        --instruction "Secure the cable in the terminal box's cord clamp: slide the cable in until its sheath reaches at least 5 mm past the clamp bar and hold it there while you tighten the bar's two screws in turns with the hex screwdriver, so that the bar presses on the sheath and stays parallel. Then let go of the cable and the screwdriver."

CPU physics (`-E device=cpu`) is what the task was built and tested on (the clamp's contacts are sub-millimetre; for
this one-environment scene CPU PhysX is also faster). The cameras render on the GPU.

`task_score` gives 1.0 when the episode ends in success, otherwise `score()` at the last step. Runtime per trial:
about 30 s of simulator start-up, then about 14 control steps per second on CPU PhysX with the three cameras
rendering (the scripted demo: 6,655 steps in 556 s with 640 x 360 video); a full 9,000-step trial is about 11 minutes.

Scripted demo (drives the same embodiment and task-file scoring directly; CPU physics by default):

    python demo.py --variant full --out demo_runs/full      # result.json, steps.jsonl, one video per camera
    python demo.py --variant all --no-video                  # full, uneven, one_screw, no_tighten, cores in one process

Clamp bench: `python cc_bench.py --case all --out bench_runs` (one JSON per case).

## Scoring

| score | condition |
| --- | --- |
| 1.0 | sheath past (below), the cable holds the pull test, **both screws snug** (each 0.25 to under 1.75 turns past its head's first contact), **tightened in turns** (their turns past first contact differ by less than 0.25 turn) and the **bar parallel** (tilted under 2 degrees); hands off, cable, screws and bar still for 1 s (success, ends the run) |
| 0.75 | sheath past, holds, both snug, but not tightened in turns (their turns past first contact differ by 0.25 turn or more), or the bar tilted 2 degrees or more |
| 0.5 | sheath past and holds, but a screw not snug (short of 0.25 turn past its first contact, or never touching) |
| 0.25 | sheath past only |
| 0 | otherwise; or the **cores clamped**: the bar ever pressed (either head 3 N or more for 0.25 s) while the sheath's end wasn't past the bar; or the cable **crushed**: either screw ever turned 2 turns or more past its first contact (both latched, even if loosened again) |

- **Sheath past:** the sheath's cut end at least 5 mm past the bar's edge on the terminals' side (9 mm past the
  bar's centre line). At the start it is 1 mm short of the centre line (the bar over 3 mm of sheath and 5 mm of
  cores). Measured before the pull test if the test
  failed (ruler).
- **First contact:** a screw's head presses the bar (2 N or more, held 0.5 s): in a real trial, the point where the
  screw becomes finger-tight; mark the head there. Turns since then are read on that mark: 0.25 and 1.75 turns
  (snug between), 2 turns (crushed). About 1.5 turns from the start bring a head onto the bar.
- **In turns:** the two screws' turns past their own first contact (read on the marks) differ by less than a
  quarter turn. Tightened alternately in small steps, they end close together; one screw tightened all the way before
  the other leaves them further apart. Judged on each screw's own contact point, not the heads' heights, which also
  depend on where each head first met the see-sawing bar.
- **Bar parallel:** the bar tilted under 2 degrees, measured at its ends, which sit under the screw heads: the two
  heads' heights differ by less than 0.77 mm over their 22 mm spacing (depth gauge or caliper on the heads; 0.7 mm
  per turn, so the screws less than 1.1 turns apart). Real clamping bars see-saw over the round cable: tightening
  one screw far ahead of the other leaves the bar crooked.
- **Pull test:** run by the task itself once no fingertip and no screwdriver tip has been within 60 mm of the screw
  heads, the bar or the cable within 80 mm of the clamp for 1 s, and a head presses the bar: the cable just outside
  the box is pulled outward along the cable, ramped to 30 N over 1 s and held 0.5 s. It holds if the cable moves less
  than 1 mm at the clamp (the sheath's end). The result counts until a screw turns 0.02 turn or the sheath moves 1 mm;
  then the test runs again once the hands are off. Real trial: a spring scale on the cable, 30 N, and a mark on the
  cable at the clamp.
- Measurable with a ruler, a marker, a feeler gauge, a spring scale and a photo.

No unintended strategy scores: tightening the bar over the cores scores 0 for good (even if the cable is pushed in
afterwards); pushing the cable in and letting go scores 0 (it springs back short of the goal), and holding it in
without tightening caps at 0.25 (the pull test never runs with a hand on it); turning a screw far scores 0
for good; the box, the saddle and the screws' threads are fixed (the screws can't come out). Holding the cable or the
screwdriver at the clamp at the end leaves the pull test unrun (no more than 0.25).

## Setup

All positions from the arm bases' midline (x forward, y to the left); the table top is the reference height.

- **Terminal box** (fixed): cast aluminium, grey, 150 mm deep (x) x 155 mm wide (y) x 60 mm tall, 4 mm walls and
  floor, lid off; its footprint centred at x 0.26 m, y 0.0025 m (right wall's outer face at y -0.075 m). A 12 x 12 mm
  cable entry in the middle of its right wall, its centre 18 mm above the table.
- **Cord clamp** (inside the entry): an aluminium saddle on the box floor, 36 mm (x) x 10 mm (y), its top 14 mm above
  the table, centred at x 0.26 m, y -0.014 m (57 mm inside the right wall). Two M4 x 16 black socket cap screws (head
  7 mm across, 4 mm high, 3 mm hexagon socket), 22 mm apart along x, already started in the saddle's tapped holes,
  heads 1 mm above the bar. The clamp bar: steel, 32 x 8 x 3 mm, ribbed underside, two 4.6 mm holes on the screws,
  resting on the cable. Screws tighten clockwise seen from above, 0.7 mm per turn.
- **Terminal block** (fixed, blue, 40 x 14 x 22 mm) near the box's left wall, its face at y 0.058 m.
- **Cable**: round control cable, 3 cores x 1.5 mm2, grey PVC sheath 8 mm across. From the terminal block, the three
  cores (brown, blue, green-yellow, taped into one bundle, ~6 mm across) run 80 mm toward the entry in a slight upward
  bow (slack); then the sheath's cut end, 1 mm short of the bar's centre line; the sheath runs level through the
  clamp and the entry and 40 mm beyond, drops to the table, and lies on it for ~180 mm, curving toward the arms (its
  last link 55 degrees from the entry's axis); its far end is free. The cores' bow is springy: pushed in and let go
  before the clamp grips it, the cable springs back until the sheath's cut end is about 3-5 mm past the bar's centre
  line, short of the 9 mm needed. It has to be held while the screws are tightened.
- **Hex screwdriver**: 3 mm hexagon blade 9 mm long, 5 mm shaft, red handle 24 mm across, 80 mm overall; it stands
  blade-down in a steel holder block (40 x 40 x 30 mm) on the table at x 0.17 m, y 0.19 m (left of the box).

## How the parts are built

yam_arena's task files only offer single primitives and library assets, and yam_arena has no extension point for
jointed parts. `cc_parts.py` is the task's own minimal mechanism; nothing in yam_arena is modified. A `Part`
subclasses yam_arena's `Box`; `task.py` calls `cc_parts.install()`, which wraps `yam_arena.scene.task_object`,
`object_states` and `make_env` when that module is imported, and spawns the parts from USD files it writes to a
temporary folder. The `make_env` wrapper also resets the clamp and the cable with each episode.

**Geometry and licensing:** every part's geometry was created by us for this task and is written into `cc_parts.py`
as coordinates; no manufacturer CAD, meshes or other third-party assets are used. The screws follow the general
form of an M4 socket head cap screw (ISO 4762), the key a 3 mm hexagon key (ISO 2936); all other dimensions are our own
choices. The cable's size is that of a common 3 x 1.5 mm2 control cable; its stiffness is our estimate.

**The clamp** is one world-fixed articulation. Each screw: a prismatic "thread" joint coupled to the screw's turning
by a PhysX mimic joint (0.7 mm per turn), a stiff spring under the head (60 N/mm, 2.5 mm travel: the give of the
bar and the cable's sheath under it), and the screw's spin with friction. The bar is a free rigid body on the screws'
shanks; the heads press it down by contact, it presses the cable onto the saddle (ribbed bar and saddle, friction
0.8). The hold in the pull test comes from that contact friction, so tightening only one screw tips the bar
(see-saw over the cable) and holds less.

**The cable** is an articulation rooted at the terminal block (so the cores stay connected): five thinner, softer core
links (16 mm, r 3.2 mm, laid in a slack bow), then sixteen 20 mm sheath links (r 4 mm), joined by two bending joints
and a twisting joint each, with springs (EI 0.006 N m2, GJ 0.004 N m2; cores 0.0009 N m2). Its laid shape is built
into the USD. The sheath's bending springs' rest angles creep toward the current shape (the set a PVC sheath takes;
beyond 6 degrees, time constant 1.5 s); the connected cores' bow stays elastic: pushed in, the tighter bow pushes the
cable back out until something holds it (a hand, or the clamp).

**The screwdriver** is a free rigid body; its 2.9 mm hexagon blade engages the 3 mm socket by contact (0.05 mm play
each side), so the robot must line the key up with the socket's flats to turn a screw.

**The per-step update** (from the patched `object_states`, once per control step, before scoring) follows each
screw's first head contact and turns since, latches "crushed" and "cores clamped", and runs the pull test.

### Clamp bench

`cc_bench.py`: the clamp in the task's scene, the arms parked. A "virtual hand" pushes the cable in (8 N on the
sheath just outside the entry until its end is 18 mm past the bar's centre), holds it in with 3 N (the cores' bow
pushes it back), turns the screws in alternation (half a turn each, up to 1.5 Nm, a hand on a short hex key) to the
case's targets past their first contact, or as far as they go, brakes them to a stop for 0.5 s and lets go of
screws and cable; the task's own pull test follows. Two more cases push the cable in and let go without touching the
screws.

| case | targets (turns past contact) | at release | after release | pull test (30 N) | score |
| --- | --- | --- | --- | --- | --- |
| snug | 0.5 / 0.5 | 0.54 / 0.51 | 0.34 / 0.49, heads 0.3 deg | held (0.09 mm) | **1.0** |
| uneven | 0.35 / 1.2 | 0.42 / 1.18 | 0.27 / 1.19: 0.91 turn apart, not in turns (heads 1.8 deg) | held (0.68 mm) | **0.75** |
| loose | 0.12 / 0.12 | 0.50 / 0.31 | 0.42 / 0.21 | slipped (1.04 mm) | **0.25** |
| crush | 2.1 / 2.1 | 2.23 / 1.26 | crushed (latched) | slipped (1.00 mm) | **0** |
| cores (not pushed in) | 0.5 / 0.5 | 0.52 / 0.44 | cores clamped (latched) | held (0.02 mm) | **0** |
| let_go | none: pushed to 18 mm, held 0.5 s, let go | | the sheath's end springs back to 3.0 mm past the bar's centre | not run | **0** |
| let_go_held | none: pushed to 18 mm, held 20 s, let go | | springs back to 3.1 mm | not run | **0** |

After being turned hard (up to 1.5 Nm) and let go, the screws can spring back 0.1-0.2 turn (the simulated thread
coupling winds up under that torque); the robot turns them with far less torque. Earlier bench runs, with the clamp
nearer the entry, gave the same spring-back for other pushes and holds: let go after a push to 25 mm, the sheath's
end settled at 4.5-4.7 mm.

## The demo

Right arm takes the cable just outside the box's entry, from above but tilted 40 degrees, its wrist leaning out away
from the box (clear of the other hand's wrist over the box), and pushes it in along the entry's axis in one stroke,
closed loop on the sheath's measured end, toward 18 mm past the bar's centre line (the goal is 9 mm). It then holds
the cable there, still pressing it in a little against the cores' bow, until the other arm has tightened both
screws; then it lets go and moves home.

Left arm takes the hex screwdriver out of its holder by the handle, from above, and holds it upright. Once the cable
is in and held, it tightens the two screws in turns: over a screw, turn the key to the socket's angle, centre it on
the socket and lower it in (a small spiral if the tip rests on the rim), then wrist strokes clockwise (the hand
pressing the key down, harder once the head presses the bar), lifting the key out and turning back between strokes.
About one turn on each screw while they are loose, then shorter goes, alternating, toward 0.8 turn past first
contact, until a further round gains nothing (as tight as this grip gets them). It lifts the key out (the cable hand
lets go), lays the screwdriver on the table and moves home; after 1 s hands off the task pulls the cable.

Measured (CPU PhysX, 20 Hz control, jitter 0): **success at step 6,595** of 9,000 (score 1.0): the push brought the
sheath's end to 15.5 mm, held at 13.0-15.5 mm from step 272 to 6,490; 28 key strokes; first contact 1.47 and 0.95
turns from the start (the bar see-saws over the cable), stopped 0.73 and 0.79 turn past it (in turns: 0.06 apart),
the bar tilted 0.8 degrees at the heads; pull test 0.004 mm. The video runs 3 s past success.

Variants proving the ladder with the arms (all six in one process, `--variant all --no-video`, which also checks
the reset; `full` gives the same result run on its own with video):

| variant | what the arms do | score | steps |
| --- | --- | --- | --- |
| `full` | as above (cable held): 0.73 and 0.79 turn past first contact | **1.0** (success) | 6,655 |
| `uneven` | cable held; the screws alternately but unevenly: one to 0.5 turn past contact, the other on to 0.9, then a last go on the first | **0.75** run on its own with video (0.47 and 0.87, 0.40 apart, not in turns; held 0.72 mm; 5,999 steps, `runs/uneven_video/`); **0.25** in the all-variants run (see below) | 7,042 |
| `one_screw` | cable held; one screw to 0.74 turn, the other barely past contact (0.09) | **0.25** (slipped 3.05 mm) | 3,137 |
| `let_go` | the cable pushed in and let go, then both screws tightened | **0** (it sprang back to 1.5 mm, under the bar: cores clamped) | 7,700 |
| `no_tighten` | the cable pushed in and let go, the screws not touched | **0** (it springs back to 2.7 mm) | 432 |
| `cores` | the cable not pushed in, the screws tightened onto the cores | **0** (cores clamped) | 9,000 |

In the all-variants run, `uneven` (after `full` and a reset) registered its first screw's first contact early, at
0.95 turns from the start instead of 1.47: the see-sawing bar pressed against that head while the other screw was
tightened. Counted from there, 0.47 turn past contact pressed the bar only 5.6 N (14 N run on its own), and the cable
slipped in the pull test (3.57 mm): 0.25.

## Fidelity: what is simplified

- The threads are joints, not thread contact: 0.7 mm per turn; the screws can't cross-thread or come out. Their
  friction is PhysX joint friction, which grows with the joint's load (as a real thread's torque grows with the clamp
  load).
- "Crushed" is judged by turns past first contact (2 turns), not by modelling damage to the sheath; the sheath's and
  the bar's give is one spring under each head.
- The cable is a chain of rigid links with springs; the sheath's "set" is a creep of the springs' rest angles, the
  cores' spring-back their bow's elastic springs (stiffness tuned so a cable let go settles 3-5 mm past the bar's
  centre line: bench). Stiffness values are our estimates. The taped cores are one bundle.
- The bar is simulated at 60 g (a real one is ~6 g): lighter, it sank between the screws and the cable in the solver.
- The give of the sheath and the bar is a spring under each head (60 N/mm), not in the sheath under the bar's middle.
  The simulated bar body can therefore level out between two pressing heads; the score takes the bar's tilt from the
  heads' heights, where its ends sit on a real clamp.
- The simulated thread isn't self-locking: the load under a head can push its screw back a little (about 0.1 turn
  when the other screw is tightened far ahead of it, through the slightly soft coupling between the screw's turning and
  its advance). A real M4 thread holds. In the demo the hand tightens each screw again in the next round. (Holding
  the screws with a joint drive while no key is in the socket made it worse: a jump of ~0.17 turn when the drive took
  hold.)
- The robot's grip on the screwdriver's handle limits how tight it gets the screws (both stall at roughly 15-20 N
  under the head, about 0.3-0.7 turn past contact); the bench's "virtual hand" (up to 1.5 Nm) reaches the crush
  mark. Turned that hard and let go, a screw springs back about 0.1 turn.
- The pull test is applied by the task (an external force on the cable just outside the box), not by a robot.
- Friction: ribbed bar and saddle 0.8, PVC sheath 0.5, the robot's fingers PhysX default 0.5. Contact offset 0.5 mm.

## Known weak points

- Borderline variants: `one_screw` (one screw snug, the other barely past contact) held the pull test in one earlier
  run and slipped in others. The arms' `uneven` variant scored 0.75 on its own but 0.25 after a reset, when its first
  screw's first contact registered early (see *The demo*).
- The scripted demo needs closed-loop corrections (the YAM's soft wrist) and is tuned to this layout. With the clamp
  close to the entry, the hand holding the cable and the screwdriver hand collided over the box (their wrists); the
  clamp now sits 57 mm inside the entry, and the cable hand grips tilted, its wrist leaning out. Runs from the start
  position repeat exactly.
- Simulator crashes: a few of our runs stopped with a crash in the camera renderer (not the physics), each time with
  two or three simulations rendering on one GPU at once; we have not seen it in a run on its own.
- A cable that fails the pull test is pulled out by the test; the score then uses the sheath's position from before
  the test.
- `cc_parts.install()` wraps three functions of `yam_arena.scene`; a yam_arena update that changes them stops the task
  with an error. It must run before `yam_arena.scene` is first imported (true for `-E task=task.py`).
- The scripted demo runs through `demo.py` (the same `YamArenaEmbodiment` and task-file scoring as `inspect-robots
  run`), not the CLI.
