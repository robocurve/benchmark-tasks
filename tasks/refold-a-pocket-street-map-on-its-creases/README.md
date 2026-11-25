# Refold an opened laminated pocket street map along its accordion creases

An opened laminated city street map lies on the table. Its six accordion creases alternate valley and mountain. The laminate keeps
every crease springy, so the map lies as a shallow zigzag rather than flat. The robot has to fold it back into its 10 x 21.6 cm
packet, with the cover outside, and leave the packet lying flat. Every crease has to close the way it was made. One crease closed the
wrong way puts the cover inside (crease 1) or makes a fold that will not lie flat, and a laminated crease pushed the wrong way springs
back open. A map also cannot be folded panel over panel like a roll, because every second crease would then close backwards.

What works is gathering it like a concertina between two hands. First the cover panel is turned up over its neighbour by its raised
edge. Then both ends are pushed in together, low on the end panels, so that every ridge rises, every valley stays on the table and
each crease closes in its own direction. Pushed in carelessly at table height with nothing gathered first, the map bows up off the
table and only some creases shut.

- Instruction: "Fold this street map back up into its pocket-size packet, cover on the outside, and leave it lying flat on the table."
- Category: Societal relatability.
- Run limit: 2,400 steps (2 minutes at 20 Hz). The scripted demonstration finishes in about 710 steps (35.5 s).

## Run and score

The task runs in the YAM arena simulator (`yam_arena`), with Inspect Robots. After `source ~/IsaacLab-Arena/yam.env`, a model run is:

```
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/refold-a-pocket-street-map-on-its-creases/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fold this street map back up into its pocket-size packet, cover on the outside, and leave it lying flat on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/refold-a-pocket-street-map-on-its-creases
```

`--scorer task_score` gives 1.0 once `success()` is true (the scorer counts its own 1 s hold, so `SUCCESS_HOLD_S = 0`); otherwise it
gives the last step's `score()`. The scripted demonstration is a closed-loop script (`demo_script` in `task.py`). It runs through
`demokit.py` from this folder, in the simulator's Python environment:

```
cd tasks/refold-a-pocket-street-map-on-its-creases
python demokit.py task.py ~/yam-eval/refold-a-pocket-street-map-on-its-creases/demo --mode full        # writes full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/refold-a-pocket-street-map-on-its-creases/redteam --fast --mode none,careless,onearm,stand,hold
```

The scene has no randomness: the map starts in one fixed layout and every seed gives the same start. A trial takes about 1 minute
of wall time on one GPU once shaders are compiled (`--fast`, physics on the CPU with no cameras, takes about 40 s).

## The real objects

**The map.** A Michelin Streetwise Chicago laminated city map (ISBN 9782067259973, about EUR 8.50 or $10). The publisher's listing
describes it as a laminated, "accordion-fold pocket size travel map", 4 x 8.5 in folded, 68 g, 4 mm thick folded (212 x 98 mm).
Retailer listings give it as 8.5 x 28 in open, which at the 4 in folded width is seven panels: six creases. In the model, each panel
is 10.16 x 21.59 cm (4 x 8.5 in) and weighs 9.7 g, which gives 68 g for the map.

The cover panel is at one end. Its underside is printed with the cover (red here, with a title band). The other faces carry the map
(drawn here as a cream sheet with grey streets, a yellow main road, a green park, and lake shore on the last panels). Crease 1, next to
the cover panel, is a valley when the map is seen open from the map side, and the rest alternate: valley, mountain, valley, mountain,
valley, mountain. Folded correctly, panel 2 closes onto the map side of the cover panel, so the cover ends up outside, facing out,
whichever way up the packet lies.

**The table.** The station table. The map's long centre line is 27 cm in front of the arm bases. It spans y = -36.1 to +26.1 cm open
(the cover end on the robot's right) and lies in reach of both arms.

**Distractors** (scoring ignores them, and they are out of the arms' paths): reading glasses at (53, 14) cm and an empty paper coffee
cup at (50, -20) cm, things that sit on a table next to a map being read.

## How the simulator models it

- **The map** is one free articulation of seven rigid panels. The middle panel is the frame, and each other panel hangs on a hinge
  along its crease, so the six hinges form two chains out from the middle. Each hinge's axis lies on the face that closes onto its
  neighbour. A crease closed to 180 degrees therefore lays the two panels face to face, and a closed packet is seven panels stacked
  1.5 mm apart. The panels are 1.5 mm plates (the real laminate is about 0.5 mm; see fidelity), with gravity, at 9.7 g and plate
  inertias. Self-collision is off inside an articulation (a PhysX rule), so the joint limits are the stops. A crease can close to
  180 degrees the way it was made and bend up to 140 degrees the wrong way.
- **Crease spring-back** (code, every control step): the laminate makes each crease a weak spring (0.03 N m/rad). While a crease is
  open it pulls toward a 60 degree set, the "half open" zigzag a laminated map springs back to. Once a crease is folded past 160
  degrees the right way, it takes its folded set: the spring is aimed 5 degrees past the closed stop, so the crease stays shut. It
  keeps that set until it is opened again below 110 degrees. A crease bent the wrong way always springs back toward its 60 degree set,
  as a reversed laminated crease does.
- **Friction.** The laminate's film slides on the table at 0.35 static and 0.3 dynamic. The map's material uses PhysX's "min"
  friction combine, so a contact takes the lower of the two values, not the average with the table's rubber-like 1.2.
- **Starting layout.** Joint zero is the sprung-open zigzag, so the map starts at rest. Each panel is 30 degrees from level, the
  valleys are on the table and the ridges are 5.1 cm up; gravity sags the cover panel's raised edge by a few degrees in the 0.5 s
  settle.

## Scoring

All of it is read at the end, from what a person could check: the packet's edge photographed from the side (each crease's direction),
a ruler (the packet's height), and the video (the grippers).

| Score | Condition |
|---:|---|
| 1.0 | All six creases closed the way they were made (folded 165 degrees or more, i.e. the two panels within 15 degrees of face to face), every panel within 15 degrees of level and every panel corner within 3 cm of the table top, every fingertip point of both grippers (each finger's tip and 3 cm up it) at least 4 cm from every panel, and every panel still (under 2 cm/s and 0.2 rad/s), all held for 1 s |
| 0.5 | At the end, all six creases closed the way they were made and both grippers 4 cm clear, short of full credit (for example the packet left standing on its edge) |
| 0.25 | At the end, at least three creases closed the way they were made, none reversed (bent 90 degrees or more the wrong way), and both grippers 4 cm clear |
| 0 | Anything else, including the opened starting map and any gripper within 4 cm of the map |

A map that goes over the table's edge (any panel corner more than 3 cm below the table top) scores 0 for the whole trial. That record
is kept across steps and cleared only by the simulator's reset hook. Possible scores: 0, 0.25, 0.5, 1.0.

All six creases closed the right way implies the cover is outside with its face out, so "cover on the outside" needs no extra check.
A packet with crease 1 reversed has the cover inside and cannot score more than 0.25. The 0.5 level covers a properly folded packet
that was not laid down (a loose accordion stood on its edge falls open in a glovebox); the 0.25 level covers half a job that is
headed the right way.

## Real trial

**Setup.** Station table, clean and dry. Lay the map open with its map side up, cover panel at the robot's right end (cover side
down), long centre line 27 cm in front of the arm bases, and the cover panel's free edge 36 cm to the right of the midline (tape marks
at both free edges). Set the zigzag: valley creases 1, 3 and 5 on the table at y = -27.2, -9.4 and +8.3 cm, and ridges 2, 4 and 6 at
5.1 +-0.5 cm high (a steel rule stood beside each ridge). Distractors on tape marks: glasses at (53, 14) cm, cup at (50, -20) cm. Both
grippers open at their ready poses. The 60 degree spring-back is our estimate for a laminated map, not a measurement. If a copy lies
flatter, its ridges are set by hand to the same height before each trial, which is what the measured preparation above does.

**Measurement.** Read the end state before anyone touches the map. Photograph the packet's long edge from the side: each crease shows
as a U turn, so a crease closed the wrong way is visible, and "closed" means the two panels touch or are within 15 degrees (a gap of
at most 2.6 cm at the free edges, checked with a ruler). Measure the packet's top with a ruler (at most 3 cm above the table), and
check from the video that both grippers stayed at least 4 cm from the map for the last second, and whether the map went over the
table's edge.

**Reset (about 1 minute).** Open the map back out along its creases, cover panel on its mark, and set the ridges to 5.1 cm again.
Nothing is consumed. Replace the map if a crease starts to split or a new crease forms (look along each panel for a fold line).

**Rig safety.** The map weighs 68 g and has no sharp parts; the gripper pads touch only the laminated film. No force a gripper can
apply damages the table or the arms.

## Distinct from

| Nearest | What it tests | How this differs |
|---|---|---|
| #3496 Fold a quad-fold game board up in its one working order | four stiff leaves of a game board, folded leaf over leaf in one order and boxed | here seven light springy panels on alternating creases, gathered from both ends at once as a concertina; panel-over-panel folding is exactly the wrong way |
| #2306 Stand a folded birthday card on a shelf as a tent (accepted) | one crease: turning a card crease-up and setting its edges down | six dependent creases whose directions alternate; the card is never refolded |
| #1425 Zig-zag fold a four-panel array wing | a hobby solar-array wing with spring-loaded hinges | an everyday street map; the knack is gathering a springy concertina without bowing it, and the cover-outside order |
| #1329 Impose an eight-page booklet | page-order planning before folding paper | no planning: the creases already exist and the skill is physical |
| #2534 Pinch-zoom a street map on a tablet | a touchscreen gesture | a paper map and a two-handed fold |

None of LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench or RoboDojo has an accordion-folded map or a multi-crease concertina
refold. Cloth-folding sets (towels, shirts) fold a limp sheet along new lines, not along fixed alternating creases.

## Importance and sources

Folding a road or street map back up along its own creases is a classic everyday frustration: most adults have opened a folded map
and struggled to get it shut again. City maps like this one are still sold in large numbers to visitors. The difficulty is the
physical knack, and that is what a robot has to show: two hands in different roles at the start (one turning the cover panel up while
the other waits as a stop), then a matched two-handed gather. Every crease has to close in its own direction, and a careless push
bows the map instead.

- Michelin Streetwise Chicago, publisher listing at a retailer (laminated, "accordion-fold pocket size travel map", 4 x 8.5 in folded,
  212 x 98 x 4 mm, 68 g, EUR 8.50): https://www.schweitzer-online.de/en/sonstige/MICHELIN/Streetwise-Chicago-Map/9782067259973/A64871505/
- The same map at another retailer ("accordion folding for effortless use", laminated, 4 x 8.5 in folded):
  https://www.abebooks.com/9782067259973/Streetwise-Chicago-Map-Laminated-City-2067259970/plp

The open size of 8.5 x 28 in comes from retailer listings that we could not open directly. It agrees with seven 4 in panels. The crease
spring-back angle and stiffness are our estimates.

## Assets and helpers

The map is a custom mesh written by `task.py` (SimReady has no folded map). The distractors are NVIDIA SimReady props, downloaded at
first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` (© NVIDIA, CC-BY 4.0; masses set at load time, nothing
else changed):

- [accessory_eyeglasses_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01): reading glasses, 30 g.
- [dishware_cup_paper_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_cup_paper_a01): empty paper coffee cup, 10 g.

`mechkit.py` is the shared mechanism kit from `tasks/heat-shrink-lug/` in this repository (another team's open-source kit), unchanged.
`task.py` writes the panels' colliders as box primitives turned to each panel's angle, because thin convex hulls do not cook for
the GPU. It also adds the "min" friction combine to the map's physics material when the map's USD is written. `simready.py` is the
SimReady loader from `tasks/cutters-off-pegboard/`, unchanged apart from its docstring. `demokit.py` is the closed-loop demo runner
shared by `tasks/heat-shrink-lug/` and other tasks, unchanged.

## Validation

Scripted demonstration (`--mode full`, recorded on the GPU; scores are the harness's `task_score`):

| Run | Score | Control steps | End state |
|---|---:|---:|---|
| demo 1 | 1.0 | 708 (35.4 s) | all six creases at 180 degrees, packet lying flat on its back panel with the cover up, both grippers clear |
| demo 2 | 1.0 | 708 (35.4 s) | the same |

Failure checks, on the final `task.py`, with physics on the CPU (`--fast`, same physics settings; the full demo also scores 1.0 this
way, at 712 steps):

| Mode | What it does | Score | End state |
|---|---|---:|---|
| none | nothing | 0.0 | the opened map as set |
| careless | pushes both ends in at once at table height, no cover fold first | 0.25 | the map bows up; creases 2, 3 and 5 shut, crease 1 (cover) and creases 4 and 6 open again once let go |
| onearm | pushes the far end in with the left hand only | 0.0 | the whole map slides across the table until its cover end hangs over the table's edge (2.1 cm below the top, inside the 3 cm limit); no crease shuts |
| hold | the full gather, then the right gripper pressing on the packet | 0.0 | a flat, correctly folded packet, but a gripper on it |
| stand | the full gather, then both hands let go without laying it down | 1.0 | the gathered packet was leaning (about 10 degrees off vertical) and fell flat by itself once let go, so the 0.5 "left on its edge" level was not produced by this mode |

The demonstration's settings (grippers leaned 42 degrees in, pushing at 28% of the end panels' height) were checked against
neighbouring settings on the CPU: leaned 40 or 45 degrees, pushing at 25% or 30%, also score 1.0. Leaned 45 degrees at 20%, or 50
degrees at 25%, leaves one or two valley creases at 137-152 degrees and scores 0.25.

**Model run.** Dream-1, run once on the final task for the full 2,400 steps, scored 0.0: its grippers moved over and around the map's ends but never folded the cover panel or gathered the map, which stayed open.
