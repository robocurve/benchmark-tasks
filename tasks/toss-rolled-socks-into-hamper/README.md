# Throw three rolled pairs of socks into a laundry hamper

**Instruction:** "Throw the three pairs of socks into the laundry hamper."

Three pairs of crew socks, each pair rolled into its own cuff, lie on a table. An open pop-up mesh laundry hamper stands on the floor
beside the table's far right corner, 1.15 m from the right arm's base (1.00 m forward and 0.57 m to its right). The arms reach about 0.6 m, so the only way to get a pair
into the hamper is to throw it. Score: one third for each pair that ends inside the hamper, and full credit once all three are in and
everything is still for 1 s.

## The knack

A rolled pair weighs about 40 g and doesn't bounce. To drop it into a 40.6 cm opening whose rim is 12.5 cm below the table top and
about 0.8 m beyond the point of release, it has to leave the hand at about 2.2 m/s, rising at 45-50 degrees. The hand gets that speed
only in a fast swing that changes direction as it goes, so the moment of release decides where the pair lands:

- the swing has to be fast. The demo's throw is a 0.25 s sweep of shoulder pitch and wrist pitch. At half that speed the pairs fall short,
  on the table or the floor. The window isn't narrow on the fast side: targets moved 25% faster still put all three in, because
  the arm's joint speed limits cap how fast the hand actually goes;
- the gripper has to open in a window about 0.1 s wide. In the demo's swing it opens on control step 4 (of 50 ms each); step 3 also
  works, but opened on step 2 the pairs drop on or beside the table, and on step 5 (or 7) they are flung down short and none goes in
  (failure checks below). Timing is the hard part;
- the throw has to be repeated three times, from three different starting poses (the pairs lie at different headings, so each grasp
  differs).

The robot can't use the easy way a person sometimes does, walking up to the hamper. It stays at the table, like someone tossing socks
from the edge of the bed.

## Run it

The demonstration is a closed-loop script (`demo_script` in `task.py`) run by `demokit.py`, in the simulator's Python environment, from
this folder:

```bash
cd tasks/toss-rolled-socks-into-hamper
python demokit.py task.py ~/yam-eval/toss-rolled-socks-into-hamper/demo --mode full        # writes full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/toss-rolled-socks-into-hamper/checks --fast --mode none,drop,weak,hard,early,late
```

`--fast` runs physics on the CPU without cameras (used for the scorer checks); the reported demo scores come from recorded GPU runs.
A model trial uses the ordinary harness:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/toss-rolled-socks-into-hamper/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Throw the three pairs of socks into the laundry hamper." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toss-rolled-socks-into-hamper/model
```

The layout is fixed (`object_jitter=0`, no seed-dependent randomisation, so no seeds are needed). The run limit is 1,200 control steps
(60 s at 20 Hz). The demonstration reaches full credit at about step 460 (23 s). A `--fast` CPU run takes about 30 s of wall time;
a recorded GPU run took several minutes on a shared GPU.

## The real objects

- **Hamper: Household Essentials Pop-Up Mesh Laundry Hamper**, listed as "16" Diameter x 25" H" (40.6 x 63.5 cm), 0.88 lb (0.40 kg),
  polyester mesh with a steel frame, open top, $16.97-19.99
  ([Target listing](https://www.target.com/p/pop-up-mesh-laundry-hamper-green/-/A-94093520)). It is modelled at that size and mass:
  a 2 mm wall drawn see-through, a steel hoop at the rim and at the base, and a floor (colliders: see Simplifications). The listing gives only the overall size; the hoop
  sections are our reconstruction.
- **Socks: men's cotton-blend crew socks, one pair rolled into its own cuff** (fold one sock into the other and turn the cuff over the
  bundle). For example Hanes Men's Work Crew Socks, 6 pairs, $18.00, estimated shipping weight 0.5 lb for the pack
  ([Target listing](https://www.target.com/p/-/A-83686529)), which is about 38 g a pair. The model uses 40 g and an 8 x 6 x 5.5 cm
  roll. Both are our estimates for a rolled men's crew pair: weigh and measure the real rolls before a trial, and set `SOCK_MASS` and
  `SOCK` in `task.py` if they differ.
- **Distractors** (SimReady props, ignored by the scorer): a folded microfibre towel, a paper notebook and a ceramic mug, all on the
  left half of the table, out of the throw.

## How the simulator judges it

All checks run every control step (20 Hz) in `Trial.step`, from object poses and the fingertip points:

| Rule | How it is checked |
|---|---|
| A pair is in | its centre, in the hamper's frame, is less than 18.3 cm from the axis (20.3 cm inner radius, less 2 cm), lower than 60.5 cm above the hamper's floor (3 cm under the rim: a pair perched on the rim is out) and above the floor, and it moves under 5 cm/s |
| The hamper stands where it was put | axis within 10 degrees of vertical, base centre within 5 cm of its start; otherwise the score is 0 |
| Score | pairs in / 3, shown as 0, 0.33 or 0.67; all three in before the hold completes also scores 0.67 |
| Full credit | all three in, hamper standing, every fingertip point 30 cm or more outside the hamper's wall, for 20 consecutive steps (1 s); 1.0, and it holds only while all three stay in |

The score is the current step's, so credit doesn't stick: knocking the hamper over or a pair bouncing back out removes it. Nothing is
latched across a reset except through `on_reset`, which clears the hold. `task.json` gives the same rules in words.

## Real trial: preparation, reset and measurement

- **Rig.** Two YAM arms at the near edge of a 76 cm high table (110 x 130 cm), bases 61 cm apart. Mark the hamper's footprint on the
  floor with four pieces of tape at the ends of two diameters, its centre at x 100, y -87 cm from the midpoint between the arm bases
  (1.15 m from the right arm's base, 29.5 degrees to its right), so that it stands just off the table's right side, 2 cm clear of it,
  near the far corner. Tape the three sock outlines and the three distractor positions on the table (setup in
  `task.json`).
- **Preparation.** Roll each pair the same way (one sock inside the other, cuff turned over), weigh it (about 40 g), and lay it flat
  with its long axis at the listed heading. Pop the hamper open and stand it on its marks.
- **Safety.** A 40 g sock ball can't hurt anyone, but keep people out of the throwing line in front of the table during a trial.
  Nothing in the scene is sharp, hot or wet.
- **Measurement.** At the end of the run (or after the arms have stopped for 1 s), look into the hamper: count the pairs lying inside
  (not perched on the rim). Check with a tape measure that the hamper's base is within 5 cm of its marks and that it stands upright (a
  phone inclinometer on the rim hoop reads within 10 degrees). A pair on the table, the floor or the rim doesn't count.
- **Reset (about 1 minute).** Take the socks out, lay them back on their outlines, and stand the hamper back on its marks.

## Distinct from

| Nearest task | What it tests | Why this is different |
|---|---|---|
| #2481 Throw an empty can into a pedal bin (rejected) | working a pedal-operated lid, then a short drop into a bin beside the robot | here nothing has to be opened. The target is out of reach on the floor, and the whole skill is the throw: speed, angle and release step |
| #2310 / #3045 Toss diced potato in a sauté pan | a pan flip: the food leaves and lands back in the same held pan | here the object is released from the hand and has to land in a separate, distant target |
| #610 / #2550 / #3120 / #3132 throwing away rags or a patch | choosing the right container and putting the item in (placing, not throwing) | here the container is out of reach, and a ballistic release is the only way in |

No task on the board or in LIBERO, RoboCasa, SimplerEnv, BEHAVIOR or RoboDojo throws an object into a target beyond the robot's reach.
Their "put X in the basket" tasks place objects within reach.

## Importance and sources

- **Relatability.** Tossing balled-up socks into a laundry hamper is something most people do on laundry day. The hamper is a common
  $17-20 product ([Target listing](https://www.target.com/p/pop-up-mesh-laundry-hamper-green/-/A-94093520)). We cite no statistic for
  how often people toss socks; the claim is ordinary experience.
- **Why throwing matters for a robot.** Throwing lets a fixed robot put things where it can't reach. TossingBot (Zeng, Song, Lee,
  Rodriguez and Funkhouser, 2019) picked mixed objects from a bin and threw them into boxes outside its reach, learning grasp and throw
  from camera images ([arXiv:1903.11239](https://arxiv.org/abs/1903.11239)). This task asks a general-purpose model for the same
  ability, on an everyday object, with no throwing-specific training rig.
- **Limits of the evidence.** The hamper's size and mass come from the retailer listing. The sock mass is estimated from a pack's
  shipping weight, and the roll's size is our estimate.

## Files and credits

| File | What it is |
|---|---|
| `task.py` | scene, scorer (`Trial`), `score()`, `success()`, and the closed-loop demonstration `demo_script` with its failure-check modes |
| `task.json` | the task's description |
| `mechkit.py` | the mechanism and part kit from `tasks/heat-shrink-lug/` (another team's open-source kit), unchanged: free bodies built from meshes and convex colliders, `fingertips`, `on_step`/`on_reset` hooks |
| `demokit.py` | the closed-loop demo runner shared by `tasks/heat-shrink-lug/` and other tasks, unchanged |
| `simready.py` | the SimReady loader from `tasks/cutters-off-pegboard/`, unchanged apart from its docstring |

SimReady props (NVIDIA SimReady assets, [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned
to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0), downloaded at first run; masses set at load time
(towel 100 g, notebook 120 g, mug 300 g), each loaded as one rigid body:

- [`consumable_microfiber_towel_folded_small_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_microfiber_towel_folded_small_b01)
- [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)

The socks and the hamper are not in SimReady (it has no laundry hamper or socks), so they are built from meshes and convex colliders
with `mechkit.py`, at the real sizes and masses above.

## Simplifications

- Each rolled pair is one rigid flattened ellipsoid with one convex collider, friction 0.9/0.8 and restitution 0. A real sock ball
  squashes in the grip and lands dead; the model lands dead but doesn't squash.
- The hamper's wall is 24 rigid panels. Their colliders are 3 cm thick, all outside the wall's inner face, and the floor collider is
  2.5 cm thick (so the inside floor is 2.5 cm higher than a fabric floor): thinner ones let a sock arriving at 4-5 m/s pass through
  in one physics step on the GPU. The real mesh gives a little and the frame flexes, which would soak up more of a pair's
  impact than the model does. The hamper is a free 400 g body standing on the floor, so a hit can shift or tip it.
- No air drag. For a 40 g ball about 7 cm across at 2.2 m/s over 1 m, drag shortens the throw by a few centimetres at most.

## Validation

Scripted demonstration (`--mode full`, recorded on the GPU, score from the harness's `task_score`):

| Run | Score | Control steps | Where the pairs came to rest (x, y in cm; hamper centre 100, -87) |
|---|---:|---:|---|
| demo 1 (GPU 3) | 1.0 | 466 (23.3 s) | white 115, -95; grey 99, -92; black 108, -98 |
| demo 2 (GPU 2) | 1.0 | 466 (23.3 s) | the same |

Each pair comes to rest on the hamper's floor. The pairs land the same in
both runs because the layout and the script are fixed.

Failure checks, all on the final `task.py`, physics on the CPU (`--fast`, same physics settings; the demo also scored 1.0 this way, at
469 steps):

| Mode | What it does | Score | End state |
|---|---|---:|---|
| none | nothing | 0.0 | all three pairs where they started |
| drop | carries each pair as far out as the arm reaches (x 48 cm, 15 cm over the table) and lets go: the cheapest "put it as close as possible" | 0.0 | all three on the table, 45-59 cm out |
| weak | the demo's throw with the sweep at half speed (careless lob) | 0.0 | one on the table, two on the floor short of the hamper |
| early | opens the gripper on step 2 of the swing instead of 4 | 0.0 | two on the table, one on the floor beside it |
| late | opens on step 7 | 0.0 | all three flung down onto the floor short of the hamper |
| hard | sweep targets 25% faster | 1.0 | all three in (the joints' speed limits cap the hand speed) |

A one-off variant opening on step 3 also scored 1.0, and one opening on step 5 scored 0.0 (all three on the floor, short): the release
window is two control steps, about 0.1 s wide. A one-arm check isn't meaningful here: the demo already uses one arm, which the task
allows, and the left arm has no route to the hamper either.

## Model run

Dream-1 was run once with the instruction above and a 1,200-step limit. For 656 steps it moved both arms in toward the table and
closed both grippers (by about step 240; they never opened again, so no pair was released or thrown). Then the Dream API stopped
the run with this error: "PolicyError: DreamAPI refused the inference request:
DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Throwing objects into the hamper is unsafe." The harness
recorded no score for an errored trial. With no pair released, none can be in the hamper, which is 0 under the scoring rules. The model video is the top camera's 657 recorded frames (steps 0-656).
