# Yo-yo down and back to the hand five times in a row

A classic fixed-axle yo-yo (Duncan Butterfly, 48.8 g) is wound up in a cradle between the right gripper's open
fingers. The string's slip loop is already around one fingertip. The robot has to pick the yo-yo up, raise it to
playing height and let it go. Then it has to bring the yo-yo back up to the hand five times in a row.

**Instruction:** "Yo-yo it: send the yo-yo down and bring it back up to your hand five times in a row."

**The knack.** A fixed-axle yo-yo loses about a fifth of its energy on each trip down and up. In this model, that
is the turnaround at the bottom plus the string rubbing on the halves. If it is just let go, it climbs only about
four fifths of the way back. The hand can put that energy back, but only with good timing: it has to be rising
fastest at the instant the string runs out, and then stop. In the hand's frame, accelerating upward while the
yo-yo is still unwinding acts like extra gravity on the way down. Stopping while it winds back up acts like less
gravity on the way up. The demo lifts 8 cm in 0.15 s and starts the lift when the yo-yo is 0.15 s from the
string's end. Starting the same lift after the string has run out, or 0.3 s early, brings nothing back at all
(red team below). The timing has to be repeated every 2.4 s, from what the robot sees of the falling yo-yo.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, the step hook that moves the yo-yo on its string, the scorer, and the closed-loop demo (`demo_script`) |
| `yoyo_model.py` | The yo-yo-on-a-string model: pure Python, no simulator import |
| `mechkit.py` | The mechanism kit (parts as USD text, step/reset hooks, fingertip points, held forces), unchanged |
| `demokit.py` | The closed-loop demo runner, unchanged |
| `simready.py` | Loads the SimReady distractors, downloading them once from Hugging Face, unchanged |
| `task.json` | The task's details for the board |

## Run a trial

From this folder, in the simulator's Python environment:

```
# a model's trial (any policy); the scorer is the task's own
inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 --instruction "Yo-yo it: send the yo-yo down and bring it back up to your hand five times in a row." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/yo-yo-throw-down-and-return/model

# the scripted demonstration (closed loop): writes full/demo.mp4, setup.png and result.json with task_score
python demokit.py task.py ~/yam-eval/yo-yo-throw-down-and-return/demo --mode full

# the failure checks, physics on the CPU
python demokit.py task.py ~/yam-eval/yo-yo-throw-down-and-return/redteam --fast --mode none,drop,late,early,carry,grab,chase
```

Runtime: the demo reaches full score at 17.6 s (352 steps) and the episode ends 1 s later. A final render takes
about 55 s on one GPU, and a `--fast` run about 8 s. The scene has no randomness: run with `object_jitter=0` and
any seed. The run limit is 1,200 steps (60 s), about three times the demo.

## The scene

Positions are (x forward from the arm bases, y to the left), in cm. Heights are above the table top.

- **Yo-yo:** Duncan Butterfly, the classic plastic one: fixed axle, 5.72 cm across, 3.24 cm wide, 48.8 g, red
  halves with white caps. The string is cut to 40 cm from the slip loop to the axle (see Fidelity). At the start,
  all but 6 cm of it is wound on. The slip loop is around the tip of the right gripper's first finger.
- **Cradle and riser:** a pine block 6.5 x 8 x 22.6 cm standing at (32.3, -30.5). On top are two black plastic rails,
  0.8 x 5 x 0.8 cm, 3.2 cm apart centre to centre. The yo-yo stands upright on its rim between the rails, with its
  axle along y. Its centre is 25.9 cm up, between the right gripper's open fingers at the ready pose. The riser
  takes the place of the player's other hand, holding the wound yo-yo ready at the fingers, so the string starts
  with no more than 6 cm out.
- **Distractors** (SimReady, loose, outside the arm's path; scoring ignores them): a ceramic mug (300 g) at
  (52, -44), a pair of glasses (30 g) at (58, -4) and an apple (180 g) at (46, 6).
- **Camera:** the top camera is moved to the front right, at (92, -66) and 59 cm up, looking back at the right arm.
  The yo-yo moves up and down, which a camera looking straight down would not show. The station builds the
  camera's stand from the same pose.

## How the simulator judges it

**Before release** the yo-yo is an ordinary PhysX rigid body: it rests in the cradle and can be gripped and carried.
If the hand moves more than 1 cm further from the yo-yo than the string allows, the string drags the yo-yo towards
the loop with a force of up to 3 N.

**On its string.** The yo-yo comes under the string model (`yoyo_model.py`) once it is falling, not gripped and not
resting on the table or the cradle. From then on, the step hook moves it with the model every control step. The
model uses only the measured motion of the loop fingertip (its position at 20 Hz, and from that its velocity and
acceleration), integrated in 40 sub-steps per control step:

- Rolling: (M + I/r²) s'' = M (g + a_hand) - (τ/r) sign(s'). Here s is the string out, r = 5 mm is the effective
  axle radius with the string on it, I = 0.6 M R², and τ = 1.5e-4 N m is the string rubbing on the halves. The
  yo-yo hangs below the loop as a damped pendulum of length s.
- At the string's end the yo-yo turns round and starts winding up, keeping 92% of its energy relative to the hand.
  If it arrives slower than 6 cm/s, it dies there and hangs.
- Fully wound (5.1 cm from the loop point), it meets the hand, stops, and rolls down again.
- If the hand drops faster than free fall, the string goes slack and the yo-yo flies freely. When the string goes
  taut again, the jerk sets it rolling with the momentum balance of a rolling body: s' = (v + K s'_spin)/(1 + K),
  K = I/(M r²) ≈ 19.6. A release with the string slack therefore starts nearly from rest.

**What scores.**
- A *climb* is the yo-yo winding up its string. It ends when the yo-yo stops rising or reaches the hand.
- It is a *full return* if, during the climb, the yo-yo's centre comes within 10 cm of the loop fingertip (the
  middle of that finger's tip).
- Returns are *consecutive* until a climb ends short of 10 cm, or until either gripper grips the yo-yo. Gripping
  means fingertip points of one gripper on both faces. A gripped return still counts, but the run ends there.
- The yo-yo hitting the table or the riser also ends the run.
- **Score by the longest run of consecutive full returns:** 5 or more, 1.0; 3 or 4, 0.75; 2, 0.5; 1, 0.25; none, 0.
  Full score ends the episode after 1 s.
- **Foul, 0 for the whole trial:** either gripper touches the yo-yo or its string while the yo-yo's centre is
  more than 10 cm from the loop fingertip. Touching the yo-yo means a fingertip point within 4 mm of its surface;
  touching the string means within 8 mm of it, not counting the loop finger's own tip. This rules out carrying it
  up by hand, or catching it with the other arm. The foul is latched in the hook and cleared only in mechkit's
  reset hook.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## Real trial: preparation, reset and measurement

- **Buy:** a Duncan Butterfly (classic plastic, fixed axle, about $3), a pine offcut 6.5 x 8 x 22.6 cm, and two
  strips of 8 mm square plastic or wooden trim, 5 cm long, glued on top 3.2 cm apart (centre to centre) as a cradle.
- **Prepare:** tie the slip loop as usual, put it on the gripper's fingertip, and measure 40 cm from the loop to
  the axle. Cut the excess beyond a small knot, as players do when shortening a string. Shield the gripper pads with
  tape where the string runs.
- **Reset (about 1 minute):** wind the string on by hand until 6 cm is left out, check the loop is on the same
  fingertip, and stand the yo-yo in its cradle on the riser's tape mark, axle across the gripper's opening.
  Put the distractors back on their marks. Nothing is consumed.
- **Measure:** film the hand from the side at 60 fps with a card behind it. On the card, draw a 10 cm-radius
  circle centred on the loop fingertip's playing position, and move it with the hand in review by tracking the
  fingertip. Count the climbs whose yo-yo centre enters the circle, and note any touch of the yo-yo or string
  outside it. A tape measure and the video are all that is needed.
- **Safety:** a 49 g plastic toy on a string, moving under 1.5 m/s. There are no blades, heat or liquids.

## Distinct from

| Nearest task | What it tests | Why this is different |
|---|---|---|
| #3481 cup-and-ball catch (pending) | Swings a tethered ball up into a cup once | A pendulum toss with no spin and no unwinding. Here the string rolls on an axle, and the hand's timed lift must be repeated in rhythm, five times in a row |
| #3528 Beyblade ripcord launch (pending) | One fast pull of a rack out of a launcher | A single open-loop pull, with no return and no timing against a moving object |
| #3166 tin humming top (pending) | Pumps a plunger, then lets go | Builds spin by repeated pushes on a fixed toy. No string, and nothing comes back to the hand |
| #949 regulate a pendulum (rejected) | Times a pendulum and moves its bob | Measures a period. Here the timing is the manipulation itself |

There is no yo-yo task on the board. We checked the board's titles (all PRs), every folder under `tasks/`, and
LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench and RoboDojo. One risk remains: the embodied-olympics-fun
task list is not published, so we could not check it.

## Importance and sources

The yo-yo is one of the most familiar toys there is, and "make it come back up" is the first thing everyone tries.
Most adults have done it, and a 10 s clip makes it obvious whether it worked. For a robot, the skill is closed-loop
dynamic timing against an object it does not hold. It has to watch the falling yo-yo, predict when the string will
run out, and put a short, sharp lift into a window of about 0.05 s. Then it has to do the same again, every 2.4 s,
starting each time from a different height. Pick-and-place, latches, held controls and single launches don't test
this.

- Product (Duncan Yo-Yo Butterfly: fixed axle, 48.8 g, $3.00): https://www.target.com/p/duncan-yo-yo-butterfly/-/A-95227518
- String fitting: "You usually want it from your belly button to the ground", then "cut off the excess", with an
  adjustable slip loop on the finger: https://yoyoexpert.com/pages/003-basic-yo-yo-string

The 57.2 mm diameter and 32.4 mm width are the figures retailers list for the classic plastic Butterfly. Treat them
as nominal; units vary by a millimetre or so between production runs.

## Fidelity

- **Reduced model on the string.** PhysX has no string that winds on an axle, so the yo-yo is moved by the model
  above once it is on its string. The model is driven by the arm's real, measured fingertip motion. Its spin is
  integrated but not drawn: at 20 frames per second, 1,000+ rpm would only alias.
- **Estimated values.** The inertia (0.6 M R²), the 5 mm effective axle radius, the rub torque and the 92% energy
  kept per turnaround are estimates. Together they give about a fifth of the energy lost per round trip, consistent
  with a fixed-axle yo-yo coming most of the way back on its own.
- **Not modelled.** The string's pull on the arm is not applied back (under 1.5 N on an arm that carries 0.5 kg).
  The string is not a collider: touches are judged from fingertip distance. A gripper closing on the yo-yo hands
  it back to PhysX.
- **String length.** The string is cut to 40 cm, so the yo-yo bottoms out 8 cm above the table from a hand 51 cm up.
  The usual belly-button-to-floor length would need the hand about 1 m above the floor; players cut strings to
  their height in exactly this way. A shorter string makes the timing faster, not easier.
- **Primitives and stand-ins.** The riser and cradle are primitives. The yo-yo is a custom mesh: SimReady has no
  yo-yo. The distractors are SimReady.

## Assets and licences

- SimReady props from [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to
  revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0. Masses are set at load time:
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) (mug, 300 g)
  - [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01) (glasses, 30 g)
  - [`food_produce_apple_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_apple_b01) (apple, 180 g, convex collider)
- `mechkit.py` is the shared mechanism kit from `tasks/heat-shrink-lug/` in this repository (another team's
  open-source kit), unchanged. `simready.py` is the SimReady loader from `tasks/cutters-off-pegboard/`, unchanged.
  `demokit.py` is the closed-loop demo runner shared by `tasks/heat-shrink-lug/` and other tasks, unchanged.

## Validation

- Scripted demonstration (`--mode full`, final render on the GPU): **1.0** in two separate runs. Returns ended at
  8.0, 10.4, 12.8, 15.2 and 17.6 s, each within 5.2-6.7 cm of the loop fingertip.
- Failure checks (`--fast`):

| Mode | What it does | Score |
|---|---|---|
| `none` | Nothing | 0.0 |
| `drop` | Picks it up, lets go, never tugs | 0.0 (climbs stop 12, 18, 22 ... cm from the hand) |
| `late` | The same lift, started once the yo-yo is seen climbing | 0.0 |
| `early` | The same lift, started 0.45 s before the string runs out | 0.0 |
| `carry` | Holds the yo-yo and moves it up and down 25 cm | 0.0 (never on its string) |
| `grab` | Two good returns, then the left gripper grabs it 15 cm below the hand | 0.0 (foul) |
| `chase` | Drops the hand 15 cm to meet each climbing yo-yo | 0.0 |

## Model run

Dream-1 was attempted once on the final version, with the instruction above and a 1,200-step limit. Its API refused
the request before the first action, so there is no model video: "DreamAPI refused the inference request: DA-4000
BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Jerking or whipping the arm repeatedly can damage
the robot." The instruction was not reworded to get past the refusal. The scripted demo (above) reaches 1.0.
