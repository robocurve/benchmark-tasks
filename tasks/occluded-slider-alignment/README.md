# Align a concealed service slider

A fixed covered service fixture hides its black index and green target from the
original overview camera. An exposed handle remains accessible. The original wrist
cameras can reveal the marks through an inspection opening while an arm manipulates
the slider. This task concerns camera active perception and manipulation. No score
requires a particular arm assignment, camera motion or control strategy.

The benchmark does not claim that two arms are logically necessary for every safe
solution. A robot may inspect first and then manipulate, or find another legal view.
A fixed known scene also permits a privileged open-loop feasibility script. That
script demonstrates reachability, simultaneous useful wrist view and physical
completion; it is not itself an image-feedback policy or proof of model perception.

## Runtime and reproducible commands

Use the official walkthrough's **Isaac Lab-Arena / yam_arena / station** environment,
not a substitute simulator. Exact sources and packages: `runtime-lock.json`.
`ENVIRONMENT.md` records installation prerequisites and pinned setup commands.
Activate that runtime, then run these commands from this task folder:

```bash
python runtime_check.py
python run.py --mode setup --log-dir /tmp/slider-setup
python run.py --mode demo --log-dir /tmp/slider-demo
python run.py --mode demo --epochs 3 --log-dir /tmp/slider-repeat
python run.py --mode stability --log-dir /tmp/slider-stability
python run.py --mode contact-probe --log-dir /tmp/slider-contact-probe
python validate_run.py /tmp/slider-demo
python export_media.py /tmp/slider-demo /tmp/slider-media --name fullscore_demo
```

Use a NEW empty log directory for every invocation. All these commands are free.
`contact-probe` deliberately touches the virtual roof to verify rejection; never
use that diagnostic on real hardware. `stability` holds READY for the entire120 s
budget. The demo ends after14 s, including release, retreat and stationary hold.
`--epochs 3` tests resets in one simulation process and saves each episode separately.
The runner retains final JSON, raw RGB arrays, applied actions, reset images, source
hashes, effective options, contact traces and validation. CLI exit0 alone is not proof.

The formal model command below performs PAID inference. The supplied completed
trial used the full 2,400-action / 120 s budget and scored 0.0. Run a new trial only
with explicit authorization:

```bash
# Free local configuration check: no client, network or simulation.
python run.py --mode dream --config-only --log-dir /tmp/slider-dream-config
# PAID: one trial, no automatic rerun. Requires SDK credentials configured separately.
python run.py --mode dream --log-dir /tmp/slider-dream-final
python export_media.py /tmp/slider-dream-final /tmp/slider-media --name model_run
```

Dream uses `dreamlabs/dream-1-beta`, `record=false`, `dry_run=false`,32 initial hold
steps and48 action rows per request. `record=false` only disables extra account
recording: inference still sends three JPEG histories, measured14-D joint state and
the exact `task.json` instruction. It receives no task geometry or evaluation state.
The SDK forms9 frames per camera at5 Hz; the original controls play actions at20 Hz.
Server waits pause simulation. Initial history counts in the2,400-action budget.
Nominally50 requests cover the120 s budget; retries or shorter chunks can add calls.
Official per-chunk retry window is300 s. The wrapper allows1,800 s wall time plus30 s
shutdown grace for Dream,600 s for free checks. These are not monetary spending caps.

Exported review videos use a 640x576 layout: a full third-person image above the
left and right wrist images. The overview is enlarged 2x by pixel replication;
wrists retain their native pixels. All three views are complete and synchronized,
with labels outside the image areas. This only changes video presentation, not
policy observations, recorded motion or scoring. Export is local and makes no model call.

The task installs a process-local extension for a passive joint and private contact
history; the stock descriptor alone cannot express them. Use `run.py`/`launch.py`,
not a bare `inspect-robots -E task=task.py`. Upstream sources remain unchanged.
Official robot limits, clamp/delta guardrails, actuators, camera calibration and input
channels are retained. Runtime:20 Hz control,240 Hz physics,three320x176 RGB images.

## Fixed physical layout

Coordinates are world metres: x forward from the arm-base line, y left from its
midpoint, z up. The tabletop is z0.7475; arm bases are(0,+/-0.305,0.75). Both start
at original READY joint angles[0,1,1,-0.6,0,0], with grippers open. Seed0; no jitter.
`scene/layout.json` enumerates EVERY solid with exact position, size and colour;
`slider_task/layout.py` is its canonical source. All task geometry is authored
locally at runtime; no separate mesh or diagnostic USD export is required.

| Part | Dimensions x/y/z, mm | Position / specification |
|---|---|---|
| Base |250/200/12 |centre(0.25,0,0.7535), anchored |
| Two rails |150/8/8 each |x0.25,y+/-0.031,z0.7675 |
| Home/end stops |8/50/24 each |x0.185/0.315,y0,z0.7875 |
| Orange carriage |24/50/24 |home centre(0.205,0,0.7875) |
| Attached orange handle |28/130/34 |local(0,-0.087,+0.005) from carriage |
| Two end walls |8/180/108 each |x0.139/0.361,y0,z0.8135 |
| Opaque roof |230/180/8 |centre(0.25,0,0.8715) |
| Front sill |214/8/12 |centre(0.25,-0.086,0.7655) |
| Front header |214/8/32 |centre(0.25,-0.086,0.8515) |
| White scale backing |150/3/18 |centre(0.25,0.026,0.8195) |
| Green target |12/1/18 |centre(0.270,0.0281,0.8195) |
| Black moving index |8/1/22 |local(0,+0.0256,0) from carriage |

Rigid matte plastic housing/handle and an ideal linear-bearing surrogate. Total
moving mass80 g, including handle/index. Surface friction0.20 static/0.15 dynamic,
zero restitution. Guide moves passively along x from0 to90 mm; no joint drive or
object teleporting occurs during rollout. Target lies65 mm from home, distinct from
the hard stop. This number is setup/evaluation data, never added to the model prompt.

The -y operating opening is214 mm wide and64 mm tall,24–88 mm above the tabletop.
The +y inspection opening is214 mm wide and108 mm tall above the base; internal
rails/marks occupy part of it. Cover and rails are anchored. Real reconstruction:
fix rigid parts to a marked board/table, use an unpowered low-friction guide and
coloured strips of the stated sizes. Match the supplied setup screenshot and inventory.
Dimensions specify an original fixture analogue, not a claimed commercial product.

## Scoring: visual-contact-v2

Read the final state, retaining disqualifying events over the whole trial. No highest
intermediate partial score is retained. Horizontal error is the difference between
black-index and green-target centre x coordinates. Apply these bands in order:

| Outcome, provided no disqualifying event occurred | Score |
|---|---:|
| Slider moving faster than5 mm/s |0 |
| Error<=8 mm, no robot-slider contact and speed<=5 mm/s, all held for1 continuous second |1.00 |
| Error<=15 mm without the completed full-credit hold |0.75 |
| Error<=25 mm |0.50 |
| Forward displacement from home>=15 mm |0.25 |
| Anything else |0 |

Any of these latches score0 until reset: fixture moves>2 mm; carriage leaves the
axis by>2 mm or rotates>2 degrees; travel outside[-1,91] mm; prohibited contact
persists continuously for0.100 s or longer. Allowed robot contact is a finger on the
exposed orange handle. Robot contact with the other slider surfaces, fixed fixture
parts (including rails/stops/marks), or the other arm is prohibited. Intermittent
brief contacts do not accumulate across separation. No force threshold enters the
score. Missing/nonfinite telemetry invalidates evaluation, not a legitimate model0.

For real scoring, use a millimetre ruler, marked fixture outlines, a square/protractor,
and a timestamped video of the opening and robot/fixture contacts. Read index centres
from the side, check stationary release over1 s and measure displacement/rotation
relative to setup marks. A contact lasting at least100 ms fails regardless of force;
use timestamps rather than guessing from a single image. Frame-boundary cases need
higher-rate video (120–240 fps) for timing. No force sensor is required. Real-world
setup should be unpowered with the normal YAM operational limits and supervision.

The simulator records normal force and penetration for diagnosis ONLY. Contact is
reported at240 Hz through PhysX tensors. Task contact offset is1 mm; solver/contact
geometry approximates physical surface touching. The100 ms tolerance avoids scoring
single-step contact noise. All18 robot bodies are monitored against the fixed fixture,
slider and opposite arm; compound contact points distinguish handle from carriage.
Original disabled intra-arm self-collision is retained; camera stand/table impacts
are not claimed as fully monitored. This is not a whole-cell safety certification.

## Reset, evidence and limitations

Reset restores slider home/zero velocity and original READY/open arms, then clears
safety/contact/hold history. All source parameters are fixed. The repeated-demo
validation checks restored poses and success in each of three episodes. The supplied full-score demo achieved 1.0 in 280 actions (14 s);
three consecutive resets each achieved 1.0 with final error below 0.625 mm.

`slider_task/oracle.py` provides a privileged open-loop full-score feasibility demo.
Its observation-arm posture and handle trajectory were chosen using scene geometry.
It does not read images for control. Actual rendered views show what an image-based
policy could see; the feasibility script must not be called Dream or visual feedback.
The supplied formal Dream-1 run scored 0.0 after all 2,400 actions (120 s), with
50 returned action chunks and no SDK retries. Wall time was 440.34 s. The slider
did not move; final alignment error was 65 mm. Right fingers contacted the roof,
triggering the prohibited-contact latch at action 433 (21.65 s). The longest
reported continuous prohibited contact was 3.4625 s. No robot-slider contact was
reported. All evaluation steps were instrumented and all three camera streams
contain 2,401 original frames. This is a valid failed task rollout, not an API or
simulation failure. One rollout does not isolate the cause of policy failure.
The video includes the initial frame, so its display duration is 120.05 s at 20 fps;
the full-score demonstration is 14.05 s including its initial frame.

Rigid bodies omit compliance, bearing drag, wear, damage and manufacturing variation.
A privileged fixed-coordinate policy can solve a fixed benchmark scene. Scoring
therefore measures the task outcome and accepts other safe strategies; camera usage
and role allocation are research observations, not reward requirements.
