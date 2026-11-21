# Apply threadlocker around a machinery mounting bolt

A bimanual maintenance task: distribute semi-solid threadlocker around the engagement threads of a freely held mounting bolt, including the roots, while keeping its head and the finished band clean. The fixed RGB-only demonstration reaches 1.0 in 1,404 control steps (70.2 seconds).

The selected operation is application of medium-strength semi-solid threadlocker around the engagement threads before reassembly. [Henkel's instructions](https://next.henkel-adhesives.com/us/en/products/industrial-adhesives/central-pdp.html/loctite-248/BP000000153639/variation/462476.html) specify a circumferential band and material in the thread roots. Curing and subsequent tightening are outside this task's current scope. The [technical data sheet](https://datasheets.tdx.henkel.com/LOCTITE-248-en_GL.pdf) supports the wax-like contact application, but does not establish our provisional transfer coefficient.

The operation also appears in [Henkel's professional automotive maintenance guide, page 5](https://dm.henkel-dam.com/is/content/henkel/auto-maintenance-do-it-right.pdf), which includes semi-solid stick application to engagement-thread roots. This is a general fastener-preparation operation used in vehicle and machinery maintenance.

No direct threadlocker task was found in the 4105-task board snapshot checked on 2026-11-20. Accepted thread-seal tape, grease-gun application and painting tasks remain relevant overlap checks; novelty is not guaranteed by the material name.

The bolt is actual-size M16 x 80 mm, 2 mm pitch, 24 mm head flats and 10 mm head height. The thread uses 60-degree flanks, a truncated crest and root. The active first 16 mm has separate convex helical contact segments; uncoated thread regions use a cylindrical collision envelope under the same rendered thread. The wax plug is 10 x 10 x 8 mm in a 80 x 24 x 24 mm rectangular carrier, so the handling geometry is reproducible instead of depending on unmeasured commercial packaging. Its 8 mm projection provides carrier clearance during application. A fresh plug would be cut from semi-solid material during reset. The carrier is an explicitly dimensioned tool adaptation. Its material-transfer parameters are simulation approximations; no physical hardware calibration is claimed.

Only native wax-to-bolt contacts with tangential sliding can transfer material. Source volume decreases by exactly deposited volume. The local smear kernel extends at most one thread pitch axially and 12 degrees to either side of a contact; it cannot paint remote regions. The film is rendered on the thread, and the evaluator separately checks root coverage. A positional proximity event alone cannot create coating. Material chemistry, cured bond strength and instant curing are not modeled.

Run with the pinned environment in `DEPENDENCIES.json`: set `ISAAC_PYTHON`, `INSPECT_OVERLAY`, `ARENA_ROOT`; then `bash run_inspect.sh --policy hold --steps 200 --output RUN_SMOKE --check-reset`. Run CPU checks with `python -m unittest test_coating test_scorer test_dream_budget`, and independently replay scores and verify recorded policy frames with `python audit_run.py RUN_SMOKE`.

Local validation includes a 200-step reset smoke, a separate 200-step bimanual pickup rehearsal, contact/noncontact material controls and 13 CPU tests. The genuine Dream-1 beta trial scored 0.0 after 1,200 steps (60 seconds simulated, 439 seconds wall time including startup). It completed successfully with RGB and robot joint/gripper readings; 25 successful action chunks required 49 adapter attempts, including 24 recovered session expirations. No coating was deposited during this trial. The fixed robot demo reaches 99.38 percent band coverage and 97.42 percent root coverage, with no invalid contact and a ten-step full-score dwell. Independent replay reproduces every recorded grade. Rebuilding the scene restores identical object states, robot joints and initial camera bytes, clears the coating and scoring history, and passes a fresh 20-step hold. Seed 0 and object jitter 0 are used throughout.

`demo.py` uses a fixed offline-IK action plan and accepts only the three RGB streams. It does not query object state during execution. The evaluator is separate and reads physical state to implement the visible-outcome rubric. No virtual grasp attachment, live pose controller or privileged demonstration observation is used in a robot trial.

The recorded native and policy-delivered images were checked at pickup, application on both sides, and withdrawal. The right wrist camera resolves the blue material and thread roots during application at 320 x 176; the overhead view is partly occluded by the arms, with the second wrist providing context. Final close physical inspection uses the procedure below. The coverage field drives both visible thread color and scoring; an independent physical judging trial has not been performed.

The external **YAM robot USD/URDF geometry and materials are MIT-licensed**, supplied through the [YAM Isaac Lab-Arena adapter](https://github.com/kairosiann/yam-isaac-lab-arena) at `0059d02a1a91bc6834c1d0846022891df0c78730`. Retain `LICENSE.i2rt`; that asset is not redistributed here. Task code and procedural geometry use the included MIT license.

The finished bolt may remain held by its clean head. The applicator must be at least 5 mm clear of the bolt and the fingers at least 5 mm clear of its coated band. No extra placement maneuver is required. The full-score dwell begins after the visible object motion is within 2 mm and 2 degrees over ten observations.

## Reproducible scene

Coordinates below are metres in the shared workbench frame; sizes are millimetres. Every loose object starts with identity orientation and zero velocity. Geometry is procedural, with no external task CAD.

| Part | Dimensions/material | Fixed starting layout |
| --- | --- | --- |
| Workbench | 1100 x 1300 x 60, wood worktop | Centre (0.50, 0, 0.47), top at 0.50; fixed |
| Two robot pedestals | 150 x 150 x 750, rigid metal | Centres (0, +/-0.305, 0.375); fixed |
| Bolt | M16 x 80, 2 mm pitch, 24 mm hex flats, 10 mm head, steel; 146.45 g | Tip (0.320, 0.150, 0.504), shaft upward, free |
| Starting nest | 8.7 mm bore radius, 18 mm outer radius, 14 mm high; rigid metal | Around bolt tip, z 0.502 to 0.516; 40 x 40 x 4 base, fixed |
| Applicator carrier | 80 x 24 x 24 rigid ABS plastic body, 50 g | Centre (0.280, -0.200, 0.540), length along X; free |
| Fresh wax | 8 x 10 x 10, density 1030 kg/m3, blue | Centre 44 mm along carrier X; 8 mm projection beyond its face |
| Two carrier rests | 12 x 60 x 28, rigid metal | Centres (0.250, -0.200, 0.514) and (0.300, -0.200, 0.514); fixed |
| Ground | 1600 x 1500 x 60, rigid concrete | Centre (0.50, 0, -0.03); fixed |

Robot bases are (0, +/-0.305, 0.75). Initial fingertip centres are (0.16, +/-0.35, 0.95), pointing down with 75 mm openings. The exact joint reset is computed from the pinned YAM kinematics and recorded with each run. Finger force is capped at 40 N per finger and the native position limits enforce a 75 mm opening. The workspace bounds are X [-0.15, 1.00], Y [-0.65, 0.65], Z [0.490, 1.40].

Only the overhead and two wrist RGB cameras are policy-visible. Native frames are 640 x 360; delivered frames are 320 x 176 at 20 Hz. The overhead camera is at (0.40, 0, 1.15), aimed at (0.36, 0, 0.65); wrist mounts use the pinned standard YAM configuration. Dream also receives robot joint and gripper readings, but no object poses, contacts, coating buffers or scoring state. The fixed demonstration accepts RGB only.

## Physical rubric and approximation

The target band lies 4 to 10 mm from the tip. The two coverage fractions are the visibly blue fraction of that band and the visibly blue fraction of its thread-root surfaces. The lower fraction determines partial credit: less than one-third scores 0, at least one-third scores 0.25, and at least two-thirds scores 0.5. A full score requires both fractions at least 95 percent, the applicator at least 5 mm clear of the bolt and the fingers at least 5 mm clear of the target band. Holding the clean head is allowed. The conditions must last ten 20 Hz steps after the parts have settled: at most 2 mm motion for all loose parts and at most 2 degrees rotation for bolt and carrier over ten observations.

Rubbing wax on the head, touching visibly coated target threads with a finger/support/carrier, leaving the workspace or damaging the fixed setup permanently scores zero. The head-rubbing and wet-contact rules are checked from the video in a physical trial. The fixed bench has no movable chassis to tip; changing the bolt's orientation is allowed. Invalid or nonfinite simulator evidence is rejected separately.

Use close photographs through a complete turn, holding the head, and a millimetre scale to judge the band and roots. The in-hand wrist views show application and contact; a physical judge can inspect the complete circumference with close photographs and a millimetre scale. No physical judging trial is claimed. No tightening, curing or achieved locking strength is part of this score.

The reduced transfer law uses positive native normal contact impulse and sliding distance, a finite material reservoir and a local smear kernel. The transfer coefficient is 5e-6 m3/(N m); visible film is 25 micrometres, capped at 150 micrometres. These are specified approximation parameters, not measured LOCTITE performance. Source-volume conservation is checked; the very small transferred mass does not change rigid-body masses or collider shapes. Wax compliance is a 2 mm prismatic travel with a 1000 N/m spring, 1 N s/m damping and 3 N drive cap. Native surface friction is 0.35 static / 0.30 dynamic; rubber-like finger pads use 1.2 / 1.0. There are no virtual grasp attachments or scripted object pose changes in a policy trial.

## Run and inspect

Use the pinned environment and asset revisions in `DEPENDENCIES.json` (Isaac Sim 6.2, Isaac Lab 47.1, Inspect Robots 0.60, Python 3.12). All reported local runs use seed 0 and object jitter 0. The declared limit is 1600 control steps, 80 seconds of simulated time. Cold startup and frame recording make wall time longer; the final 1,404-step demo took 674 seconds wall time including startup, recording and the post-run reset check on the local DGX.

Start from the Isaac Lab / Isaac Sim versions in `DEPENDENCIES.json`. Install the pinned YAM adapter (including its robot assets), Inspect Robots, Inspect YAM and Dream adapter into an isolated Python package directory. Set `ISAAC_PYTHON` to the Isaac Lab environment interpreter, `INSPECT_OVERLAY` to that package directory, and `ARENA_ROOT` to a separate checkout at the listed Arena revision. Apply the included compatibility patch to that isolated checkout before running; it updates old actuator/camera API names and does not change their numeric parameters. Preserve the dependency versions in `DEPENDENCIES.json` rather than upgrading the simulator environment. The official [YAM adapter setup guide](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730) describes asset and Inspect installation; this task's tested newer simulator versions require the included compatibility patch.

```bash
"$ISAAC_PYTHON" prepare_arena_compat.py "$ARENA_ROOT"
python -m unittest test_coating test_scorer test_dream_budget
bash run_inspect.sh --policy hold --steps 200 --check-reset --output RUN_SMOKE
bash run_inspect.sh --policy demo --steps 1420 --check-reset --output RUN_DEMO
python audit_run.py RUN_DEMO --require-full
# Only after local full-score validation; credentials stay in the environment:
bash run_inspect.sh --policy dream --steps 1200 --budget-ledger DREAM_LEDGER.jsonl --output RUN_MODEL
inspect-robots video RUN_MODEL/inspect/LOG.json --out VIDEO_OUTPUT --fps 20
python export_media.py RUN_DEMO/evidence/episode-00 DEMO_MEDIA --name demo
```

The runner always stores all three native and delivered image streams, joint targets, physical evidence, the reported grade and an executable source snapshot. `audit_run.py` replays every grade and checks frame equality against native-image resizing. `--check-reset` rebuilds the world, verifies equal pose/velocity/joint/camera reset records, zero material and history, and performs a fresh 20-step hold. `calibrate_transfer.py` is a separately labelled virtual-fixture development test and is never a robot demonstration.
