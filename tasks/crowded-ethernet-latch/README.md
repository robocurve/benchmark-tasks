# Service a crowded 4x2 Ethernet panel

**Incomplete feedback draft:** see [DRAFT_STATUS.md](DRAFT_STATUS.md) for current measured evidence and missing work. No full-score demo or competition-ready submission is claimed.

Native mechanics and robot controller under development; no completed task demo yet.

2026-11-16 CPU candidate: keep the same gripper, wrist orientation, 2.3 mm
physical stroke, PI-D gains and 0.2 mm clearance guard; move the intended tab
contact 0.5 mm rearward (plug-local X -10.5 mm). Final clip031 entered the
contact stroke at26.8s and stopped at37.2s when actual tip_left/housing
side_0_m clearance fell to0.1872mm. This target shift improves that recorded
pose's gap to0.6788mm; a41-point nominal full-stroke sweep and translated
intervals remain above0.95mm with continuous IK below1um. These are scoped
CPU geometry checks, not native contact/release proof. All121 CPU tests pass.

First native checkpoint (unscored, stops after measured latch release):

```sh
/data/IsaacLab-Arena/.venv/bin/python run.py --mode clip-press-probe --no-video --out /home/ubuntu/yam-eval/ethernet-clip032
```

Only after that succeeds, run `--mode release-probe` from a fresh standard
reset to attempt the full source-to-target cycle. Camera-free engineering
capture is not native RGB media.

Unlatch the marked RJ45 plug from its current port without breaking the tab,
then insert it into the marked empty port and leave its latch engaged. Six
other connected cables crowd the eight-port, two-row/four-column panel. Their
plugs must remain seated. A second arm may move a flexible neighbouring cable
aside to reach the clip; no added handles or enlarged port pitch.

Native geometry must reproduce a plug shell, flexible spring-return latch,
retention hook and jack ledge. Pulling an engaged plug must physically resist;
pressing the tab must clear that ledge. Insertion must deflect and re-engage a
normal latch through contact, rather than toggle a boolean when a plug crosses
a distance threshold. Actual plug/cable orientation and clipping clearance must
come from a selected connector drawing before the dimensions are frozen.

Proposed checks: source-port clip depression before withdrawal; target plug
alignment and seating; clip re-extension into the target retention recess;
released retention, neighbouring connections undisturbed, tab intact, cable
not severed or overstrained. All measures need native pose/contact readback.
The controller attempts clip release and transfer, but current trials score 0.0: the plug has not been successfully disconnected and retained in the target. No successful demo or media ID is claimed.

DataCenterBench already joins an Ethernet plug and coupler and its discarded
candidate list mentions network-cable latch release. Real release/retention
mechanics and bimanual cable obstruction are the intended distinctions, not
merely eight connectors instead of one. See the batch record for sources.

## Current native tests

`run.py` builds actual open sockets, passive spring clips with retaining hooks,
and jointed cable chains. Connector geometry is explicitly approximate and
needs calibration against a selected product; it is not licensed vendor CAD.
The switch is fixed on a stand; plugs and cables have real rigid-body dynamics.

```bash
/data/IsaacLab-Arena/.venv/bin/python run.py --mode fit-probe --out /path/to/new-fit
/data/IsaacLab-Arena/.venv/bin/python run.py --mode retention-probe --out /path/to/new-run
/data/IsaacLab-Arena/.venv/bin/python run.py --mode clip-constraint-probe --no-video --out /path/to/new-constraint-check
/data/IsaacLab-Arena/.venv/bin/python run.py --mode press-tracking-probe --no-video --out /path/to/new-tracking-check
/data/IsaacLab-Arena/.venv/bin/python run.py --mode release-probe --out /path/to/new-attempt
```

Retention probes apply external bench forces and cannot receive task credit.
The separate clip-constraint probe retains the 1 N engaged pull and pressed
withdrawal sequence, applying 0.5 N along the plug's downward Z axis at the
actual tab point (-10, -1.5, +0.5) mm, including its moment about the clip centre.
It checks all seven clips for relative rotation below 0.5 degrees, transverse
joint error below 0.1 mm and slider limits within a stated 10 micrometre numerical
tolerance. Plug and clip bodies now explicitly request 64 position and 16 velocity
solver iterations to resolve the slider's locked axes. The static off-centre
load check passed (rotation at most 0.0246 degrees, transverse error 1.3 micrometres,
travel 1.974–2.008 mm). The strict engaged-pull margin and the uncontrolled
free-withdrawal/table-impact check failed; this is not a comprehensive constraint
pass. Masses, inferred geometric inertias, spring force,
colliders and 2.4 mm travel remain unchanged. Its external loads are diagnostic
only and never run in robot/scored mode.
The isolated press-tracking probe leaves the left hand at READY, uses the same
native switch, cables and robot, and stops after the right-hand prepress stage
before clip contact. It uses no external test forces or object pose writes and
has no task score or eligible submission media. Its 0.291 mm mesh-error target
comes from the remaining rigid-clearance reserve in loaded trial 022; passing
this diagnostic still requires a full loaded release/insertion run.
Release attempts use bounded normal robot joint/gripper commands and privileged
native telemetry, never object pose writes or software latch-release toggles.
Actual source release, target alignment, clip return, six neighbours and a
2-second released retention hold determine scoring. Controller phase names do
not establish success. A failed physical precondition stops the attempt.

Every output freezes its Python sources and contact-point JSON. Require a real
`result.json` and no `error.json`; Kit shutdown can swallow an exception and
return exit code 0. Videos and traces are diagnostics unless final scoring and
visual review establish full completion. See `VALIDATION.json` for evidence.

### Physical-layout reference correction

The early 28 mm prototype pitch was only a mechanics scaffold and is superseded.
Horizontal pitch is now 16.26 mm, with a 65.91 x 27.30 mm 2x4 housing envelope
referenced to the public Amphenol RJE7K drawing. Vertical pitch remains a
13.72 mm procedural approximation pending visual drawing verification. The
retaining ledge, flexible clip and unbooted plug details are original approximate
geometry, not vendor CAD or a standards certification. The source/target are
the lower/upper ports of the outside column; six other cables remain connected.
Source: https://cdn.amphenol-cs.com/media/wysiwyg/files/drawing/p-rje7k-888-1xx0.pdf


The current controller uses the left hand on the ordinary boot and the right
hand's unmodified left finger on the clip. The current grasp TCP is 38.0 mm behind, 3.5 mm toward the outside and
0.24 mm above the measured plug origin; the ordinary finger tips reach the rear boot.
Position and approach axis follow the actual settled plug pose while the left
wrist keeps a fixed world orientation. It narrows the jaws to a 13 mm aperture while 40 mm in front,
then inserts and force-closes. Pressing begins after measured pose/jaw gates;
actual clip travel must exceed 1.8 mm before withdrawal. Pulling pauses if the
clip returns and pressing follows the actual moving plug until the measured
hook front clears the retaining ledge by 0.5 mm. The pressing finger then
rises 20 mm before the boot finishes withdrawing; this avoids dragging its
spare jaw through a neighbouring socket roof. Physical plug-tip
clearance, measured grasp slip, target seating and clip return gate later
phases. Both hands retreat before the retention rest.

The trace records both measured finger joints, TCP tracking and the actual
pressing support point. The right arm settles at hover, then uses bounded joint
PI-D during a 2 mm prepress move and the complete contact stroke. A measured
mesh-clearance governor advances the unbiased reference only as tracking slack
allows; its path clock pauses independently of elapsed time. Contact support-point
reference speed is at most 0.7 mm/s, and maximum press depth remains 2.3 mm.
Inner actuator gains and friction are unchanged. The left hand preserves its
bounded learned compensation while carrying the connector. The adaptive engineering probe permits up to 2,400 actions / 120 s;
it stops on a measured failure or completed retention rest. Nominal CPU mesh
and IK checks do not establish full-arm/cable clearance or native success.

Measured native pose regressions include 8–11.6 degree source pitch under
cable load. They check full gripper surfaces against sockets, loaded neighbour
plugs, the service plug shell and its own clip tab. The earlier nominal boot
certificate omitted that tab; moving the grasp TCP 3 mm rearward restores
clearance while retaining contact on the ordinary boot. These static numeric
certificates do not establish native grip stability or full cable/arm clearance.


The current socket-fit candidate adds two ordinary internal longitudinal
shoulder guides per port: X = 399.5–425.5 mm, Y = port centre ±4.75 mm,
width 2.5 mm, and Z = port centre +4.3–6.8 mm. The nominal plug shoulder
top is +4.0 mm, leaving 0.3 mm vertical clearance. The 7 mm centre channel
clears the 5 mm latch. These procedural dimensions are a physical fit repair;
they are not vendor-validated CAD, and native fit has not yet passed.

`fit-probe` applies no external force and holds both arms at READY for 6 s.
Its final 1 s reports every one of the seven connected plugs: maximum absolute
pitch must be ≤3.5°, and absolute released clip travel ≤0.2 mm. The separate
9 s `retention-probe` records the same fit before its 1 N pull/depression
bench loads; those forces are never called in a robot/scored run. Scoring
limits, masses and port pitch are unchanged.

Every future trace includes a post-step snapshot of all plug, clip and 84
wire-body poses, plus actual joint positions, finger positions, TCP rotation
and robot root pose for both arms. Controller targets describe the preceding
step; `native_hands` and body snapshots share the post-step timestamp. The
shared CPU object builder includes every task collider. Round boots and cable
capsules use enclosing boxes for conservative clearance certificates. Cable
axes include the capsule's authored local rotation as well as body rotation.
Full arm-link clearance and dynamic fit/retention still need native evidence.


For state-only engineering checks add `--no-video`. This disables AppLauncher
camera support and sensor/review cameras, and skips video readback/encoding.
The physical URDF wrist-camera housings, collider assets, masses, joints,
physics timestep and action rate are retained. Each run writes a
`physics-manifest.json` containing collision-mesh/native-asset hashes and a
camera-independent live body/joint/mass digest for video/no-video comparison.
Native equivalence must be checked from these manifests; state-only evidence
is not final submission media. Defaults continue to record video.

Results report monotonic setup, simulator construction, controller build,
policy, environment step, telemetry and video times, plus total wall time to
stop. Environment-step time includes internal sensor rendering in video mode.
Fit reports distinguish the unchanged task limits (axis angle <5°, lateral
error <1 mm, clip travel <0.3 mm) from the stricter engineering margins
(axis angle ≤3.5°, absolute clip travel ≤0.2 mm). Guided003 met task fit and
1 N retention/release checks; its total-axis engineering margin did not pass
all seven ports. No full robot success is established by that bench test.

Final retention readback now uses the four actual clip-body retaining-face
vertices against the ledge, including finite Y/Z overlap. Released scoring
retains the 70 mm TCP condition and additionally requires full gripper-surface
clearance of at least 5 mm from every rigid object and measured cable capsule.
Capsule clearance uses its exact support function; possible flexible brushing
during entry remains permitted and is logged rather than used as a new task
constraint.

The next grasp candidate shifts the TCP another 10 mm rearward and 4 mm
outward after actual native wire contacts blocked the prior approach. CPU
swept-shape checks cover the measured source/neighbor cable cluster, with
at least 0.5 mm separation during entry and loaded closure. This narrow
fingertip grasp still requires the native predicted-jaw and slip gates;
its sustained physical hold has not been demonstrated.

The force-balanced grasp candidate uses a 3.04001 mm command per jaw
through holding and transfer. With predicted stops of 3.67787/3.40215 mm,
the unchanged 5000 N/m PD stiffness and 69.657 g finger masses predict
about 2.5 N opposed contact forces after gravity. This is a quasistatic
estimate, not a force-sensor measurement or native success claim. The last
1 mm closes over 2 s; bounded fixed-reference joint feedback continues
during stationary closing/holding, with non-target rigid clearance checks.
