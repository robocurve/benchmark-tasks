# Cut the bottom tape, unfold the flaps and flatten a used carton

**Incomplete feedback draft:** see [DRAFT_STATUS.md](DRAFT_STATUS.md) for current measured evidence and missing work. No full-score demo or competition-ready submission is claimed.

Native mechanics/controller draft.63CPU checks pass, including hinge geometry, scorer outcomes, tool-frame feedback and cut handoff/watchdogs. Camera-free native capture and source-bound pose/scorer audits pass. Fresh sealed cut011 completed all158 cuts and cutter parking at score0.25; physical opening/flattening and a full demonstration or Dream score remain incomplete.

Start with an **empty used carton**, bottom-up with its four top flaps folded back and
its four bottom flaps closed and taped. Use a guarded cutter to
cut the long bottom seam and the tape across both end folds, park the cutter,
unfold **all four bottom flaps**, then collapse the four-wall tube and leave
the complete carton flat on the table. The carton has no grasp handles.
This is a recycling task after contents have been removed, not another item
retrieval submission. It requires the bottom preparation absent from #1858.

## Geometry and mechanics

Carton: 220 × 180 × 100 mm, 3 mm corrugated board, four 35 g side panels and
eight 18 g flaps. Short flaps are inset one board thickness to fit beneath the
long pair. Four vertical crease hinges form a free closed loop; eight flap
hinges retain every flap. The box is not fixed to the table. `scene.py` exports
this geometry as native USD rigid bodies and revolute joints. Layout tests
check that both local anchors of every hinge coincide initially.

Packing tape is 48 mm wide, across the full bottom seam with 40 mm tails down
both end walls. Three cut paths are sampled at 2 mm resolution. A held blade
must sweep each cell within 1.1 mm, with valid depth/orientation and sampled
motion below 15 mm per control step. Merely positioning the cutter at the end
of the seam earns nothing. The sealed outer bottom flaps may not be forced
past 12 degrees. Future runtime releases native hinge limits only after every
path is cut; it must never set flap poses to simulate opening.

The bottom cutter and marked pad use an ordinary guarded tool, now integrated
with native geometry and measured robot feedback. Cutter return and released parking passed in sealed-start cut011; this is only
the cutting/parking milestone. The fixed starting layout is bottom-up; no turnover demonstration is claimed.

## Proposed score

- 0.25: every seam/end-fold cell cut validly.
- 0.50: all four bottom flaps physically opened to at least 150 degrees.
- 1.00: all eight flaps remain at least 150 degrees open, every cardboard
  vertex lies within a 24 mm horizontal slab resting on the table (minimum
  height −2 to +8 mm, maximum height 30 mm), carton speed below 10 mm/s,
  cutter parked and hands clear for one second.

Any crease separation over 4 mm, forced sealed outer flap, nonfinite state,
missing control samples or carton drop more than 30 mm below the tabletop
invalidates the trial. Breaking panels off cannot earn credit. Final pose
orientation and choice of collapse direction are unrestricted.

The 24 mm slab is a provisional engineering tolerance for multiple 3 mm
layers, not demonstrated feasibility. Numeric limits must be checked against
native contacts and a real carton before submission. A full-task allowance is not yet validated. The current cut-only diagnostic
allows4,800 controls (240 seconds at20Hz); set the complete task limit from
a successful cutting/opening/flattening demonstration.

## Checks and remaining work

```sh
python3 -m unittest discover -s tasks/break-down-taped-carton -p 'test_*.py' -v
# In the pinned Isaac environment, export only (does not prove physics):
python scene.py /tmp/recycling-carton.usda
```

Next: native closed-loop stability, finite-thickness folding/corner clearance,
free-carton turnover, cutter contacts and live pose readback, tape rendering
and fracture coupling, a physical four-flap opening demo, collapse/release
tests, then a matching Dream trial. `run.py` supplies the scorer from actual native geometry, blade pose and
measured hand state. Controllers cannot set scorer inputs or tape coverage.

## Native development probes

```bash
/data/IsaacLab-Arena/.venv/bin/python run.py --mode dynamics --out /path/to/new-probe
/data/IsaacLab-Arena/.venv/bin/python run.py --mode grip-probe --out /path/to/new-attempt
/data/IsaacLab-Arena/.venv/bin/python run.py --mode cut-probe --no-video --out /path/to/new-state-only-attempt
```

`--no-video` is optional for all existing modes. It omits robot/review camera
sensors, render requests, RGB/video writing and screenshots. Native control,
geometry, physical camera housings, forces, timestep, scoring and mode-specific
starting conditions are unchanged. It is an engineering diagnostic, with no
eligible final media. This option has CPU contract tests; native camera-free dynamics and31-owner pose capture passed a9-second smoke.
A video/state physics equivalence experiment remains unverified.

Every run exports output-only `capture-scene.usdc` after the existing reset and
`capture-manifest.json` with scene/source hashes, external asset dependencies,
31 native owner paths and parent-owner metadata. Each existing trajectory row
adds all 18 actual robot `body_link_pose_w` actor poses, the twelve measured
panel roots and measured cutter pose in world XYZ/XYZW order. No FK animation,
object pose writes, interpolation or synthetic initial sample is used. The
first row remains at 0.05 s, at 20 Hz; scorer initialization is unchanged.

The manifest records the fixed `review.py` eye/target definitions and actual
18 mm focal/aperture spawn configuration as **virtual offline view
configuration**, without claiming camera observations. The flattened scene
preserves static native appearance for later offline replay. A replay renderer,
compositor, source-bound outcome audit and final task demonstration are not
implemented by this capture change.

The carton has twelve genuinely free rigid panels and twelve physical creases.
The stability probe kept crease errors below 0.06 mm. The first robot grip
attempt remained intact but failed to lift: its fingers met the outward top
flaps before reaching the rims. Top flaps now allow a real fold-back motion to
270 degrees with a 3 mm finite crease offset outside the wall. A geometric
regression checks that the folded flap clears the outer wall; the next native
grasp test still has to establish that this fixes access.

Probe forces and `--unsealed` starting conditions are explicitly diagnostic,
with no task score. A completed task still requires robot turnover, measured
blade cuts, four bottom flaps physically opened, intact collapse and released
flatness. Every probe freezes its sources; `error.json` invalidates it even if
Kit returns exit code 0. See `VALIDATION.json` for recorded limitations.

Scoring clarification: the robot must first open all four bottom flaps through
at least150 degrees after cutting all three tape paths. This is recorded from
measured hinge angles. During subsequent flattening, the flaps return to a
wall-coplanar90/270-degree fold; requiring150-degree angles at the flat finish
would physically contradict the24mm total-cardboard slab criterion. Final
credit requires every flap within15 degrees of its wall plane, all cardboard
vertices within the slab, intact creases, parked cutter and released hands.

## Feedback cutter and flap diagnostics

The cutter controller retains the ordinary seven-second pickup prefix, then
calibrates the measured TCP/tool transform after checking native cutter height,
finger opening and hand distance. Cutting uses the rotated TCP-to-blade offset:
turning the tool also changes the required TCP translation. Separate staging,
orientation, hover, lowering, cutting and lifting phases advance only when the
actual blade follows the current command and settles at the endpoint. Tool
travel pauses above 6 mm tracking error or 0.07 rad orientation error; cutting
uses a stricter 2.5 mm tracking gate. Non-return lowering additionally requires
absolute cross-track error at most 0.65 mm relative to its following cut. Elapsed
time can cause an explicit failure; it cannot advance the cut to the next path.
The scorer still counts only actual native blade sweeps.

In the earlier cut005 trace the timed seam started while the blade was still
162 mm from its start; at 20 s it remained 129 mm behind the moving target.
The same trace later reached 0.23 mm error on the front fold. A CPU FK check
also found the old diagonal seam hover intrinsically unreachable by about
25 mm. The revised seam uses the opposite tool direction, keeping the TCP on
the reachable side, and a 20 mm blade clearance rather than that high hover.
These observations diagnose tracking/endpoint problems; they do not establish
whether native inter-arm or carton collisions occurred. That trace omitted the
actual arm joints needed to reconstruct all robot collision meshes.

The flap gripper basis puts its Y axis along the edge and its X closing axis
across it. Approach and entry now require actual pose alignment; pinching needs
a measured 2.3–15 mm finger gap at the edge. Rotation must produce at least
5 degrees of actual flap movement and keep the acquired TCP/flap offset;
measured slipping or stalled motion fails the diagnostic. No grasp weld,
object-pose write or automatic flap opening is used.

Every new trace records normalized fourteen-dimensional actions, native arm and
finger joints, and FK TCP positions/rotations for both arms. Cutter reports
include actual/commanded/endpoint blade positions, orientation error and path
progress. The cut probe has at most 4,800 controls (240 s), stopping early on
controller failure or completion of its cutting/parking attempt. Completion of
this probe is not a full carton-breakdown demonstration.

Sixty-three CPU tests pass with NumPy available:

```bash
/private/tmp/robocurve-media-env/bin/python -m unittest discover -s tasks/break-down-taped-carton -p 'test_*.py' -v
python3 -m compileall -q tasks/break-down-taped-carton
```

An earlier ideal attached-tool FK dry run reached its then-current cutting/parking phases
in 63.8 simulated seconds without an IK failure. It assumes perfect joint
tracking and rigid tool retention and includes no contacts; it is not native
physics evidence. Native grip retention, blade alignment under load, inter-arm
clearance and all four physical flap grasps remain to be tested.

Cut timing: each physical cut has a100-second hard phase cap and a20-second
watchdog since its last accepted adaptive-cursor advance. These limits permit
slow valid tracking; they do not relax blade depth, lateral/orientation gates,
scoring or hardware limits. A matching lower-to-cut handoff retains the existing
world-frame Cartesian bias once, within the same±3mm bounds.

## Latest sealed cutting baselines (not task completion)

| Run | Actual cuts: seam / front / back | End state |
|---|---|---|
| cut006 |45/110,24/24,24/24 | Cutter-return alignment timeout at78.5s; no complete cut set. |
| cut007 camera-free |53/110,0/24,1/24 | Existing phase deadline ended an advancing seam at86.45s. |
| cut008 camera-free |109/110,0/24,2/24 |20s no-cursor-advance watchdog at92.1s; return/repasses not reached. |
| cut009 camera-free |108/110,0/24,2/24 | Grasp rotated12.57deg; front orientation reached a joint limit. |
| cut010 camera-free |109/110,24/24,21/24 |154/158 cells at125.75s; boundary seam-repass safety stop before unlock/parking. |
| cut011 camera-free |110/110,24/24,24/24 |Score0.25 at179s; all cells first complete157.55s, cutter parked/released, controller complete. |

The cut007 and cut008 captures pass source/scene hashes, all31 actual rigid-owner pose streams,
and exact captured-scorer replay (1729 and1842 samples, firstt=.05s). Creases
remained intact; scorer invalidation was null. Both scored0, and neither is
completed media. Counts from different trials cannot be combined. Full opening
exists only as an unsealed diagnostic; integrated sealed cut-to-open control and
collapse remain unfinished.

The matching lower-to-cut handoff retains its existing WORLD bias once, within
±3mm, without rotating or doubling it. A recorded007 regression preserves the
TCP target/rotation and same-seed IK result; resetting bias would jump3.72mm.
Cut008 reached109 seam cells, but its final2.925mm tracking residual exceeded the
unchanged2.5mm progress gate. Blade/inner-flap geometric contact is now under
review; do not assume that increasing force or loosening gates is appropriate.


## Current controller candidate and measured limits

The restored-start012 diagnostic completed its seam/lift/front-orientation route
in47.95s (959 native rows), with maximum grasp rotation0.044055deg and tip-offset
change47.69micrometres. Hover-to-lower issued exactly the previous14D command
on its first tick, with no integration or path advance. Its18 west-root contact
candidate rows had zero reported normal force and positive separation at least
0.2338mm; contact candidates alone are not forces. This was a synthetic,
pose-only restored start with scorer disabled, not a fresh task demonstration.

Fresh sealed cut010 then retained its pickup grasp within0.03393deg/47.15um
and passed all original tool orientations. Its2515 actual rows pass independent
source/scene/31-owner and exact geometry/scorer audits; score0,154/158 cells.
The remaining seam cell atX−109mm was1.318mm from the measured front crossing,
outside the unchanged1.1mm sweep radius. Three back-fold gaps (Y−13,−9,+1mm)
were crossed while actual blade depth was below the unchanged−1.55mm film
limit. The guard rejected the requested−113mm boundary retrace instead of
forcing the tool through the known flap-root region. No unlock or parking
milestone was reached, and counts from separate trials are never combined.

The current CPU-reviewed candidate commands both fold planes0.5mm inward,
atX±109.5mm, and fold lower/cut depth−0.75mm; hover/lift stay−20mm. First passes
and cleanup spans share those parameters, while all158 coverage cells remain
at their original coordinates, including fold rootsX±110mm. First-pass average
speeds remain12mm/s seam and16mm/s folds; all cleanup cuts use8mm/s. A numeric
counterfactual of the010 crossing windows fills158 cells if residual tracking
stays the same; this is a regression check, not native credit. Four actual010
grasps pass803 remaining-route IK samples each and all±3mm endpoint corners.
A separate273-sample nominal mesh review passes (minimum cutter gap0.750mm).
These sampled checks do not prove the live tracking envelope or future
post-unlock flap positions. Fresh native cut011 validates this candidate through cutting and released cutter
parking; it does not validate opening or collapse.

The native012 control fixes remain: matching hover-to-lower nominal pose and
WORLD bias carry, one exact14D handoff tick, lowering peak10mm/s, and full-vector
lower/cut integral gain0.5/s with75um norm steps and±3mm component bounds. The
initial seam is+106.5→−107mm at−1mm depth. Internal seam cleanup retains its
original−0.275mm depth and translates centrally at(0,−50,−40)mm before changing
yaw, with explicit incoming orientation. Any cleanup seam span outside
[−107,+106.5]mm fails before phase insertion; it is not clipped and no coverage
is fabricated. The physical/scoring/hardware limits and weak22mm jaw command
are unchanged. Full158-cut completion and released cutter parking now have same-episode
native proof in cut011; opening and collapse remain unproved.


## Cut011 milestone: actual cuts, unlock and released cutter parking

Fresh sealed cut011 used source commit`1cc0eed` without further policy/physics
changes. All158 cells first became cut at157.55s; the native outer-flap limits
released and measured bottom flaps subsequently moved to approximately−78.56,
−78.57,−84.87,−84.87deg. The cut/park controller completed at179s, score0.25,
with actual cutter_parked=true and cutter_held=false. Its3580 rows and110,980
native owner poses pass source/scene and independent geometry/scorer audits.
Final hands_clear=false, bottom_opened=false and flat-release hold0: no full
opening, flattening, task completion or eligible final media is claimed.

Archive-preserved file mtimes are mode19:57:50 and result20:05:12 on2026-11-15:
442s (about7.4min) for the recorded loop-to-result interval, at one-second mtime
resolution. This is not monotonic profiling and excludes startup before mode
creation; startup/render/export overhead must not be inferred from it.
