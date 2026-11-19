# Saw a clamped wooden strip to length

A robot closes a screw vise on a loose wooden strip, picks up a hand saw and
cuts through the strip at a measured location. Full credit requires a separate
offcut and a retained piece measuring 150 ± 1 mm. This is a regular carpentry
operation: the [O*NET carpenter task list](https://www.onetonline.org/link/details/47-2031.00)
names measuring and marking cut lines and cutting materials to specified
measurements with hand tools as core tasks.

The scene has a 300 × 20 × 20 mm clear, straight-grained, kiln-dried Scots
pine (Pinus sylvestris) strip, 80 mm vise jaws and a 200 mm
hand-saw blade with a specified 10 TPI (2.54 mm pitch) alternating-bevel
crosscut tooth pattern, cutting on the pull stroke in a real trial. The strip rests partly in the open vise and partly on a low
support. A pencil line crosses its top 150 mm from the left end. The saw starts
in a rack. The robot must turn the screw to close the
moving jaw, then hold the saw and reciprocate it at the desired line. The saw
has no guide. A [75 mm commercial mechanics vise](https://www.faithfulltools.com/p/FAIMV75SG/Mechanics-Vice-75mm-%283in%29-with-Magnetic-Jaws)
and a [210 mm hand saw](https://silkysaws.com/silky-gomtaro-210-large-tooth-hand-saw/)
are scale and form references. The specified wood and 10 TPI pattern are trial
protocol choices, not claims about those products or calibrated cutting data.
The simulated geometry is procedural rather than a replica of either product.

The scripted demonstration aims the blade centre on the waste side of the
150 mm pencil line. This allows for the 1 mm tooth-strip width and the measured
lateral deflection of the hand-held saw; scoring still measures the actual
retained wood, not the commanded blade position.

## Run

With `yam-isaac-lab-arena` and the Inspect Robots embodiment installed, from
this folder:

```bash
inspect-robots run --no-prompt --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E device=cpu \
  -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 3200 --seed 3 --store-frames \
  --instruction "Clamp the wooden strip in the vise, then saw it so the retained piece is 150 mm long, within 1 mm. Leave the saw clear of the wood when finished."
```

The scripted robot demonstration uses only the 14-D arm and gripper action
interface to move objects. It reads object poses and the simulated material
grid to align and feed the saw; these reads choose actions only. The task
physics decides material removal and scoring.

```bash
python demo.py --mode full --out demo-out
```

The same runner has `idle`, `grasp` and `clamp` diagnostic modes. It writes
`result.json`, `physics.json`, camera images and videos in the output folder.
Use `--mode partial` for a robot trial stopping at the half-depth stage and
`--check-reset` to verify that a second episode restores intact loose stock.
The runner saves the final material array as well as poses and measurements.

## Validation

The final-source scripted robot run reached **1.0 in 2,170 actions** on the
published SDK, with two physical pieces and retained end-face measurements
149.25–149.5 mm. The separate reset restored intact loose stock, removed the
offcut and cleared material/history state. An independent development run
reached 1.0 in 2,210 actions; its partial demonstration reached 0.5 in 1,890
actions while the stock remained connected. Both resets passed. All 21
simulator scoring fixtures and 17 CPU checks passed. Evaluator-placed fixtures
check scoring boundaries and are not robot demonstrations.

Dream-1 beta seed 3 completed all 3,200 actions and scored **0.0**,
with zero errored trials. The command above reproduces its configuration.
Seeds 1 and 2 stopped with an official adapter gripper-range error after 1,364
and 2,644 actions respectively; neither has a valid score. The third attempt
is the first complete trial and was retained regardless of its score. No SDK,
task, action bounds or instruction changed between attempts.

The model video contains the complete 3,200-action trial. The demo video
contains every recorded frame of the 2,170-action trial (108.5 seconds), ending
at the scored state; only the separate post-trial reset check is excluded from
that video. Raw reset evidence is retained separately.

An earlier cloud trial scored zero under an any-fractional-removal veto,
despite correct physical separation and length: an out-of-zone accumulator
reached 0.03263 mm without clearing any 0.5 mm material layer. The current rule
below instead measures 0.5 mm damage. That failed receipt is preserved as a
failure; the full-score result above is a fresh trial on the current source.
The demonstrated controller uses measured poses and material state to choose
14-D robot controls; it does not write object poses or material state.

## Starting heights

The tabletop is 747.5 mm above the floor. Relative to it, the wood bottom and
support top are 80 mm, wood top 100 mm, 25 mm-high jaw centres 82 mm, and screw
axis 62 mm. The saw grip is spawned at 90 mm and settles onto its wooden
handle pedestal with its centre at 88 mm; the tooth edge is about 53 mm high.
The pedestal is 40 × 60 × 74 mm, with two open blade slots at y = 200 and
80 mm. Their uprights end at 75.5 mm, leaving the saw free to lift upward.

## Scoring and physical model

The task scores once at the end, taking the highest eligible stage:

| Score | Numeric end condition |
| --- | --- |
| 0.25 | Correctly clamped stock; no off-target removal. Intact stock, a shallow groove, or a groove through only part of the width can earn this stage. |
| 0.5 | One connected groove reaches at least 10 mm below the original top across the entire 20 mm width. At that depth its retained-side wall lies 149–151 mm from the original left end in every width sample. Stock is still one piece. |
| 1.0 | Exactly two physical pieces remain; every sampled width/depth point on the retained end is 149–151 mm from the original left end. |

Every stage also requires all of these conditions: the jaw opening is
19.2–20.2 mm (inclusive), screw rotation is greater than 1.5 rad, the retained
wood origin is less than 3 mm from its starting coordinate on each axis, its
orientation is within 2 degrees of its start, and the entire saw (blade, teeth,
guard and grip) is at least 5 mm clear of both wood pieces. The saw may be held
or placed anywhere meeting this clearance; returning it to the rack is optional.
The simulator conservatively measures clearance using bounds of each saw box
in each wood body's frame. A close diagonal pose can fail conservatively.

Measurable cutting while unclamped permanently gives zero, even if the vise is
tightened later. This means at least 0.5 mm cumulative removal attributable to
unclamped contact at any one material column, summed across loose episodes;
clamped contact does not erase the history. Off-target damage means at least one
cleared 0.5 mm layer outside the 149–154 mm work zone and gives zero. Sub-layer
fractional accumulators alone are not removed physical geometry and cannot
trigger either damage veto or earn cut-depth credit. Repeated contacts at the
same column accumulate; spreading shallow scratches over unrelated columns does
not add their depths together. This 5 mm zone allows kerf width and alignment correction around the pencil
line; it is not a relaxed full-score length tolerance. A completely severed
wrong-length piece scores zero, even if it once had a qualifying deep groove.
An incomplete cut within the work zone but missing the half-depth wall-location,
width or connectivity criterion earns at most 0.25. Separate cuts cannot be
added together to earn 0.5. No eligible stage means zero. A saw left at the wood,
loose/shifted/rotated stock, and cutting elsewhere cannot earn credit.

The 10 mm threshold is half the initial thickness: a substantial, unfinished
cut rather than a scratch. Check it in a real trial using a thin marked depth
probe at 2 mm intervals across the width and at both edges, side photographs,
and a ruler/calipers for the retained-side wall position. The minimum probe
depth must be 10 mm, with a continuous groove between probes; a narrow isolated
hole or cuts covering separate width segments do not qualify. Check clamping
from jaw measurements, a screw index mark, and top/side video throughout the
cut. With the saw withdrawn, measure out-of-zone groove depth with a thin marked
probe or depth gauge against the original top; 0.5 mm is the damage threshold.
During any loose episodes, compare the depth at each cut location before and
after that episode and sum the loose-only increments at the same location.
At least 0.5 mm gives zero. This avoids trying to measure invisible accumulator
values while retaining a measurable veto on cutting loose stock. Check saw clearance with a ruler and top/side images of both pieces.
Full credit still requires physical separation and the actual retained face
measurements, never a stroke count or elapsed time.

Each physics step uses the saw's measured pose relative to the freely moving
wood, its actual motion along the stroke and the measured saw–wood contact
force. The task removes only material under the tooth strip, in 0.25 mm by 2 mm
columns and 0.5 mm height increments. Collision shapes are rebuilt from the
remaining wood whenever a layer is removed. A separate rigid offcut is created when the remaining solid
disconnects across the board's width. The removal law is a reduced-order,
uncalibrated model of sawing; it does not resolve the specified individual teeth, stroke-direction asymmetry, grain,
splintering, bending or heat. The saw is 200 mm long and 150 g, compared with
the cited 210 mm, 200 g reference; its contact strip is a rectangular proxy.
The 6 mm-per-turn screw drive and its friction are approximations, with a 100 N
force cap against the reference vise's stated 750 N maximum. The box-built
vise, saw, rack and support use task-owned
procedural geometry.

`task.py` holds the scene and public scorer. `runtime.py` and
`cutting_runtime.py` operate the physical screw, contact sensor and dynamic
colliders. `stock.py` is the material and connected-solid model; `scoring.py` checks
stages, clamping and conservative whole-saw clearance. The cutting law and
scene geometry are unchanged; the scripted approach is described above.
`partkit.py` is a task-local derivative of the existing component helper and
does not modify the installed SDK. One layout is supplied; use
`-E object_jitter=0` for the stated dimensions.

## Third-party assets and licenses

| Asset | Use | Source and revision | License | Bundled here? |
| --- | --- | --- | --- | --- |
| YAM arms, grippers and meshes | Robot embodiment | [i2rt](https://github.com/i2rt-robotics/i2rt), commit `120c3c81400171174604e503943f8d1ebc891058`, via the installed YAM SDK | MIT, as shipped in `yam_arena/assets/yam/LICENSE.i2rt` | No |
| Station table, floor, cameras and lights | Base scene generated by the installed YAM SDK | [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at `0059d02a1a91bc6834c1d0846022891df0c78730` | SDK repository states that it has no license yet; its i2rt robot assets are MIT | No |
| Isaac Lab-Arena and PhysX | Simulation runtime | Installed simulator, Isaac Lab-Arena 0.3.1 | Apache 2.0 for Isaac Lab-Arena; PhysX supplied with Isaac Sim | No |
| Wood, vise, screw, saw, rack and task code | Task geometry and logic | This folder | MIT; see `LICENSE` | Yes |

No third-party object mesh, texture or photograph is bundled. The task does
not load any object from the YAM SDK's station library.
