# Open a box with a three-wheel combination lock

Set the three numbered wheels to 3–1–7, then open the lid. Each large wheel
turns independently on a physical axle and has an orange fixed index. The lid
has a real hinge and raised handle. All wheels start at incorrect digits.

## Reproduce

Use Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0
and YAM Arena 0.1.0. Source pins are YAM
`0059d02a1a91bc6834c1d0846022891df0c78730` and Arena
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`.

From the benchmark repository, activate the installed Arena/YAM environment:

```sh
export TASKDIR="$PWD/tasks/rotary-combination-box"
uv pip install --no-deps -e "$TASKDIR"
inspect-robots run --embodiment yam_rotary_box --policy rotary_box_scripted \
  -E task="$TASKDIR/task.py" -E object_jitter=0 -E device=cpu \
  -E top_cam=demo -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Set the three numbered wheels to 3–1–7, then open the lid." \
  --scorer task_score --max-steps 3600 --seed 0 --store-frames --log-dir rotary-demo
```

For Dream replace `--policy rotary_box_scripted` with
`--policy dream -P model=dreamlabs/dream-1-beta`; use a separate log directory.
Both use the identical embodiment, cameras, physics and task revision.
The scripted policy uses the known layout but commands only robot joints.

## Fixed setup

Coordinates are metres, x forward, y left, z up; tabletop z=0.7475. The
existing YAM station arm bases are `(0, ±0.305, 0.750)`. The fixed storage box
has an adapted catalog visual and matching analytic floor/walls. It occupies
x=0.250–0.390, y=−0.200–0.200, height 60 mm. A 120×400×10 mm wheel ledge
extends toward the robot, x=0.130–0.250. Three wheel centers are
`(0.190, 0.120/0.000/−0.120, 0.7875)`, in code order. Initial digits are
2, 0, 8. Each hub is 70 mm diameter, 26.5 mm thick, 80 g, with a
45×18×12 mm raised gripping bar. Wheel pitch is 120 mm. Digits 0–9 are
printed around each hub; an orange fixed triangle points to the selected digit.

The separate 140×400×8 mm lid is 120 g. Its world-y hinge is at
`(0.250, 0, 0.8125)`. The raised handle's bar is 55×18×12 mm, centered
at `(0.360, −0.120, 0.8385)` while closed. The catalog bin and hub geometry,
source hashes and asset-use terms are in `assets/provenance.json` and
`assets/SOURCE.txt`. Build with `python build_assets.py WHEEL_USD BIN_VISUAL_USD`
using the recorded source downloads; generated assets are self-contained.

CPU PhysX runs at 240 Hz, control at 20 Hz; seed 0, zero jitter. Uniform
static/dynamic friction is 1.2/1.0; task collider contact/rest offsets are
0.3/0 mm. Observations are 640×360. The existing top demo camera uses its native
pose and intrinsics without crop or zoom; wrist calibrations are unchanged.

## Mechanism and score

Each wheel has a world-anchored revolute z joint. Equal soft detents apply
`−0.0005*sin(10*angle) − 0.0005*angular_velocity` Nm about z. Every digit is
stable; torque never uses the secret code. A simplified latch holds the lid's
real y revolute joint at 0°. Only when all measured wheel angles are within
5° of 3–1–7 does its lower angular limit change to −110°. The latch then
stays retracted until reset. Gravity and robot contact move the lid; there is
no opening actuator or object pose command. This approximates the latch,
not a commercial combination lock's complete internal mechanism.

End score is 0.2 per correct wheel after release: both TCPs over 80 mm away,
angular speed under 0.1 rad/s and within 5° for one continuous second. A later
wrong setting loses that wheel's credit. Axle displacement over 5 mm or tilt
over 5° permanently invalidates that wheel. The remaining 0.4 requires the
lid open at least 60° after unlocking. Hinge-anchor displacement over 5 mm,
axis tilt over 5°, or opening over 5° while locked permanently invalidates
opening credit. No opening credit is earned by forcing or bypassing the latch.

Reset restores the layout, closes and locks the lid, clears retention/bypass
history and returns fresh camera observations after temporal-history flushing.
`pytest -q tasks/rotary-combination-box/test_scoring.py` checks partial and full
scores, wraparound, tolerance, release, spin, lost settings and forced opening.

## Recorded runs

Runtime revision: `a3cbf1145df54a0676f433ca2543ec22762c4916cc086e3cfca8c95269cc3660`.

| Policy | Score | Steps | Video time | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Scripted | 1.0 | 941 | 47.10 s | 108.64 s |
| Dream-1 beta | 0.0 | 3,600 | 180.05 s | 659.87 s |

Both completed without trial errors, using the same source revision, cameras,
physics, seed 0, zero jitter and 3,600-step limit. The recorded videos and submission text were reviewed and approved for
submission. The uploaded demo, Dream run and setup image are recorded as
media `M-QXQPSE` in `task.json`; synchronized wrist views are retained locally.

The native top-camera pose is approximately `(-0.166495,-0.009000,1.541821)` m,
quaternion xyzw `(0.212629,-0.212629,-0.674380,0.674380)`, OpenGL convention.
At 640×360 its intrinsics are fx=392.195618, fy=391.722351,
cx=318.389435, cy=177.876312. The exact calibration and result records are in
`validation.json`. Physics and reset checks were performed before the camera
revision; their physics inputs are unchanged. The recorded demos demonstrate
full scores under the final camera configuration.

The one-wheel physics milestone preceded scene expansion: the robot rotated
2 to 3 and released the wheel stationary at 108° with its axle fixed.
