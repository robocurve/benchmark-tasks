# Spike an IV bag and prime its drip chamber

This task models aseptic preparation of an infusion set. A filled saline bag hangs from an IV support with both ports
down. The robot removes the bag-port and giving-set spike caps, keeps the chamber upright, inserts the untouched spike
upward to its collar, and primes the chamber to a measured 15–27 mm gravity-referenced liquid height. Full credit
requires release and return home without either exposed sterile interface touching a gripper or work surface.

## Reproducible setup

- Nonsterile stainless work tray: 34 × 54 × 0.4 cm, covering `x=0.16–0.50 m`, `y=-0.30–0.24 m`.
- Sealed 500 mL PVC saline bag: 13 × 20 × 2.6 cm, 530 g, hanging vertically at `(0.39,-0.122,0.99) m` from a fixed IV
  support, with a hanger opening and a 10 mm-diameter × 30 mm spike port pointing downward.
- Bag-port cap: blue pull-off cylinder with an extended 12 × 12 × 110 mm handling strip; its seal releases at 3.5 N.
- Giving-set chamber centre: `(0.36,-0.30,0.8315) m`, upright in an 80 × 80 mm presentation rack with a raised
  support. A simulated latch holds the set during cap removal and releases when the fingers enclose the chamber
  after the cap has cleared. This latch is a discrete fixture model, not a measured snap force.
  The rigid chamber/spike assembly weighs 20 g. The chamber is 20 mm in diameter and 45 mm long, with a 39 mm
  graduated span; molded handling wings and flanges resist slip. Tubing and its closed roller clamp are visual proxies.
- Spike cap: navy pull-off cylinder with a 12 × 12 × 40 mm offset handling tab; its seal releases at 0.08 N.
- Fixed layout: `object_jitter=0`.

The capped spike and sealed port are the sterile interfaces. After either cap is removed, that interface must not
touch a robot gripper or the work tray. During priming, keep the chamber's longitudinal axis within 12° of vertical,
with outlet below inlet and its lower region at least 10 mm below the wetted port. Measure liquid height upward from
the outlet-end graduation, so the target is gravity-referenced rather than an axial length in an arbitrary pose.

## Scoring

Contamination irreversibly scores 0 before spike lock if the open-port disk or exposed-spike segment intersects the
union of convex hulls for every URDF collision mesh on both palms and all four moving finger links, or if
the sterile surface reaches within 0.5 mm of the nonsterile tray. Liquid overfill above 34 mm also scores 0. Otherwise:

| End state | Score |
|---|---:|
| Both protective caps displaced at least 6 mm | 0.25 |
| Spike seated 23–25 mm, collar at the port mouth | 0.50 |
| Uncontaminated seated spike; 15–27 mm vertical fill; valid chamber orientation; release, rest and both arms home | 1.00 |

For full score the bag remains on its IV support, both TCPs are at least 4 cm from the chamber and port, both
TCPs finish within 5 cm of their initial poses, and the bag and set rest for one continuous second.

The simulator uses rigid bodies and geometric cap/spike measurements. Mesh-derived collision envelopes follow each
measured link pose and include offset pads and finger shafts. Each hull is expanded by the sterile-interface radius;
this remains conservative near mesh edges and corners and may reject near contacts. These checks
they do not simulate microorganisms or glove sterility.
`priming_meter.py` converts measured finger travel into vertical graduated liquid height only while the spike is
locked and chamber orientation and gravity-head conditions are valid.

## Run

```bash
python configure_native_camera.py

INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Scripted demonstration (validate the resulting score before submitting)
IV_VIDEO_DIR=~/yam-eval/iv-bag-demo/media python record_video_cli.py run --embodiment yam_arena --policy yam_scripted \
  -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --log-dir ~/yam-eval/iv-bag-demo/log

# Required Dream-1 attempt
IV_VIDEO_DIR=~/yam-eval/iv-bag-dream/media python record_video_cli.py run --embodiment yam_arena --policy dream \
  -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --log-dir ~/yam-eval/iv-bag-dream/log
```

Render submission recordings as H.264, 1280 × 720, `yuv420p`, under 50 MB.
The recorder writes `run.mp4`, the initial `setup.png`, and `recording.json` with every simulation timestamp.
It checks native sensor dimensions and records the top-camera observations without changing policy inputs.
Use a fresh output directory for each attempt. Runtime dependencies are the YAM Isaac Lab Arena/Inspect Robots
environment, NumPy, SciPy, Pillow and FFmpeg. The contact geometry loads the installed YAM URDF collision meshes.

## Verified revision

On 2026-11-16 the scripted run scored **1.0 in 1,132 control steps**. Its final measured seating depth was
24.9 mm, fill height 22.0 mm, and tilt approximately 0.1°; contamination remained false. A separate diagnostic
also completed at 1.0. Dream-1 beta completed **2,400 control steps at 0.0**. The final recordings contain
1,133 and 2,401 frames respectively, including the initial frame, at native 1280 × 720 and 20 fps.
Both H.264/yuv420p videos were fully decoded and checked for missing or duplicate simulation timestamps.
Nineteen regression tests passed. `validation.json` records the measured scores, step counts and evidence hashes.

## Reset and real-trial safety

Use a fresh sealed bag and fresh sterile giving set for every real trial. Discard either item if the exposed spike or
open port touched a glove, gripper or work surface. Hang the bag vertically and return the capped giving set to its stated coordinates,
fully attach both protective caps, empty the chamber, and close the roller clamp. Simulation reset restores both cap
seals, the contamination latch, spike joint and liquid length. Follow the product instructions and local clinical
aseptic policy; this benchmark is not a substitute for clinical training.
