# Spike an IV bag and prime its drip chamber

This bimanual task models preparation of a 500 mL saline bag and gravity giving set. The left arm stabilizes the bag by
its hanger tab. The right arm removes the spike-port cap, seats the spike to its collar, lowers the drip chamber below
the wetted port, and primes it. Full score requires a measured 15-27 mm liquid column, release, and both arms home.

The task is materially different from simply moving an IV bag or routing tubing: cap removal, forceful axial insertion,
gravity-head orientation and controlled liquid-column height must all be achieved in one ordered workflow.

## Physical setup

- Fixed 34 x 54 x 0.4 cm stainless tray, spanning x 0.16-0.50 m and y -0.30-0.24 m.
- Sealed 500 mL PVC saline bag centred at (0.32, 0.06) m, ports toward -y. Its 13 x 20 x 2.6 cm external pillow has a
  676 mL rectangular bounding volume and a measured filled mass of 530 g. It has a welded rim, 28 x 8 x 20 mm upright
  hanger tab, 10 mm-diameter x 30 mm spike port with pull-off cap, and adjacent medication port.
- Giving set centred at (0.42, -0.13) m: 25 mm spike, 20 mm inside-diameter x 45 mm clear chamber, 39 mm usable
  liquid-height span, 10 cm tube stub and roller clamp. The spike points toward the bag.
- The sealed full bag keeps the low-side spike-port lumen wetted. After insertion, lower the chamber centre at least
  10 mm below the port before squeezing and releasing it.

For a real trial, use a fresh sterile bag and giving set. Do not reuse a pierced bag. Clinical connection and patient
use are outside this benchmark; the trial ends at bench preparation.

## Model and scoring

`bodykit.py` builds the compound rigid bodies and joints. `priming_meter.py` is a pure state model for chamber fill.
Unlike a squeeze counter, it measures finger travel during each complete squeeze/release cycle. The measured
compression is calibrated to liquid-column rise and accumulated continuously. Closing the hand before spike lock,
away from the chamber, or without the required 10 mm gravity head adds no liquid.

The visual column is split into six small segments, while scoring uses the continuous height:

| Outcome | Score |
|---|---:|
| Cap displaced at least 5 mm | 0.25 |
| Spike seated 23-25 mm, collar at port mouth | 0.5 |
| Seated; liquid height 15-27 mm; released; bag on tray; both arms home and objects resting for 1 s | 1.0 |
| Liquid height above 34 mm | 0.0 |
| Any other outcome | highest completed stage, otherwise 0.0 |

For full score each TCP must be at least 4 cm from the chamber and port, and within 5 cm of its initial pose. The
overfill/spill latch prevents repeated squeezing from gaming the task.

## Run

From this directory in the Isaac Lab Arena environment:

```bash
INSTRUCTION="Prepare the IV set: hold the saline bag by its hanger tab, remove the blue spike-port cap, seat the giving-set spike to its collar, lower the chamber below the port, and squeeze and release it until the liquid column is 15-27 mm high. Release everything and return both arms to their starting poses."

# Required Dream-1 run
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/iv-bag-dream

# Scripted feasibility demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/iv-bag-demo

python -m pytest -q test_priming_meter.py
```

The scripted sequence takes roughly 600 control steps; Dream-1 receives the full 2,400-step limit. The layout is fixed
with `object_jitter=0`, so no seed changes object placement.

## Reset

Simulation reset restores cap, spike joint, bag pose and zero liquid height. For a physical trial, discard the pierced
bag and giving set, then install a fresh sealed pair at the measured coordinates. Confirm the chamber is empty, the
spike cap is attached and the roller clamp is closed before recording.
