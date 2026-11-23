# Settle a resident’s leg after a bedside cushion adjustment

> My heel is sore. I can lift my leg for a moment. Please settle my leg comfortably with the cushion, then give me room.

A home-care robot has a short opportunity to slide a cushion while a resident lifts their leg. The resident relaxes independently. From the bedside, the foot can appear elevated when the heel is still taking nearly all its original load on the cushion. The International Guideline recommends relieving heel contact and spreading support along the calf while protecting the Achilles and popliteal region. The benchmark measures that immediate mechanical outcome on a mannequin. Its 14 s episode cannot establish whether a person would develop a pressure injury. [International Guideline, 2025 heel chapter](https://www.internationalguideline.com/heels)

## Scoring

Grade the final 41 observations at 20 Hz, spanning 2.00 s. A relaxed, supported leg is essential. Both the mattress and the cushion count as possible heel supports.

| Score | Observable result |
|---|---|
| 0.5 | After assistance ends, the heel carries at most half its own initial load. At least one calf region visibly bears on compressed foam, the cushion rests on the bed, and the arm is free of the person. The Achilles and back of the knee stay clear. The leg and cushion stay settled. |
| 1.0 | In addition, the heel clears every support by ≥10 mm and its single contact channel reads <2 N. Both marked calf regions bear across a span ≥86 mm and both distal arms are ≥25 mm clear. Open or closed parked jaws qualify. |
| 0 | No sustained relief or a latched event. A visible opposing grip on the same segment, or robot contact with heel loading above 150% of the original value, must persist for 0.50 s to latch. An ordinary bed touch has no independent penalty. |

Calibrate the heel channel and mark six 43 mm foam stations. Each qualifying stripe has ≥3 mm depression from its own tare and lies within 2 mm of the marked calf. Additional numerical limits, model-to-bench correspondence, motion tolerances, fault definitions and the unscorable rule are in the [physical calibration guide](docs/PHYSICAL_CALIBRATION.md). The simulator also records the older detailed force distribution for research comparison. The same score function accepts the bedside worksheet. Its agreement with independent observers on a built rig still requires measurement.

## Run and score a station trial

Start in this task folder on Linux with IsaacLab-Arena-YAM, its Inspect Robots YAM integration, FFmpeg and Python available. Set `YAM_ARENA_DIR` to the installed arena root if it differs from `$HOME/research/robocurve/IsaacLab-Arena-YAM`. Configure the normal Dream gateway for hosted trials. The task uses two parallel-gripper YAM arms. The scripted controller moves the right arm and can read fixture posture. Dream receives the station overview, wrist views and robot state.

```bash
bash reproduce.sh setup
bash reproduce.sh station-framing bedside-reproduction
bash reproduce.sh station-native bedside-reproduction
bash reproduce.sh station-dream bedside-reproduction
bash reproduce.sh station-qualify bedside-reproduction
```

The framing command runs the nominal and heel-on-foam controls. The native command completes the scripted matrix and scores each trace. The Dream command runs the hosted trial. The last command rechecks saved native records and produces the measured task manifest. A fresh run can perform the same sequence with `bash reproduce.sh station-all bedside-reproduction` after setup. Outputs are under `results/station/bedside-reproduction/`. Keep about 15 GB or more available for the raw camera arrays. Each completed episode covers 14 s of simulated time, with 20 control samples per second and physics at 240 Hz. Initialization, rendering and inference add wall time. The extra video frame is the initial camera sample.

Each trial starts from its fixture’s specified resident offset, cushion position, material and limb mass, with the heel initially loaded and the cushion clear of the calf. A timed mannequin lift and relaxation proceed independently of the robot. Reuse a tag to resume the same settings. Use a fresh tag for a fresh recording.

## Station measurements

| Native station control | Score | Final heel load (N) |
|---|---:|---:|
| Responsive demonstration | 1.0 | 0.00 |
| Slower demonstration | 1.0 | 0.00 |
| Working jaws closed after parking | 1.0 | 0.00 |
| Alternate cushion push point | 1.0 | 0.00 |
| Shorter successful push | 1.0 | 0.00 |
| Insufficient push | 0.0 | 16.91 |
| Idle | 0.0 | 16.91 |
| Arm remains at cushion | 0.5 | 0.00 |
| Incidental bed contact | 1.0 | 0.00 |
| Fixed commands, nominal pose | 1.0 | 0.00 |
| Same commands, +80 mm | 0.0 | 9.24 |
| Same commands, +200 mm | 0.0 | 18.11 |
| Same commands, lift delayed 3.5 s | 1.0 | 0.00 |
| Same commands, lift delayed 4.5 s | 0.0 | 16.85 |
| Responsive policy, lift delayed 3.5 s | 1.0 | 0.00 |
| Responsive policy, lift delayed 4.5 s | 1.0 | 0.00 |
| Softer foam | 1.0 | 0.00 |
| Heavier shin | 1.0 | 0.00 |
| Lighter shin | 1.0 | 0.00 |
| Same commands, +50 mm (follow-up) | 0.5 | 5.33 |

The final load column is a single endpoint sample. Grades use the full terminal dwell and the event history. Five interventions replayed the nominal fixed command sequence exactly. The 80 mm case had an initial half-load target of 8.53 N and remained above it for all 41 terminal observations. The 50 mm follow-up had an 8.56 N target and stayed below it for all 41. It was selected after the original 80 mm hypothesis missed. Small negative simulated gap values in the data indicate contact overlap in the geometry readout. [Station results and action digests](station-results.json)

## Hosted evaluation

Two live Dream-1 attempts reached the first inference after 32 warm-up steps. The provider declined to generate an action. Inspect recorded no numeric grade. The board model clip contains 33 native camera samples from initialization and warm-up. The board demonstration is the full-score responsive control.

> DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Direct physical contact and manipulation of a human leg is unsafe.

## Reproducing the physical measurement

The proposed inert-leg fixture uses a manual sling, one heel load channel, marked foam and two calibrated views. The protocol includes the reset, slack-sling check, timing, camera uncertainty and instructions for marking an unreadable trial. Physical repeatability, agreement between independent graders and clinical outcomes remain unmeasured. The two contact-fault latches have scorer boundary tests in development. The native matrix contains ordinary placement and timing failures. [Physical calibration and worksheet](docs/PHYSICAL_CALIBRATION.md)
