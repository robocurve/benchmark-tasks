# Return a powered enclosure guard to service

> Repair the detached edge and leave the enclosure guard ready for operation.

A section of the pressure-sensitive rubber on a vertical machine-enclosure guard has rolled out of its aluminum clip rail. The cable remains connected. The rigid shoulder can now reach a hand beneath the damaged span before the sensing surface operates. The bench reproduces a 300 mm working section of a vertical loading guard at full edge cross-section. Its guided rail has a 122 mm stroke.

## Importance & physical testing

The person at risk is an operator reaching through the opening during loading. A normal empty cycle can finish with the gate back at its upper stop in either repair state. A loaded test distinguishes them. The robot has to decide what evidence justifies returning the guard to service. A yellow gloved hand and blue adult-size forearm rest on a fixed loading shelf in front of the opening. They represent the next operator. The right arm moves a separate orange test cylinder by its outboard carriage handle. Its spring-loaded 50 mm crown and red travel pointer measure the outcome. The operator stays outside the sweep throughout. Probe travel measures loss of clearance in the apparatus. The experiment establishes neither a human injury threshold nor machine certification.

## What the robot can do

The robot can press the turned-out rubber crosswise until the second lip seats, move the instrument by the orange outboard handle, and press the green cycle button. The guide on the test carriage keeps repeated measurements at the same damaged span. The rubber repair has no handle. The reference uses the left fingertips on the rubber and the right arm on the test carriage. Neither arm supplies the door's motion.

The normal cycle descends and returns at the lower limit. Compression of the repaired sensing span during descent commands an earlier reversal and raises a red telltale. The telltale stays up until the next cycle. A separate limit on the test instrument inhibits the drive at 30 mm of probe travel. An intervention at that limit counts as a failed test. A white top-stop stripe and a pointer scale with 5 mm divisions permit independent video scoring.

## Scoring

| Score | Observable outcome |
|---:|---|
| 0 | Edge displaced, operator contact, instrument overtravel, robot contact with the guard or probe during powered motion, or instrument still out of its holder at the 1,200-action deadline. Safety faults latch. |
| 0.50 | Repaired span seated. A cycle in free air can earn this score. |
| 0.75 | At the repaired span the deployed probe physically loads the bulb, the switch trips during descent, the pointer moves at least 15 mm, the telltale rises at least 25 mm and the guard returns within 5 mm of home. |
| 1.00 | The probe returns within 8 mm of its parking stop and unloads. Both grippers are open and outboard, all robot contact with the fixture has ended, and the state persists across one second. |

A reopened guard leaves time to remove the test instrument. The intermediate 0.75 records the proof while recovery is in progress. If the carriage is still more than 8 mm from its holder at timeout, the final score is zero. A free-air cycle after repair is capped at 0.50.

The scorer uses measured joint positions, gripper positions and solver contacts. The robot receives the outcome instruction and station observations. One repair, one button press or one relay transition alone cannot demonstrate the complete outcome.

## Reproduction

The frozen asset is `assets/guard.usda`. Use `build_asset.py` to regenerate it from `fixture.py`. The station uses CPU PhysX at 240 Hz, 20 Hz actions, seed zero, zero object jitter and a 1,200-action maximum. The reference and intervention videos capture the calibrated station cameras. For physical build dimensions, signal routing, calibration and independent video measurements, see [PHYSICAL_CALIBRATION.md](PHYSICAL_CALIBRATION.md).

The dimensions and mounting method draw on the ASO SENTIR edge 25-40 CN and AL 25-14 retaining rail. The movable span uses a reduced-order roll coordinate and a compression spring. The fixture models local switching and return travel. Certification of the entire guard and characterization of viscoelastic material response require separate testing. ISO 13856-2 addresses design and testing of pressure-sensitive edges. OSHA 1910.212 provides general machine-guarding requirements. The 50 mm instrument and the scoring limits are specific to this benchmark. [ASO component catalogue](https://asosafety.com/shared-files/17827/?ASO-product-catalogue-2025-DE-EN.pdf=) · [ISO 13856-2](https://www.iso.org/standard/57108.html) · [OSHA 1910.212](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.212)
