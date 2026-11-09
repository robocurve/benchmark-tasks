# Judging the magnifier task

The skill is positioning and orienting a hand-held lens **relative to the wrist
camera** so it reveals useful detail. A plausible lens pose alone earns no credit.
Finish the trial holding the magnifier by its handle, with the whole tool at least
20 mm above the table, while the left wrist camera has an enlarged, readable view
for the final three seconds. The limit is 1,200 actions at 20 Hz (60 seconds).

Use a ruler, the full video, and a pixel viewer for the camera images. The supplied
scorer makes the same measurements automatically; no force sensor is required.

## Physical conditions

Watch the complete trial. It must start from the specified layout with the tool
on the table. The robot must pick it up by its handle using the gripper. Throughout
the final three seconds, the gripper must support it by the handle, every part of
the tool must be at least 20 mm above the table, and the tool must not rest against
the camera, another robot link or another object. The tool and target must remain
intact and unmodified, and the printed target must stay fixed. Object attachments,
teleports, external assistance and manipulated images cannot earn credit.

Use the ruler or a calibrated side view to check the lowest point, including the
frame and glass. In simulation the scorer transforms all tool mesh vertices by
the recorded actual pose. Video review checks pickup, support and visible continuity. Both recordings
sample at 20 Hz; they do not certify motion between samples. Stable object
pose alone is not proof of a grasp.

## Image check

Keep the original 640×360 wrist images and the actual 224×224 wrapper observations.
The latter are the images available to the policy before any model transport
preprocessing. Display enlargement is allowed for inspection; all measurements
refer to original pixels. Do not crop, zoom or change camera intrinsics.

1. Take a lens-free reference at the final camera and target pose. After a
   simulated trial, freeze physics and hide only the glass; leave the frame and
   handle visible. In a physical trial, hold the camera still and move the whole
   magnifier aside. Neither reference operation is part of the robot's trial.
   All four outer black mark centres must stay within **one native image pixel**
   of the reference in every image of the three-second hold. This allows small
   wrist jitter while preventing a closer camera from earning magnification.
2. Measure each of the four sides of the square formed by the coloured dot
   centres. Divide its held-image length by its reference length. Do this in both
   image sizes and use the **smallest of the eight ratios**. This prevents a
   single stretched edge from standing in for useful magnification.
3. In the 224×224 images, count the two five-bar patterns. A readable pattern has
   **five dark bars and four bright gaps**, each containing at least one original
   pixel, across the entire central half of the bar length. Follow the printed
   pattern's orientation; camera roll is allowed. The black and white swatches
   give brightness values B and W. A qualifying dark pixel is at most
   `B + 0.4(W−B)`; a qualifying bright pixel is at least `B + 0.6(W−B)`.
   The swatches must differ by at least 20% of the full brightness range.
   These margins keep weak grey noise from counting as separated detail.
4. Repeat throughout the final three seconds. At least one pattern must be
   readable with the glass and unreadable in the lens-free reference. The
   reference and all 61 states at the interval endpoints and intervening 20 Hz
   samples must be available. The full video must show an uninterrupted hold.

| Score | Required throughout the same three seconds |
| --- | --- |
| **1.0** | All physical conditions; at least **1.5×** enlargement; **both** patterns readable; at least one newly readable with the glass. |
| **0.5** | All physical conditions; at least **1.25×** enlargement; at least **one** pattern newly readable with the glass. |
| **0.0** | Any other outcome, including dropping or propping the tool, insufficient clearance, target absent, unreadable patterns, insufficient enlargement, or outer marks moving more than one pixel. |

Missing or corrupted evidence also produces zero, marked **uncertified**. A
complete unsuccessful trial is a valid zero; failure to see the target is not
mistaken for a missing image. No score is inferred from lens geometry alone.

## Reproduce a score

The run directory contains `pickup.json`, the full `trace.jsonl`, original
`baseline`/`final` PNGs and NPZ arrays, and the 61 `hold-NNNN-raw.png` /
`hold-NNNN-policy.png` pairs. The runner records the exact wrapper resize and
frozen-reference checks. The scorer independently checks the image resize,
ordered timestamps, image integrity and all per-frame measurements.

Create a review form bound to the actual trace, video, run record and target:

```bash
python scoring.py --run RUN --target-json RUN/target/target.json \
  --write-review-template --video-file demo.mp4 --output RUN/review.json
```

Watch the full video and inspect the run provenance, then complete the named
observations in `review.json`. Use `false` for an observed failure and leave
unknown observations `null`. Set the reviewer name and `reviewed_full_trial`.
For a Dream trial use `--video-file dream.mp4`.

```bash
python scoring.py --run RUN --target-json RUN/target/target.json \
  --review RUN/review.json --output RUN/score.json
python -m unittest test_scoring
python inspection_measure.py --self-test
python score_hold.py --self-test
```

`score_hold.py` can also report the optical component alone. `scoring.py` is the
complete task scorer: its nonzero result requires the numeric checks and the
completed, matching physical review. A missing review cannot silently pass.

The pixel checker follows chart-aligned paths through original image pixels,
spaced no more than one pixel apart across the central half of each pattern.
It neither rotates/resamples the image nor invents additional pixels. Outer
marks are located against their own local background, with contrast and square
shape checks. Strong nonlinear distortion can make the printed regions
unmeasurable and then earns zero; there is no pose-based fallback.
