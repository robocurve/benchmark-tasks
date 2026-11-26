# Set up and check a folding utility cart

Lower the utility cart floor, slide both sidewall locks across their hinges, and press the sidewalls inward to check that they stay open.

The dbest Quik Cart Pro 01-803 folds flat for storage. Its two molded sidewalls each have a center hinge and a captive bar that bridges the hinge after setup. A straight-looking wall can still fold inward with its bar parked on one half. The separate floor swings down between the lower walls and rests on the molded base. The floor and bars make the opened cart stable before groceries, books, or work supplies are put in it. [dbest Quik Cart Pro](https://dbest-products.com/collections/teachers-and-students/products/quik-cart-pro?variant=40102344917155), [manufacturer patent US11338835B2, Figs. 2, 6 and 8](https://patents.google.com/patent/US11338835B2/en), [manufacturer instructions, step 5](https://www.manualslib.com/manual/3805909/Dbest-Quick-Cart-Pro-01759mb.html).

The rigid cart uses its own floor hinge, base support, and independently captive sidewall bars. The check concerns the cart's folding joints before loading it.

## Setup

The cart's named basket dimensions are 432 mm wide × 406 mm deep × 432 mm high. Its polypropylene panels, steel handle and plastic rear wheels are represented at stock scale. The body sits upright on a 700 × 780 mm lowered work deck, with the deck top 525 mm from the floor. Two risers keep the stock robots at the usual 750 mm base height. The lid is removed from the work area and the handle remains stowed. The front wall is farther from the arms. Both sidewalls start partly unfolded with their center hinges directed inward, their rigidity bars entirely on one leaf, and the floor raised above its supporting faces.

Dimensions of the panel ribs, hinge pins, and slider groove were estimated from the manufacturer photo and patent drawings. The estimated floor is 382 mm wide. The near-hinge support face uses a 1.5 mm relief so the floor can bear on the rear molded supports. The last 35 mm of the floor has 2–5 mm of corner relief per side around the factory contour and clear of the inward ends of the sliding bars. The wall hinge axes, floor support and captive rail cross-sections are physical collision geometry. The stock molded finger opening and shallow floor edge recesses are modeled. The telescoping handle stays collapsed. Material stiffness and friction are simulation parameters. Each captive bar has the same passive viscous guide drag of 8 N·s/m. It has no position drive. We did not physically measure a cart, production tolerances, collapse resistance or load rating. [Manufacturer annotated rail photos for the blue variant](https://mobileimages.lowes.com/productimages/f8bb0cf9-d294-48eb-bcfa-d54b5fe56f60/63494827.jpg).

## Run locally

Run from this task directory with Inspect Robots and the YAM simulation environment installed.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py \
  -E control_hz=20 -E object_jitter=0 -E cam_width=320 -E cam_height=176 \
  --instruction "Lower the utility cart floor, slide both sidewall locks across their hinges, and press the sidewalls inward to check that they stay open." \
  --scorer task_score --max-steps 1000 --seed 0 --store-frames
```

Set `CART_EXPERT=unlock_before_proof` to assemble the same cart and seat the same floor, then retract both stock bars before repeating the inward press. Set `CART_DIAGNOSTICS` to a writable path for per-control witness measurements. To capture both fixed side views and the stock overhead view, replace `inspect-robots` in the command with `python capture_detail.py`.

## Scoring

Score 0.25 when the floor is within 2° of horizontal and sits on the cart's bottom support faces. Score 0.50 when both wall hinges are also within 2° of straight. Score 0.75 when the two captive bars have each traveled at least 80 mm and bridge the center joints. Full credit requires a monitored inward push above 3 N on each wall for 0.75 s, each receiving rail reading above 0.4 N, folds staying within 2° of straight, the cart remaining on the bench, and both hands releasing and clearing the basket. The force required here checks this test setup and does not assert a manufacturer's strength limit.

For a physical trial, set a fixed overhead camera and 100 mm ruler on the deck. Photograph the retracted and bridged positions and measure each molded slider's travel along its track. Place a straightedge along each side wall to check center-hinge angle. An angle gauge on the floor confirms the 2° limit and the underside view shows the floor supported by the cart. Two small compression force gauges against the sidewall ribs provide the inward load without touching either bar directly. Keep the same gauge positions, load, and duration for the retracted-bar control, and observe both receiver rails and any hinge angle change under force.

## Verified local trial

The scripted trial scored 1.0 at 716 controls with CLI seed 0 and episode seed 1415158438. It took 35.8 simulated seconds and 177.36 seconds of wall time. The floor settled under its own weight by control 562 and finished 0.017° from horizontal, with 3.84 N on the rear support. Both bars finished across their hinges at 91.81 and 84.24 mm of travel. During the inward press, the two receiving rails saw peaks of 17.73 and 16.51 N, and both walls stayed within 1.53° of straight. Both sides met the proof conditions for 39 consecutive controls. The cart stayed on the deck, and no floor-to-bar or arm-to-arm contacts occurred. I checked the reset, approach, both grasps, slider travel, floor seating, inward press and release through the retreat in the encoded overhead and both fixed detail views.

The retraction control scored 0.25 at 965 controls with the same seed. After the floor seated, the arms physically moved both bars back and lifted clear. During the same commanded inward press, bar travel remained below 7.18 mm and no load reached either receiving rail. The folds reached 6.02° and 5.44°, with measured peak fingertip loads of 2.55 N and 3.59 N. As the unlocked walls yielded, the two sides never sustained 3 N simultaneously. The floor still finished 0.017° from horizontal and both hands cleared. I reviewed the actual encoded reset through retreat and both detail views at bar retraction and inward pressure.

The local archive holds `review/setup.png`, `review/demo.mp4`, `review/detail-left.mp4`, `review/detail-right.mp4` `review/verification.json`, `review/unlocked-overview.mp4`, `review/unlocked-left.mp4`, `review/unlocked-right.mp4` and `review/control.json`.
