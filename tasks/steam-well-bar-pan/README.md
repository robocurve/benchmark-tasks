# Fit an adaptor bar across a countertop food warmer's well, then hang a quarter pan of macaroni and cheese on it

Two arms lift a loose flat stainless Vollrath 75020 adaptor bar (531.0 x 25.4 x 8.0 mm, the maker's listed size) off the stand and seat it across the well of a
12 x 20 in full-size countertop food warmer (304.8 x 508.0 mm opening), ends on the end rims. Then they slide, square up
and pick one 1/4-size steam table pan (161.8 x 263.5 mm, 15 mm flanges) holding a 12 mm layer of macaroni and cheese
(0.28 kg, a rigid block with drawn pasta) from the stand and hang it in the far half of
the well, centred along it: one long flange on the bar, the other on the far long rim. The pan's 161.8 mm side cannot
span the 304.8 mm opening, so at this centred position it falls in without the bar (checked with the control run
below). Near either end of the well the pan's short flange can also catch an end rim, which is why the pan is seated
centred. The pan must stay level: tilting it more than 15 deg after it is lifted exceeds the permitted carrying tilt and
scores 0 for the rest of the trial. The pinch is aimed at the pan's measured pose after the slide.

## Run a trial

From this folder, with the yam_arena / inspect-robots environment active:

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
      --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
      --scorer task_score --max-steps 1200 --log-dir ./logs

- The scripted demo plan is 899 steps (44.95 s at 20 Hz). The run limit is 1,200 steps.
- Seeds: none. Fixed start layout, object_jitter 0.
- Control run without the bar: create an empty file `NO_BAR` in this folder (or export `SWBP_NO_BAR=1`).
- Scorer unit tests (numpy only): `python test_bar_pan_scorer.py`.

## Score

`task_score` is computed in `task.py` by `w50_bar_pan_scorer.py` (bar logic from `w50_baseline_bar_scorer.py` and
`w50_bar_scorer_v2.py`, pan seat check `pan_ok_bar_frame` from `w50_four_pan_waypoints.py`), checked at
20 Hz over the final 1.0 s. The numeric rule is in `task.json` "scoring".

## Reproducing on real hardware

Every item is stock food-service equipment; nothing is custom-built. Prices were read on 2026-11-22 from the pages
listed (search-result prices where noted).

| Item | Maker and model | Price (USD) | Source page opened |
|---|---|---|---|
| Countertop food warmer, 12 x 20 in full size | Avantco W50 (item 177W50), "12" x 20" Full Size Electric Countertop Food Warmer - 120V, 1,200W" | 79.99 (search results) | https://www.webstaurantstore.com/search/avantco-w50.html ; specs https://www.avantcoequipment.com/product/177W50/ |
| Adaptor bar | Vollrath 75020, 20-inch steam table pan adaptor bar (1 x 20.907 x 0.313 in, 300-series stainless) | 10.49 (search results); maker MSRP 11.40 | https://www.webstaurantstore.com/search/vollrath-75020.html ; https://www.vollrathfoodservice.com/products/smallwares/steam-table-pans-accessories/steam-table-adaptors/adaptor-bars/75020 |
| 1/4-size steam table pan, 2-1/2 in deep | Vollrath Super Pan V 30422 | 13.92 (search results; listed out of stock that day) | https://www.webstaurantstore.com/search/vollrath-30422.html |
| Equipment stand, 30 x 48 in, 24 in work surface | Regency 600ES3048S with undershelf | 289.99 | https://www.webstaurantstore.com/regency-30-x-48-16-gauge-stainless-steel-equipment-stand-with-undershelf/600ES3048S.html |
| Sealed food stand-in, 0.28 kg, spread 12 mm deep | any sealed pouch of that mass laid flat in the pan | price not verified | - |

- Task-specific equipment (warmer, bar, pan): 104.40.
- Stand: any sturdy bench with a 610 mm work surface; the example stand is 289.99 and optional.
- Consumed per trial: nothing (the sealed stand-in is reused).
- Scoring tools: ruler, phone level app, phone camera for the side photo, the trial video.
- Loads are small for the arms: the bar is 0.136 kg (two-hand pinch), the pan with its stand-in is 0.73 kg carried by two
  hands (about 3.6 N per hand), far below the roughly 40 N each YAM finger pair can grip.
- Estimates: setup about 10 minutes; reset between trials about 1 minute (bar back flat on the apron, pan back on the
  left deck).
- Occupations: O*NET 35-3023.00 Fast Food and Counter Workers lists "Replenish foods at serving stations"
  (employment 2025: 3,855,900, https://www.onetonline.org/link/summary/35-3023.00); O*NET 35-9011.00 Dining Room and
  Cafeteria Attendants lists "Replenish supplies of food or equipment at steam tables or service bars"
  (https://www.onetonline.org/link/summary/35-9011.00); O*NET 35-2014.00 Cooks, Restaurant, employment 2025:
  1,417,500 (https://www.onetonline.org/link/summary/35-2014.00).

## Files

- `task.py`: scene list, scripted demo (bar transfer, pan slide, square-up, two-hand far-channel seat, park), the pan
  guards, per-step scoring.
- `w50_bar1pan_scene.py`, `scene_w50.py`: stand, warmer, bar and pan geometry.
- `w50_bar_pan_scorer.py`, `w50_baseline_bar_scorer.py`, `w50_bar_scorer_v2.py`, `w50_four_pan_waypoints.py`: scorer.
- `test_bar_pan_scorer.py`: teleport unit tests.
- `w50_bar_waypoints.py`, `w50_two_hand_near_waypoints.py`, `w50_slide_landing_variants.py`,
  `w50_squareup_and_crossing_witness.py`, `w50_occupied_q2_and_far_isolated_audit.py`, `w50_multi_pan_isolated_runner.py`,
  `w50_live_scorer.py`: trajectory and geometry helpers.
- `labkit.py`, `arena_assets_fix.py`, `hand_cover_data.py`: shared helpers.
