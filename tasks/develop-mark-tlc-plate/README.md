# Finish a thin-layer chromatogram and mark its solvent front

Thin-layer chromatography (TLC) separates components of a sample on a coated plate. The solvent front moves while the plate remains immersed and disappears once the plate dries. Its position is recorded with graphite to calculate retention factors. This trial starts with a plate developing in a covered chamber. It tests timed extraction through the chamber opening, clean edge handling, and a light pencil mark at the wet boundary while supporting the plate.
[CU Boulder laboratory method](https://www.orgchemboulder.com/Technique/Procedures/TLC/TLC.shtml) · [Homi Bhabha Centre edge-handling procedure](https://chem.hbcse.tifr.res.in/wp-content/uploads/2016/11/A-Guide-to-performing-the-Thin-Layer-Chromatography-technique.pdf)

## Apparatus and reset

Dimensions are in millimetres. YAM bases sit at (0, ±305, 750), with X pointing away from the bases and positive Y to their left. Tabletop height is 747.5.

| Part | Size, material and start |
| --- | --- |
| Developing chamber | Clear borosilicate tank, 92 × 120 × 75, with 3.5 mm walls and a 4 × 65 × 8 glass baffle positioned 66 mm up and 17.5 mm toward the rear from the tank centre. Place its lower face at (289, −64, 751.5) on a 104 × 132 × 4 non-slip mat. Fill to 4.5 mm with 1% saline. Use a comparable lidded clear TLC tank or assemble matching borosilicate sheets with cured aquarium silicone. |
| Cover | Glass hood with a 101 × 128 × 5 roof, a 29 mm glass skirt that rests on the rim, and a 28 mm × 18 mm knob on a 14 mm stem. Use a short clear glass tank and a matching handled glass hood, or bond a standard knob to the roof with two-part epoxy. The two vessels form the covered development chamber. Keep (389, 232) clear for the removed cover. |
| TLC plate | One 50 × 100 × approximately 2 glass-backed silica plate, e.g. Supelco 1.05789. Draw a light pencil baseline 12 mm from the lower edge. Make three small, separate red, yellow and blue food dye spots along it with clean toothpick tips and allow them to dry. Stand the plate in the chamber with only its lower edge immersed, leaning about 12° from vertical against the glass baffle. Coating faces the robot. |
| Pencil | Pressure-cushion mechanical pencil with soft graphite, approximately 136 mm long with 8 mm shaft, center at (216, 212, 771.5), point toward positive X. Two shallow rests keep the pencil from rolling. |
| Drying rest | Rigid 61 × 85 × 14 plastic or silicone base centred 5 mm behind (275, 95), with two stops to hold the plate coating-out about 12° from vertical. A 5 × 67 × 7 toe keeps the lower edge from sliding. The rear plate support is 29 × 67 × 62 mm. Place a narrow dry lint-free wipe at the foot of the rest. |
| Finger covers and score tools | Cover gripper pads with clean nitrile finger cots. Align a ruler with the plate at the front of the chamber for calibration. Place a fixed camera outside arm reach where the wet boundary can be seen through the front glass. |

Start with the plate resting against the baffle. Close the lid and wait until the front reaches 82.5 mm from the bottom, as seen alongside the ruler. Start the trial at that physical state. Pre-development time depends on humidity and plate batch. Use the same TLC plate batch, saline concentration, and cover across a cohort. Start the arms in the reference ready pose with fingers open. Use a new spotted plate each trial and replace a damp wipe. Set the pencil out with about 1 mm of lead exposed.
[Supelco 50 × 100 mm TLC plate](https://www.merckmillipore.com/PG/en/product/mm/105789) · [DWK handled tank covers](https://www.dwk.com/na/kimble-kontes-tlc-developing-tank-pgr32131) · [Zebra axial lead protection mechanism](https://www.zebra.co.jp/global/products/del_guard/) · [Juniata food dye and saline method](https://more.juniata.edu/academics-classes/science-in-motion/labs-and-equipment/media/chemistry-files/TLC%20Food%20Dyes.pdf)

## Scoring

The extraction window is 86–91 mm from the plate's bottom edge, inclusive. Use the 100 mm plate outline to calibrate a view square to its coated face. From the first frame with the lower edge clear of the bath, measure the distance along the plate from the lower edge to the wet front. Repeat the ruler measurement on the dry plate to compare the surviving graphite mark with the recorded wet front. Either arm can handle either object.

- **0** while the plate remains in the bath, if it is returned to the bath, or if the plate is dropped, broken, or visibly smeared. In simulation the glass remains rigid and contact of the extracted plate with the bare bench is a drop or smear.
- **0.25** for removing the plate outside the extraction window.
- **0.55** for removing the plate within the extraction window.
- **0.80** for timely removal and either one continuous graphite mark across at least 11 mm of plate width or two edge ticks at least 32 mm apart. Every mark must be within ±2.5 mm of the wet front and made within 15 seconds after extraction.
- **1.0** for the preceding conditions plus a stationary plate with lower edge supported in the drying rest, coating toward the robot and no more than 34° from vertical, with both grippers at least 70 mm from the plate center for 0.75 seconds.

The simulator computes the evolving visible solvent front while the bottom is immersed. Native contact of the pencil tip with the coated plate loads the axial graphite cushion. The simulated trace represents the ensuing graphite deposit. The scoring tools in a real trial are the calibrated view and the finished mark. The reference reads rigid object poses to plan ordinary 14-dimensional YAM commands. One arm removes the cover and marks the boundary, while the other extracts and supports the plate. Model trials use RGB and joint state.

## Run

Use the accepted-license Isaac Lab-Arena runtime and install with `python -m pip install -e . --no-deps`. The action limit is 900 steps, 45 seconds at 20 Hz. The seed is `1415158438`.

```bash
python run.py --device cuda:0 --physics-device cpu --output /tmp/tlc-reference
python run.py --device cpu --no-cameras --policy hold --output /tmp/tlc-idle
python run.py --device cpu --no-cameras --policy unmarked --output /tmp/tlc-unmarked
```

Inspect Robots embodiment is `tlc_plate` and the scripted policy is `tlc_plate_scripted`. The runner records the three native camera views, compiled demo, diagnostics, and score receipt. The unmarked control holds the plate in the same position and sweeps the pencil above the wet front without touching it.

## Assets and licenses

The original assets are generated in `tlc_plate/builder.py` and `tlc_plate/usd_scene.py`. Call `create_assets(output_directory)` after launching Isaac Lab to export the physical asset and geometry inventory files in meter-scale USD and JSON. The repository MIT license applies. The robot and camera assets come from the pinned adapter.
[YAM license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [YAMLab license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)
