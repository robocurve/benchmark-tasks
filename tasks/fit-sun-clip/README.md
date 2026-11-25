# Fit a four-hook sun clip to folded glasses

Mount the tinted sun clip on the glasses. Cover both clear lenses and catch the upper and lower hooks around each outside rim. Let go. The sunglass clip is a separate accessory with an extending spring bridge. It adds sun protection while the wearer keeps their own lenses.

## Objects and reset

Place a rectangular acetate frame (120 mm across, 38 mm from top to bottom) on a flat 145 by 172 mm microfiber mat, 8 mm thick. Fold the temples and lay the clear lenses facing up. Place the spring clip 150 mm in front, on a separate 25 mm high pad of folded cloth that supports its bridge and leaves both outer ends exposed. Both tinted lenses face up, the four return hooks face down, and the bridge is relaxed. The clip is centred 7 mm left of the frame. Tape the cloths to the worktop. The glasses and clip are loose. Lift the clip off the frame, let the spring bridge relax and return the two loose pieces to their cloths with their lenses facing up.

The modeled acetate front is 120 by 38 mm, with a clear optical insert of 49 by 31 mm per eye, 64 mm between optical centers and 4 mm rim thickness. Each tinted panel is 55 by 40 mm. A candidate frame is the rectangular full-rim acetate Framesbuy Dexter 1114-001, size 49-15-135. The supplier confirms the eye width and bridge. Measure the actual front width, height, clear insert height and rim thickness before fitting a clip. [Dexter frame specifications](https://www.framesbuy.com/details/dexter-1114/1114-001).

For a four-hook clip, try the Visionaries Rectangle REX 52, ordering code VCOREC-52. Its printed outline is 118 mm overall with 52 by 39 mm panels. Also compare the REX 54, ordering code VCOREC-54, at 124 mm overall with 54 by 40 mm panels. These bracket the modeled 120 mm frame width. VS Eyewear gives a 2 mm per-eye width and height fitting guideline and a 4–5 mm maximum rim-edge thickness for its four retaining hooks. Compatibility with an individual frame requires a physical fit check. [Rectangle product and dimensional guideline](https://vseyewear.com/products/visionaries-clip-on-polarized-sunglasses-rectangle) and [four-hook thickness and attachment guide](https://vseyewear.com/pages/visionaries-size-chart).

Print the Rectangle outlines at 100% and verify the printed dimensions with a ruler. Lay the actual frame on both outlines. Compare eye width and height including the rim, bridge spacing and full outside width. In a dry fit, engage one side, extend the bridge, lower the opposite hooks behind their rim and release. Use a pair only when the spring holds with both hands away, both clear inserts are covered and all four returns visibly overlap the back. Select another size or frame when necessary. Reset the two verified pieces on their separate cloths before scoring. [Rectangle full-size fitting outlines](https://clipons.com/wp-content/uploads/2023/08/FITTING_GUIDE_REC.pdf).

The rectangular clear polycarbonate lenses and tinted clip have distinct optical surfaces. Each bent metal hook and both folded temples have collision geometry. A passive spring acts between the two tinted sections. The taped cloth pads are represented by rigid blocks with friction. The optical power, temple folding and minor flexibility of the frame and hooks are omitted.

## Scoring from the video

Check the final second using the overhead view and a low close-up of each outer rim. The folded glasses must rest on their mat. Both hands stay clear, and the clip stays clear of both cloths. For each side, verify that the tinted panel covers its clear lens, the two hook shanks pass outside the rim, and BOTH pale return tips overlap the far face of that rim. One fully secured side earns 0.5. Two fully secured sides earn 1. A clip perched on top, or one supported by a hand or cloth, earns zero. Earlier attempts may be retried.

Hands may stay close to the eyewear after letting go. The hand-related score uses the six measured physical contact pairs between either robot hand and the two tinted panels or the glasses. The referee can evaluate release from any visible hand position. The robot must transport the loose clip, extend it, align the four returns and lower them behind the rims. The spring then retracts to retain the alignment.

View the original video at full size. In the overhead inset, trace the clear optical insert inside each brown acetate rim and check that the tinted contour covers it. Each low inset shows the two pale metal tips for one outer rim. Follow each visible upright shank down from the tinted lens, past the outside of the brown frame, to its inward-facing pale toe. The toe must sit behind the brown rim and overlap it. A toe resting on the upper face can look aligned from overhead. Use the main wide view to verify that both grippers have withdrawn. The fourth inset gives an oblique view of the loose frame and its folded temples on the mat. In a matching real trial a small pale removable mark on each hook tip can make the returns easier to see.

| Final second | Score |
| --- | --- |
| Neither side has both returns secured and its clear lens covered, or a hand or cloth supports the clip | 0 |
| Exactly one side has both returns secured and its clear lens covered | 0.5 |
| Both sides have their two returns secured and both clear lenses covered | 1 |

A final score requires 20 consecutive control samples. The reference's first interval meeting all physical conditions starts at 31.35 seconds. During the final second the smallest coverage margins are 0.66 mm on the left and 0.82 mm on the right. The two left returns extend at least 1.50 and 2.96 mm behind their rim. The two right returns extend at least 4.44 and 5.89 mm behind theirs. Physical edge contact can produce up to 0.4 mm of simulator contact tolerance at a shank or lens front. In this full-credit episode the four shank inner faces are between 1.63 and 2.41 mm outside the respective rims. The four pale lips overlap the rear rims by 1.45 to 2.50 mm.

## Reference and model views

The reference operates the ordinary dual YAM arms through joint-position and jaw-opening commands. The loose frame and the two connected clip sections respond to gravity and surface friction. A passive spring retracts the bridge. The reviewed videos include the stock overhead and wrist images with four additional fixed views for the human evaluator. Dream-1 receives the ordinary policy observation. The model receives no additional referee images or score measurements.

- Lens coverage camera at (0.354, 0, 0.316), aimed at (0.405, 0, 0.025), focal length 34.0 mm.
- Left returns camera at (0.337, 0.177, 0.062), aimed at (0.405, 0.055, 0.022), focal length 43.0 mm.
- Right returns camera at (0.337, -0.177, 0.062), aimed at (0.405, -0.055, 0.022), focal length 43.0 mm.
- Hands and frame camera at (0.182, -0.18, 0.14), aimed at (0.402, -0.006, 0.026), focal length 38.0 mm.

## Recorded evaluation

Dream-1 run: 0 (800 steps); demo: 1.0 (800 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 419.55 seconds for the demo and 292.73 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```

An earlier Dream-1 session expired. The submitted model video records one complete new trial from the same initial scene, seed and control horizon.
