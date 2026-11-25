# X-ACTO KS 1031 manual sharpening protocol

Use a stock X-ACTO KS 1031 (1031LMR) manual pencil sharpener and a fresh blunt yellow hexagonal wooden pencil, 140 mm long and 7 mm across its corners. Fit the shavings receptacle, secure the sharpener to the tabletop using its stock three-screw mount, and select the standard 5/16 in guide opening. Use the front opening while feeding the pencil and the rear crank while turning clockwise, as viewed from the handle side. The manufacturer's product page and demonstration show the KS model, eight-position dial and hand crank. [X-ACTO product page](https://www.xacto.com/SAP_1031LMR_PR.html) · [X-ACTO demonstration](https://www.youtube.com/watch?v=tXmSpx6C4z0) · [1031LMR aperture and mount specifications](https://www.shifflerequip.com/x-acto-ks-8-hole-manual-pencil-sharpener/)

Locate the front mouth at (0.215, −0.040, 0.821) m in the YAM station frame. Mount the sharpener with its feed axis angled 30° clockwise from +X as viewed from above. The front opening faces the left arm and the rear crank sits toward the right arm. Lay one flat side of the pencil directly on the tabletop with its centre at (0.140, 0.120, 0.75053) m and its shaft parallel to the feed axis. A flat tape outline may indicate the starting position on a physical table. Turn the crank until its handle is below the axis. Start both arms in READY with open grippers. Reset with a fresh pencil, an empty receptacle and the same dial position.

## Score a physical trial

Before a filmed trial, sharpen three identical blunt pencils by ordinary hand feeding on the same unit. Measure each intact point from the beginning of continuous exposed wood along the axis to the tip. Use the median as the reference taper, and record the measurement with the close-up video. To observe feed, put a thin witness band 40 mm from each end of the trial pencil. With the crank still, gently insert a blunt end until it first contacts the cutter and record the witness band position against a millimetre scale fixed to the housing. Return the pencil to its starting position on the bare table. Film the pencil and scale side-on, keep the crank in view, and record a close-up of the resulting point alongside a ruler.

Check that the witness band moves inward at two or more points during clockwise cutting and travels at least 1 mm beyond first contact. Measure the resulting taper on the same end to the nearest millimetre. Use the highest tier whose conditions are satisfied. Empty cranking and turning at a single fixed insertion depth earn zero.

| Score | Visible measurement |
| --- | --- |
| 0.25 | Inward advance during clockwise cutting and a new taper at least one-third of the reference length. |
| 0.50 | Inward advance during clockwise cutting and a new taper at least two-thirds of the reference length. |
| 0.75 | Inward advance during clockwise cutting and a reference-length or longer taper with an intact exposed graphite point. |
| 1.00 | Meets the 0.75 tier. The pencil is withdrawn, released flat on the tabletop and still for 1 s with both hands over 100 mm from its midpoint. |
| 0.00 | Below the first tier, a broken pencil or a pencil that falls off the tabletop. |

## Score a simulated trial

The included geometry specifies the approximate housing, knob and crank envelope and the fixed mount. The 7.94 mm standard hole corresponds to the stock 5/16 in detent. The simulation preselects that detent. The front dial, base and crank remain part of the scene throughout a trial. The cut surrogate has nominal taper thresholds of 5, 10 and 15 mm. The full-score release checks the entire pencil footprint, tabletop height, near-horizontal orientation, sustained stillness and both hands clear.

The native cutter face moves axially on a 780 N/m spring. Loaded clockwise rotation can increase the cut by up to 2.4 mm per loaded turn. Insertion beyond first contact supplies the compression and limits total material progress. The face retreats as the taper grows, so continuing to cut requires further insertion under load. Counterclockwise rotation subtracts from the angular progress. Each pencil end has a separate taper and angular history.

The model admits 1.4–5 mm cutter compression, corresponding to about 1.1–3.9 N, and caps crank drag at 0.08 N m, approximately 2 N at the nominal 40 mm crank radius. These are simulation parameters. The physical reference procedure measures the stock tool's taper. Housing dimensions are estimates based on the stock envelope and product views. The render depicts newly exposed wood and graphite. The contact solver retains the original pencil collider and omits individual chips and cutter teeth.

## Recorded comparison

The progressive-feed demo scored 1.00 in 63.10 simulated seconds. Pose replay measured 16.02 mm of axial advancement during cutting, 15.98 mm of cutter-face retreat, and a 15.00 mm taper. The sharpened pencil stayed still on the tabletop with both hands clear for one second.

A control used the same scene and crank path with the left wrist's axial target held fixed after initial seating. It completed 10.99 net clockwise turns and produced a 0.91 mm taper, scoring 0.00. The pose replay reproduced both scores.
