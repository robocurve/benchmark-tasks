# Build the optical fingertip and run the recorder

This build keeps the benchmark's 42 × 36 × 42 mm grasping envelope, flat
26 × 30 mm sensing face and 8 mm ball. It uses a commercial USB camera inside
a dark housing. A reflective silicone face deforms with the ball. The camera
views that deformation through the clear backing and gel under fixed RGB
illumination. The recorder runs on an existing Linux host.

## Parts

| Quantity | Part | Specification |
| --- | --- | --- |
| 1 | Camera | [Arducam B0520][camera], color USB UVC, manual M12 focus, metal body 19.5 × 15 × 17.3 mm. Stock 3.7 mm lens, 640 × 480 at 30 fps MJPEG. The supplier lists the module at about US$65. |
| 1 | Housing | Matte black PA12 or PETG, cut or printed from the rectangular dimensions below. Also use black 3 mm blocks, M2 screws and light-tight tape. |
| 1 | Optical backer | Clear cast PMMA, 34 × 31.6 × 2.0 mm. Use a 26 × 30 mm piece of clear [3M 9731][tape] silicone-to-acrylic adhesive film. |
| 1 | Gel cartridge | [Smooth-On Solaris][solaris] A and B with [Slacker][slacker]. The published [gel preparation][recipe] uses an A:B:Slacker mass ratio of 1:1:4. |
| 1 | Reflective face | [Smooth-On Psycho Paint][paint] A and B, white and black [Silc Pig][pigment], smooth disposable applicator, 0.10 mm shims. |
| 3 | Directional lights | [Kingbright WP154A4SUREQBFZGC][led] common-cathode RGB LEDs. Use only one color die on each part, with a 330 Ω resistor for red and a 220 Ω resistor for each of green and blue. |
| 1 | Wiring set | 5 V USB supply or breakout for the lights, three ¼ W resistors, fine flexible wire, 5.2 mm LED sockets, and the camera USB cable. |

Budget about US$200–300 in parts and small material packs, excluding the host,
robot and shared fabrication tools. Each material pack supplies multiple pads.
The housing uses simple rectangular cuts with two openings. A small scale,
caliper, hand tools and a level acrylic mold are sufficient for the build.

## Housing and optics

Use millimetres. The origin for the dimensions below is at the center of the
housing bottom, with `x` across the 26 mm gel dimension and `y` across the
30 mm dimension. Orient the lid's `+x` and `+y` witness marks to agree with
the task's pad frame. Keep the exterior grip faces flat.

1. Make a 42 × 36 × 2 mm bottom plate. Fit two 2 × 36 × 36 mm end walls
   and two 38 × 2 × 36 mm side walls above it, producing a 38 × 32 mm clear
   cavity from `z=2` to `z=38`. Place a cable exit in the short end wall near
   the base. Chamfer the exit and fasten the camera cable to the base inside
   the case. Keep a slack service loop between the sensor and the table.
2. Seat the center of the B0520 lens axis over `(0, 0)`, looking up along `+z`.
   Fasten the camera body to the base with a clip or a removable thin pad of
   double-sided mounting tape. Rotate the camera body so its wider image axis
   covers the 30 mm pad dimension. With the supplied chassis on the 2 mm base,
   the lens end is nominally near `z=19`. The face is about 23 mm above it.
   Loosen the M12 lens lock and turn the lens to focus at the actual gel face.
   Re-lock it after checking the reference image below.
3. Place the emitting tips of the red, green and blue LEDs at `(12.5, 0, 26)`,
   `(-6.25, 10.8, 26)` and `(-6.25, -10.8, 26)` respectively. Aim each toward
   `(0,0,42)`, about 38° from the face normal, and hold it in a 5.2 mm hole in
   a small black block bonded to the nearest wall. Mask the lower half of
   each socket to shield the camera from direct light. Wire each selected
   anode to 5 V through its own resistor and join the cathodes to supply ground.
   Leave unused anodes open and insulate the joints. Run the lights continuously
   through the trial. Bundle their flexible supply leads with the camera cable.
4. Fit four 3 × 3 mm corner posts from `z=2` to `z=36`, centered at
   `(±17.5, ±14.5)`. Chamfer the four backer corners by 2 mm to clear the
   M2 screws, then seat the backer on the post tops with the gel face up.
   Fit a 42 × 36 × 4 mm lid frame with a centered 26.4 × 30.4 mm cutout and
   M2 clearance holes over the posts. Seat it on the wall tops and secure
   it with four countersunk M2 × 8 mm screws to retain the backer edges.
   Keep its inner edge clear of the gel and seal the housing seams against
   ambient light. The outer lid and contact face are around `z=42`.

## Cast one replaceable pad

1. Make a clean 26 × 30 × 4 mm cavity with a flat polished PMMA base and
   perpendicular removable walls. Use the polished side for the contact face.
   Mix 2.00 g Solaris A, 2.00 g Solaris B and 8.00 g Slacker, scrape the bowl
   and stir slowly. Remove visible bubbles before pouring, using a vacuum
   chamber if needed. Fill level with the 4 mm mold edge. Cure for 24 hours
   at 23 °C, then demold. Reject pads with trapped bubbles in the image area,
   tears, or a face thickness outside 4.0 ± 0.2 mm.
2. Within two days of demolding, mix 2.00 g each of Psycho Paint A and B with
   0.10 g white and 0.02 g black Silc Pig. Stir pigments into B before combining
   A and B. Draw a thin uniform undiluted coat over the smooth contact side
   with a clean blade supported by 0.10 mm shims. Cure flat at 23 °C for at least
   two hours and until tack-free. Use ventilation and the maker's handling
   guidance. Inspect that the coating is uniform and blocks outside light.
3. Laminate fresh 9731 onto the centered backer area, acrylic adhesive toward
   PMMA. Apply the unpainted gel side to the silicone adhesive with one edge
   first to avoid trapped air. Measure the four 0.8 mm black fiducials on the
   top coat at `(-12,-13)`, `(12,-13)`, `(12,13)` and `(-12,13)` using a
   1:1 paper or acetate marking template. Check the 24 mm and 26 mm spacings
   with a caliper. Make the dots with black-pigmented silicone paint and allow
   them to cure. Keep markings outside the contact regions. Re-use the backer
   and replace the cast and laminated pad when the surface is scratched or worn.

## Prepare and score a physical trial

Install Python and the two recorder packages on the host. The recorder is
independent of Isaac Lab and connects only to the fingertip camera.

```sh
python -m pip install -r physical/requirements.txt
v4l2-ctl -d /dev/video0 --list-ctrls-menus
python physical/record_impressions.py --device /dev/video0 --snapshot /tmp/fingertip-reference.png
```

Choose manual exposure, fixed gain and fixed white balance in the camera's
listed V4L2 controls before taking the snapshot. For example, use
`v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1,white_balance_automatic=0`
if the device lists those controls and `1` selects manual exposure. Set
the absolute exposure and gain so the unloaded image has no clipped channels.
Keep the three LEDs on for
both reference and trials. Position and focus the camera until all four dots
are sharp and fully visible, including their surrounding gel. When viewed
from the camera, the point order can be mirrored relative to the top view.
Read their pixel centers from the snapshot in the physical order above and
save the measured values as `calibration.json`. This example shows the format
and uses illustrative coordinates.

```json
{"camera_pixels": [[205, 101], [440, 103], [438, 370], [206, 369]], "difference_threshold": 14}
```

At fixture commissioning, check that an unloaded pad stays below the threshold
when the housing moves under room lighting. Press the actual 8 mm ball by hand
into each region on a spare check run, then release. Verify that the in-focus
image contains one bounded, approximately circular color-change patch. Use
`--diagnose` with the scored capture command below to log each detected patch's
diameter and center in `detected_patches`. Use the four fiducials to verify the
image scale and a slow trial indentation to verify that the 4.7–6.5 mm interval
is attainable at all three sites. Adjust the fixed exposure and lighting if
needed. Choose the difference threshold once for that build, then retain it
for all trials with that pad and lighting. Recalibrate after changing the pad,
lens position or image resolution.

Set the housing and probe at the positions in the [bench reset](../README.md#physical-setup).
With the pad clean and unloaded, run the recorder and start the robot when it
prints `Armed`. The host can use that message as the trial-start signal.

```sh
python physical/record_impressions.py --device /dev/video0 \
  --calibration calibration.json --output /tmp/fingertip-trial-01
```

The recorder takes a 30-frame unloaded median and opens a 31 s scoring window.
It rectifies images to 10 px/mm, measures the center and area-equivalent diameter
of a single rounded patch, and uses the [V4L2 frame timestamp][timing] supplied
by OpenCV. A region qualifies after continuous valid frames span at least
0.400 s. Loss of contact, a move out of tolerance and a frame gap over 75 ms
break the hold. The script saves a raw camera image, a rectified image and the
timestamped qualifying frame sequence for each region once. `report.json`
records each measured center, diameter and timestamp and the score. The host
automatically scores zero, one third, two thirds or one according to how many
distinct regions qualified. A new run takes a new unloaded reference and
writes to a fresh output directory. For audit replay, pass `--video` with a
timestamped optical-camera recording instead of `--device`.

The recipe borrows the transparent gel and directional-lighting principles
from published optical tactile work. The enclosure cut list and recorder are
original. The benchmark's supplied demo and Dream-1 video use the simulated
sensor. Hardware assembly and camera commissioning are the real-trial steps.

[camera]: https://www.arducam.com/arducam-2mp-low-light-usb-camera-module-with-m12-pinhole-lens.html
[solaris]: https://www.smooth-on.com/products/solaris/
[slacker]: https://www.smooth-on.com/products/slacker/
[recipe]: https://arxiv.org/html/2506.15087v1
[tape]: https://www.3m.com/3M/en_US/p/dc/v000364683/
[paint]: https://www.smooth-on.com/products/psycho-paint/
[pigment]: https://www.smooth-on.com/products/silc-pig/
[led]: https://www.digikey.com/htmldatasheets/production/1046351/0/0/1/wp154a4sureqbfzgc.pdf
[timing]: https://github.com/opencv/opencv/blob/4.13.0/modules/videoio/src/cap_v4l.cpp
