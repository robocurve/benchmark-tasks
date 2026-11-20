# Technical reference

Dimensions and material values below distinguish published figures, measurements reported by sources, and estimates. Product pages identify the physical objects; no manufacturer assets are bundled.

## The toy, and where its numbers come from

The object is the classic full-size Etch A Sketch (Spin Master; the red-framed one with two white knobs). No
manufacturer drawing is published. The sizes below were measured on a full-size Etch A Sketch by a Fab Academy
student who built a machine to turn its knobs, and published on the course page
<http://fabacademy.org/2018/labs/fablabdassault/students/greg-buckland/exercise14.html> (read on the page).

| Quantity | Value used | Source | Status |
| --- | --- | --- | --- |
| Frame underside | 227 x 186 mm | Fab Academy page ("Underside: 186 x 227mm") | measured |
| Frame height | 23.7 mm, 26.6 mm at the raised lip | same ("Height: 23.68mm", "Height incl. Ridge: 26.58mm") | measured; the lip is taken to run round the screen, 4 mm wide (estimate) |
| Corner radius | 7.8 mm | same ("Fillet corner radius 7.8mm") | measured |
| Frame borders round the screen | 32.8 mm at the sides and top, 38.6 mm at the bottom | same | measured; screen 161.4 x 114.6 mm follows |
| Knob | 28.15 mm across, 12.32 mm tall, top 40.42 mm above the table | same ("Knob diameter: 28.15mm", "Knob height: 12.32mm", "Base (incl. ridge) to top of knob: 40.42mm") | measured |
| Knob position | 4.76 mm from the side edge, 4.03 mm from the bottom edge (so the centres are 189 mm apart, 18.1 mm from the near edge) | same | measured |
| Stylus travel | 155 x 108 mm | same ("X travel: 155 mm", "Y travel: 108 mm") | measured; taken as centred in the screen |
| Line per knob turn | 35.5 mm | same ("360 deg = ~35.5 mm travel", "~10 deg turn per mm") | measured |
| Knobs keep turning at the travel ends | strings slip | same ("knobs turn past their limits") | measured |
| String slack after a reversal | 20 degrees | Cornell ECE 4760 project, 2009, <https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/s2009/kk383_sl486/kk383_sl486/index.html> ("for about the first 20 degrees of the knob's rotation in the new direction"); EtchABot, <https://www.geekmomprojects.com/etchabot-a-cnc-etch-a-sketch/> (backlash correction of 100 to 140 steps of a 2048-step motor turn, 18 to 25 degrees) | read on the pages |
| Mechanism | knobs drive the stylus by cables over pulleys; left knob horizontal, right knob vertical; aluminium powder on the inside of the glass | <https://en.wikipedia.org/wiki/Etch_A_Sketch> | read on the page; also the 1960 launch, over 100 million sold and the Spin Master ownership |
| Which way a knob moves the line | left clockwise: right; right clockwise: up | as the toy is commonly described | estimate; a mirror-image toy would only flip the knob directions, not the task |
| Mass | 454 g | retailer listings of the classic (1 lb; other listings 567 to 635 g, probably boxed) | seen in search-result summaries of retail listings; estimate |
| Knob turning torque | 0.03 N m | no published figure. A 2023 Cornell project (<https://courses.ece.cornell.edu/ece5990/ECE5725_Spring2023_Projects/3%20Saturday%20May%2013/8%20Etch%20a%20Sketcher/Etch-a-sketch/index.html>) found small stepper motors "did not have enough torque" and stalled; the knobs are known to be stiff | estimate; the demonstration's grip can hold more than ten times this |
| Split of the mass, friction | frame 438 g, knobs 8 g each; plastic on wood 0.40, knob on the rubber pads about 1 | assumed | estimate |

No product image is included in this folder, and the maker's printed name is not drawn on the frame.

Station coordinates (mm; x forward from the line through the arm bases, y left, z up from the table): the toy's
centre is at (380, 0), its long sides along y, knobs toward the robot. Knob centres (305.1, +94.7) and
(305.1, -94.7), tops 40.4 above the table. The start dot is at (345.6, +50.7): 30 mm from the screen's left
edge and 20 mm from its bottom (robot-side) edge. The triangle's other corners are at (345.6, -29.3) and
(405.6, -29.3). A point (X right, Y up the screen) of the drawing, in mm from the dot, is at
`x = 345.6 + Y`, `y = 50.7 - X`.



## What the model sees

The policy gets the overhead camera, a camera on each wrist (all 320 x 176 RGB) and the joint and jaw positions.
In the overhead view one pixel is about 5 mm of table: the red frame (227 x 186 mm, about 45 x 37 pixels), the
silver screen and both white knobs are in view from the first frame, and the drawn line is a dark band 3 mm
wide, under a pixel but darker than the silver screen around it, 16, 12 and 20 pixels long for the three sides.
The wrist cameras see the knob under each gripper and the nearer part of the screen. A review-only camera
(1280 x 704, never a policy input) looks down at the screen and both knobs from in front.



## How the drawing works in the simulation

The knobs are real articulated bodies turned only by finger contact. Each knob's angle is read every control
step (unwrapped over many turns) and passed through the stylus rule in `stylus.py`: a 20 degree dead band
(the string slack), then 35.5 mm of stylus travel per turn, stopping at the ends of the 155 x 108 mm travel.
The stylus is behind the glass and touches nothing. The line is drawn where the stylus has been, as a dark
ribbon 3 mm wide on the screen (`skin.py`); this rule stands in for the powder being scraped off the glass, a
material response. The scorer reads the stylus path itself.



## Licences

All code and the generated scene are under the MIT licence in `LICENSE`. No third-party assets are used: the toy
is built from boxes, cylinders and generated meshes.


## Full simulation fidelity statement

Rigid bodies in Isaac Lab-Arena (PhysX, 240 Hz physics, 20 Hz control) at the toy's measured size. The toy is one free articulation: the frame (a box collider with four lip colliders round the screen, on the table by friction) and two knobs on revolute joints about the vertical with no stops. Each knob is a plain cylinder 28.2 across and 12.3 tall that the grippers hold by real finger contact; nothing attaches a gripper to a knob, and the knobs turn only when the fingers turn them. A knob resists turning with a drag capped at 0.03 N m (an estimate: no figure is published). Inside the real toy each knob winds strings that pull the stylus along two rails; here that drive is a kinematic rule: each knob angle passes through a 20 degree dead band (the string slack) and moves the stylus 35.5 mm per turn, and at the ends of its travel the stylus stops while the knob keeps turning, as on the real toy. The drawn line is a stated rule standing in for the aluminium powder being scraped off the glass (a material response): every stylus position is drawn on the screen as a dark line 3 mm wide, wider than the real line (under 1 mm), so that it shows in the 320 x 176 overhead view; the scorer uses the stylus path, not the drawn width. Erasing by shaking is not modelled (lifting or tilting the toy scores 0). What the cameras see are display meshes with no collision: the red frame with rounded corners and its lip, the silver-grey screen and ribbed white knobs with a small mark on top; the maker's printed name is left off. Measured figures: frame and knob sizes, knob positions, screen borders, stylus travel and 35.5 mm per turn; the 20 degree slack is from two published knob-driving projects; the 454 g is from retail listings; the turning drag, the split of the mass, friction values and the knob turning directions are estimates (see README).

## Photo measurement

The judge uses one final image and the motion/release verdict from the trial video. The capture geometry, marker centres, command and output legend are in [README.md](README.md#score-a-real-trial). Use matte labels with their top edges parallel to the top screen edge. Centre the lens over the screen midpoint (50.7, 37.3) mm from A. At the nominal 300 mm lens height, the labels are 298 mm from the lens along the screen normal. Mount the labels within 0.5 mm of their specified positions on a flat surface at that height. Include the whole screen, avoid glare and keep it clean before the trial. The overlay goes on only for manual counting.

1. **Registration.** OpenCV decodes DICT_4X4_50 IDs 0–3 and refines the four black-square corners of each to subpixel precision. Project the known marker corners onto the drawing plane by scaling about the lens centre by 300/298. Fit one least-squares homography to all 16 correspondences. Reject missing, duplicate or unexpected IDs, edges under 72 pixels, or a maximum corner residual over 0.5 mm. Bilinearly rectify at 10 pixels/mm. Use the screen minus a 1 mm rim to exclude the bezel. This includes the entire 155 × 108 mm reachable area and therefore includes disconnected stray marks.
2. **Line extraction.** On the rectified 8-bit grayscale image, take a 3 × 3 median filter and compute Otsu's threshold over the whole crop. Ink pixels have intensity at or below the lower of that threshold and the screen median minus 20. The contrast floor keeps an empty, evenly lit screen from being split into ink and background. Zhang–Suen thinning gives a one-pixel centreline. If thinning entirely removes a component, keep its original pixel nearest its centroid. The annotator shows what entered the grader.
3. **Shared geometry.** Walk all adjacent skeleton pixels, including isolated dots. Omit diagonal shortcuts around orthogonally connected corners. Pair the straightest continuations at junctions, breaking ties by neighbour order from top left to bottom right. Process open trails before loops in raster order. Reset the grader's step anchor between trails so gaps add neither coverage nor step length. Each centreline sample goes through the same `TriangleGrade.sample` used by the simulator. The simulator interpolates each control-step segment at intervals ≤0.25 mm. Photo samples are at most √2/10 mm apart. Both call `drawing_result` for the tiers and stray cap.
4. **Bins and steps.** Project each sample onto each finite side, clamping to its ends. Within 2 mm of that projection, credit bin `ceil(distance_along_side) − 1` clipped to the valid indices. Any sample farther than 2 mm from all three sides sets the stray flag. On CA between 5 and 95 mm inclusive, keep an anchor at the start of a run. Once the larger of |ΔX| and |ΔY| reaches 0.2 mm, a displacement outside both ±10° axis windows starts a new run at that sample. Otherwise record the larger displacement as run length. A maximum strictly over 1 mm bars full credit. The grader compares unrounded measurements and rounds only its reported diagnostics.

The image gives geometry at the camera's resolution. A merged crossing has no visible drawing order, and an ink edge close to a cutoff can move a centreline sample by a pixel. Preserve the original image, JSON and annotated result so another judge can reproduce that measurement. The real-trial video supplies the motion and final-contact checks. The simulator continues to use its logged pose and contact history. The photo tests use synthetic captures, including the scripted controller's kinematic dry-run path rendered at physical stroke width.
