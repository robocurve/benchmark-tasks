# Take a 2x photo with an iPhone 16's Camera Control

An Apple iPhone 16 stands in landscape in a desk phone stand. Its screen faces the arms and its rear cameras face a
slice of cake. The Camera app is open at 1x, and Zoom is the setting the Camera Control last adjusted. The
instruction is only: **Use the phone's Camera Control to take a photo at 2x zoom.**

On iPhone 16 and later, zoom and shutter are one 17 mm sapphire button on the phone's edge. A light press shows
the zoom dial, sliding a finger along the control zooms, and a click (a full press) takes the photo. For a robot,
the hard part is force on one small button. It must press just hard enough to open the dial (about 0.5 N here),
slide about 6 mm along a 17 mm strip with no click (under 2 N), and then click. If it presses straight down, the
photo comes out at 1x. If it presses too hard while it slides, the click comes first and the photo is again at 1x.

## Importance and sources

- **Everyday operation:** 91% of US adults own a smartphone ([Pew Research Center, Mobile Fact Sheet, 2024](https://www.pewresearch.org/internet/fact-sheet/mobile/)).
  That most of them take and zoom photos with it is our inference, not a measured figure.
- **The real control:** Apple's iPhone User Guide documents the light press, the slide that adjusts the setting,
  and the click that takes the shot ([Use the Camera Control](https://support.apple.com/guide/iphone/use-the-camera-control-iph0c397b154/ios)).
  It was introduced with the iPhone 16 and iPhone 16 Plus ([Apple Newsroom](https://www.apple.com/newsroom/2024/09/apple-introduces-iphone-16-and-iphone-16-plus/)).
- **The physical knack:** force held inside a narrow band on one small button while sliding along it. The press
  forces are our estimates; Apple does not publish them.

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`
(`source ~/IsaacLab-Arena/yam.env`):

```bash
# scripted demonstration
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/take-2x-photo-with-iphone-camera-control/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Use the phone's Camera Control to take a photo at 2x zoom." \
  --scorer task_score --max-steps 900 --store-frames \
  --log-dir ~/yam-eval/take-2x-photo-demo

# model trial
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/take-2x-photo-with-iphone-camera-control/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Use the phone's Camera Control to take a photo at 2x zoom." \
  --scorer task_score --max-steps 1200 --store-frames \
  --log-dir ~/yam-eval/take-2x-photo-dream
```

The run prints `task_score`. Nothing is random: `object_jitter=0`, and no seed is read. The scripted demo is open
loop and one-armed. It reaches full credit at step 265 (13.3 simulated seconds), and the episode ends there. A demo
trial took 2.5 to 3 minutes of wall time on one GPU, start-up included. The run limit is 1,200 steps (60 s), about
four times the demo. In its 1,200-step run, Dream-1 reached its right gripper towards the phone and then drew
both arms back; it took no photo that scored, the phone stayed in its stand, and the score was 0.

## The phone and its Camera Control

- **Product:** Apple iPhone 16, US$799 at launch ([Apple Newsroom](https://www.apple.com/newsroom/2024/09/apple-introduces-iphone-16-and-iphone-16-plus/)).
  Size 147.6 x 71.6 x 7.80 mm and mass 170 g ([Apple tech specs](https://support.apple.com/en-us/121029)). The
  model is a rigid box at that size and mass. Its inertia is that of a uniform box.
- **Camera Control position:** from Apple's accessory dimensional drawing
  ([iPhone 16 PDF](https://developer.apple.com/download/files/accessories/dimensional-drawings/iphone-16.pdf)):
  centre 96.23 mm from the phone's top end, 2 x 8.55 = 17.1 mm long, on the side edge, about 3 mm across. In
  landscape with the cameras to the left, this edge is on top and the control is 51.4 mm from the right-hand
  end. The rear camera plateau and the two 16 mm lenses come from the same drawing (detail D).
- **Behaviour:** from Apple's iPhone User Guide
  ([Use the Camera Control](https://support.apple.com/guide/iphone/use-the-camera-control-iph0c397b154/ios);
  [Camera basics](https://support.apple.com/guide/iphone/camera-basics-iph263472f78/ios)). Apple says to "Slide
  your finger on the Camera Control to adjust the setting. The setting is saved and appears the next time you
  lightly press the Camera Control", and to "Click the Camera Control to take your shot". The tech specs list
  Zoom among the control's settings, and a 2x telephoto at 52 mm.
- **Sensor model (in code: `Phone.step`):** the cap is a slider on a 2,500 N/m spring, so its travel stands for
  the force on the control's sensor. A light press is 0.2 mm or more (0.5 N) but short of the click, held 2 control
  steps. The click is 0.8 mm (2.0 N), and it re-arms above 0.4 mm. Apple does not publish these forces; they are
  estimates. The touch is contact: the cap is pressed at least 0.04 mm and a fingertip is over it. The touch
  position is the mean position of that fingertip along the phone. A contact lost for under 0.15 s counts as the
  same touch.
- **Zoom (estimates):** while the dial is open, the zoom doubles for each 6 mm the finger slides towards the
  cameras' end, from 0.5x to 10x. The dial has detents at 1x and 2x: from 1.80x to 2.22x it reads exactly 2x. The
  dial closes 2 s after the finger leaves the control, and the zoom stays where it was.
- **Screen:** the viewfinder (a green-grey patch), the zoom dial with its marker (yellow at 2x), the zoom chip
  (yellow at 2x), a white flash at each photo, and the last photo's thumbnail. A policy can see all of these from
  the top camera.

## Scoring

The score reads the phone's latest photo and the phone's pose. A click takes a photo at the zoom the dial shows.
A photo is **spoiled** if, at the click, the phone moved faster than 2 cm/s or turned faster than 10 degrees/s, or
any fingertip point was within 1.5 cm of a rear lens's centre.

- **1.0:** the latest photo is unspoiled and at 2x. The phone rests in its stand, within 5 mm and 3 degrees of its
  start and moving under 1 cm/s and 3 degrees/s, and every fingertip point of both grippers (each finger's tip and
  a point 3 cm up it) is at least 3 cm from the phone. All of this holds for 1 s, and the score is then 1.0 for
  the rest of the trial.
- **0.75:** the latest photo is unspoiled and at 2x, but the full conditions are not met (for example, a finger
  is still on the control).
- **0.5:** the latest photo is unspoiled at 1.5x to 3.0x, other than 2x.
- **0.25:** the latest photo is unspoiled at any other zoom (for example, 1x after a straight press).
- **0:** no photo; or the latest photo is spoiled; or, before full credit, the phone was ever moved more than
  2 cm or turned more than 10 degrees from its settled start. That last record is kept across steps and cleared
  only in mechkit's `on_reset` hook.

Retakes are allowed: only the latest photo counts.

## Real trial: preparation, reset and measurement

- **Rig:** a dedicated iPhone 16 with no case, in a reclining aluminium desk stand (about 15 degrees, a silicone
  ledge with a lip) that is taped to the table so that the phone's centre is at (30, 0) cm. Mark the phone's
  lower corners on the ledge with tape. Put a printed millimetre scale behind the phone, and set up a 60 fps camera
  facing the phone's back edge. The gripper pads are rubber and the forces are under 3 N, so the glass needs no
  protection. Nothing is hot, sharp or wet.
- **Preparation:** in Settings > Accessibility > Camera Control, turn the light-press gesture on at the default
  pressure. Open Camera with a click on the Camera Control. Lightly double-press, slide to Zoom, and lightly press
  it, so that Zoom becomes the saved setting. Slide back until the dial reads 1x, and let the dial close.
- **Measurement (at the end, before anyone touches the phone):** read the phone's pose against the tape marks
  (within 5 mm). Then open Photos and the latest photo, and swipe up for Info: 52 mm is 2x; 39 to 78 mm (1.5x to
  3x) is the 0.5 band; any other focal length is 0.25. Check that the photo's time is after the trial's start. On
  the 60 fps video, the click is the frame where the shutter animation starts. The phone must not move more than
  0.3 mm (2 cm/s) between the frames either side of it, and no gripper may be over the lenses in that frame.
- **Reset (about 1 minute):** note the latest photo's time, or delete the trial's photos. Lightly press the
  Camera Control and slide back to 1x. Set the phone back on its tape marks. Nothing is consumed.

## Distinct from

| Nearby task | What it tests | Here |
|---|---|---|
| PR #2524, phone screenshot buttons (accepted) | two side buttons pressed at once | one button, three force levels, and a slide along it |
| PR #2534, pinch-zoom a map on a tablet (accepted) | two-finger zoom on a touchscreen | zoom by sliding along a pressure-sensing side button, then a click on the same button |
| PR #3362, phone panorama (pending) | turning a held phone about its camera | the phone stays in a stand; the skill is graded press and slide |

We know of no task in LIBERO, RoboCasa, BEHAVIOR or SimplerEnv that operates a phone's camera button. This was
not checked exhaustively.

## Simulation notes and honest limits

- **One arm is enough here.** The phone rests on the stand's ledge and back rest, and a press of under 3 N on
  its top edge does not move it (under 0.001 mm in the demo). Steadying it with the other hand is allowed but not
  needed. The sharpness rule therefore only matters for a careless push.
- The cap's collider stands 1.1 to 2.1 mm proud of the frame so that a rigid gripper pad presses it and not the
  frame. The real control is flush and is pressed by a soft fingertip.
- The phone's surfaces use friction 0.35, roughly that of glass and sapphire against a fingertip. This is lower
  than the station's rubber-like 1.2, which made the finger stick and jump. The stand uses 1.2.
- The scripted demo commands the finger 2.5 mm into the cap for the light press, 1.2 mm while it slides 7.5 mm,
  and 8 mm for the click. The arm's own compliance turns these commands into roughly 0.7 N, 0.4 N and the full
  2.5 N stop. The finger lags the command by about 1.5 mm, so it really slides about 6 mm.

## Assets and helpers

Phone, cap and stand are authored parts (mechkit meshes and colliders). The cake (photo subject) and the two
distractors are NVIDIA SimReady props. They are downloaded at first run from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) at pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`food_prepared_cake_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_prepared_cake_a01):
  the photo's subject, loose, mass set to 0.10 kg.
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01):
  distractor, loose, mass set to 0.30 kg (convex-hull collision at load).
- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01):
  distractor, loose, mass set to 0.03 kg (any joints are welded into one body by the loader).

`score()` ignores all three. No meshes or textures are bundled. `simready.py` is adapted from the open-source
`cutters-off-pegboard` loader. `mechkit.py` is the shared mechanism kit, copied unchanged from `heat-shrink-lug`
(another team's open-source task).
