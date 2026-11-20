# Close a hardback notebook whose cover springs open and fasten it with its elastic band

A hardback pocket notebook with an elastic closure band (the Moleskine Classic Large format: 13 x 21 cm, hard
covers with rounded corners, a 5 mm elastic band sewn into the back cover) lies on the desk. Its front cover stands
open 70 degrees: the notebook is new, and its stiff spine springs the cover back up whenever it is let go. The elastic band lies
stretched across the first page, where it snaps when the cover is opened with the band still on. The instruction
asks for the notebook to be closed and fastened shut with its band, and then let go; it does not say how.

The task is the task file [`close_a_notebook_with_its_elastic_band.py`](close_a_notebook_with_its_elastic_band.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

Notebooks with an elastic closure band are everywhere: in bags, on desks, in meetings. The retailer's description
of the Moleskine Classic Large hard cover reads: "Each notebook features an undeniably stylish hard black cover with
rounded corners, an elastic closure & folding pocket for notes (on the inside back cover)" and "Dimensions: 13 x 21cm"
([Cowling & Wilcox, Moleskine Classic Large][cw]). Closed, the band runs from the back cover's head edge up over the
head of the book, along the front cover near the fore-edge, and down over the tail to the back cover again; its
pull holds the cover shut.

Closing such a notebook is an everyday two-hand job, and the reason is visible in the scene:

- The band lies on the pages, under the open cover. Shutting the cover first shuts the band inside the book, where
  it does nothing. So the band has to be drawn out past the fore-edge and held there while the cover is shut.
- The cover springs back open the moment nothing holds it, so it has to be held shut until the band is across it.
- One hand therefore holds the cover shut (and with it the light book still) while the other stretches the band up
  over the fore-edge and lays it across the front cover. Let go too early, the band snaps back onto the pages or
  round the fore-edge.

The hand skill is handling a thin elastic: pinching a 5 mm band off a flat page, drawing it out under its pull
without dragging the 0.3 kg book along, and stretching it up high enough that its ends ride over the cover's corners
at the head and the tail before it is laid down.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the notebook and its band are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a hinge. The
notebook (back board, page block and spine as one body; the front cover on a sprung hinge) and the band (a chain of
short rigid links on joints that turn freely and stretch on a spring) are built with `bodykit.py`, which adds such
bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and
gives the task file each body's pose. This copy of `bodykit.py` also lets a body carry a second joint (the band is
sewn to the book at both ends) and lets a ball joint stretch along any of its start axes (the band's links).

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Notebook, loose, 0.30 kg in all, 21 x 13 x 1.7 cm, lying back cover down at x 12 to 33, y -6.5 to 6.5: its head
  (top) away from the robot, its fore-edge toward the robot's right (y -6.5), its spine toward the left (y 6.5).
  Back board 2.5 mm thick (red cloth over board), page block 12 mm thick (ivory), 3 mm in from the boards at the head,
  tail and fore-edge; the boards' fore-edge corners rounded to 8 mm. Front cover 21 x 13 cm x 2.5 mm, 30 g, on a hinge
  along the spine (1.6 cm up, at y 6.25); it stands open 70 degrees and springs back to 70 degrees when let go
  (0.15 N m per radian, 0.18 N m when shut: about 3 N pressed 6 cm from the spine). Shut, it lies on the pages.
- Elastic closure band, black, 5 mm wide, 226 mm long unstretched, 80 N per metre of stretch (estimate). It is sewn
  into the back board at its tail and head edges, 1 cm in from the fore-edge (x 11.8 and 33.2, y -5.5). At the start
  it runs up the tail and the head and lies across the first page along y -5.5, stretched to 25 cm (2.0 N), where it
  snaps when the cover is opened with the band on. Fastened, it runs up over the tail, along the shut cover at y -5.5
  and down over the head, at about the same stretch.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the poses of the notebook, its cover and the 16 links of the band, and the points of
each finger's collision hulls.

- The cover is shut when it is within 3 degrees of the pages (its free edge within 7 mm of them). It springs open
  whenever nothing holds it, so a cover that stays shut with no hand on it is held by the band.
- The band is on the cover when its middle link lies on the front cover's face (no more than 4 mm above it) and at
  least 3 mm in from the cover's fore-edge.
- Released and still: every finger point 3 cm or more from the notebook, its cover and the band; the notebook flat
  on the desk (tilted less than 5 degrees); nothing moving more than 2 mm over the last second.
- 1.0 (success, held 1 s): shut, the band on the cover, released and still.
- 0.3: shut, released and still, but the band's middle not on the cover (caught round the fore-edge, say).
- 0 otherwise: the cover sprung open, or still held, or the notebook tipped up.

Nothing is latched; sliding the notebook on the desk costs nothing. A real trial is scored from a photo at the end,
after the grippers have moved away: the cover's fore-edge within 7 mm of the pages (a ruler), and the band lying across
the front cover, at least 3 mm in from its fore-edge.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=close_a_notebook_with_its_elastic_band.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Close the notebook and fasten it shut with its elastic band, then let go of it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/notebook-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The right gripper comes down over the
band at its middle (x 21.75), its fingers astride it on the page, pinches it and lifts it off the page; leaning its
hand 40 degrees out (wrist toward the robot's right, out of the cover's way), it draws the band out past the fore-edge
to y -10, low over the desk, and waits 2 s while the band's ends slide off the page down the head and the tail. Then
the left gripper, closed, comes onto the back of the open cover near the spine (x 30.5, 3.5 cm from the hinge) and
pushes it shut along its arc in 8 s, and holds it shut (pressing 1 cm into it). The right gripper lifts the band
straight up beside the fore-edge to 6 cm above the cover, so that its sewn ends stand up beside the head and tail,
carries it in over the cover to 3.5 cm past its line, lays it on the cover, slides it back to its line (y -5.5), lets
go and moves away; then the left gripper lets go of the cover. The band holds the cover shut.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_a_notebook_with_its_elastic_band.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Close the notebook and fasten it shut with its elastic band, then let go of it." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/notebook-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/notebook-dream/*.json; do inspect-robots video "$log"; done
```

`NBAND_DEBUG=1` prints, every 5 steps (`NBAND_DEBUG_EVERY=n` for every n), the cover's angle, where the band's middle
is in the cover's frame and whether it counts as on the cover, the notebook's position and tilt, the nearest finger's
distance and the body it is nearest, the most any body moved over the last second, several band links and both grippers;
`NBAND_DEBUG_BAND=1` adds every band link's position across and above the book.

`NBAND_MODE=one` runs the demo the wrong way, with one hand: the left gripper shuts the cover just as in the demo,
but nobody draws the band out first, so the cover comes down on the band lying across the first page (it stops 2.5
degrees open, the band under it). It holds the cover 3 s and lets go: the band under the cover holds nothing and the
cover springs back to 67 degrees. Score 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other simulation jobs: the scripted demo while rendering 1280 x 720
video took about 6 minutes (it succeeds after about 750 steps, 37 s of simulated time); the Dream-1 trial of 2,400
steps at 320 x 176 took 13 minutes (0.33 s per step), both running at the same time. Debug runs of the demo without
video or frames (1,000 steps) took 5 to 6 minutes, two at a time. Isaac Sim's first start on a new machine adds a few
minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## What is modelled and what is not

Rigid bodies (PhysX), 20 Hz control. The notebook is one rigid body (back board, page block and spine) with the front
cover on a sprung hinge; the pages do not bend and do not fan. The cover's spring (0.15 N m per radian toward 70
degrees) stands in for a new notebook's stiff spine; a real notebook's rest angle varies, so it is measured and stated
for a real trial. The elastic band is a chain of 16 rigid 2 g links (32 g in all, heavier than a real band's few
grams, for the solver): its two sewn-in roots (18 mm, sprung upright, leaning only toward the fore-edge or back, as a
stitched end does), then 14 links on ball joints that stretch along the band on springs (80 N/m for the whole band, an
estimate). The roots stand 2 mm above the shut cover's face, so the fastened band lies within 2 mm of it. Friction:
band 0.3 on paper and board, notebook 0.45 on the desk (estimates).

## Building it for real

A Moleskine Classic Large hard cover notebook (13 x 21 cm), or any A5 hardback notebook with an elastic closure band,
new, its cover springing back open when let go (open it fully a few times; measure its rest angle with a phone level
and state it). Lay it on the desk as in the screenshot, open the cover with the band on so that the band snaps onto the
first page, and start. Score from a photo after the arms have moved away.

[cw]: https://www.cowlingandwilcox.com/notebooks/2819-moleskine-moleskine-classic-large
