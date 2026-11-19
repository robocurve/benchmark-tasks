# Tilt a window's lower sash back up into its frame after cleaning and push it home until both tilt latches click

A small white vinyl single-hung tilt window stands on the table facing the arms, fixed in place as a window is fixed in
its wall: frame 47 cm wide and 65 cm tall, fixed upper sash. Its lower sash (38 x 30 x 3.5 cm, double glazed, 1.8 kg)
has been tilted in for cleaning: it hangs on its two pivot bars in the balance shoes at the bottom of the jamb tracks
and lies flat toward the arms, interior face down, its top rail resting on a foam-padded block. The robot must put it
back up in the frame so that it latches.

Two spring-loaded tilt latches are screwed to the top of the sash's top rail, one at each end. Each is a low housing
(8.0 x 3.3 cm, 4 mm high) with a finger tab whose top is 8 mm above the rail, and a bolt whose nose stands 11 mm
(7/16 in) out of the sash's side into the slot of the jamb liner. That nose is what holds the sash upright. It is
ramped on the side that meets the jamb: pushed home, the sash presses both noses in against their springs, and they
click out into the slots once it is upright. Nothing on the sash is a handle.

## What window makers tell owners to do

Care guides for tilt windows give the same step for putting a sash back. The sash is pushed home and the latches click
in by themselves; they are slid by hand only to tilt the sash in.

- Gentek, Window Care and Maintenance
  (https://gentek.ca/wp-content/uploads/sites/2/2021/01/Window-CareMaintenance-English.pdf): "To replace the sashes
  to the operating positions, hold the top rail of the sash with both hands and push it all the way into the main frame
  tracks. Make certain that the tilt latch buttons click in to lock the sash in the main frame. If the tilt latches are
  not reengaged in the main frame, the sash may fall inward unexpectedly."
- Vinylmax, Care and Maintenance Guide
  (https://www.vinylmax.com/wp-content/uploads/2019/06/Vinylmax-Care-and-Maintenance-Guide.pdf): "Tilt-Latch -these
  latches are spring loaded and when engaged, allow your window sash to be released from the frame." "To return the
  window sashes to their normal location, one at a time hold the top rail of each sash and firmly push it all the way
  back into the frame. Make sure that both tilt-latches click back into the frame".
- Champion Windows, double-hung sash removal and replacement
  (https://www.championwindow.com/service/double-hung-sash-removal-replacement): "Raise the top of the sash back
  towards the window frame, and push it into place until you hear the tilt-latches click."

## What the task tests

- **Raising a hinged 1.8 kg sash with two hands and nothing to hold on to.** A gripper cannot stay closed on the top
  rail to the end, because the upper sash stands 3 mm behind it once the sash is upright. The rail has to be carried
  on open hands and turns through 90 degrees in them as the sash comes up.
- **Keeping the sash on its pivots.** The pivot bars only rest in the shoes. Lifting the sash rather than turning it
  takes it off them (score 0).
- **Pushing home against two sprung noses.** The noses meet the jamb liners about 10 degrees short of upright. From
  there the sash has to be pushed, not just raised: about 11 N at the top rail by calculation from the spring rate,
  the ramp angle and the friction used here (an estimate, see Fidelity).
- **Not stopping short, and letting go.** A sash left resting on its noses against the liners is not latched and
  falls back in when it is let go. The job is done when the latched sash stands in the frame with both hands away.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `sashkit.py` | Rigid parts built from boxes, cylinders and wedges, with spring-loaded sliders, fixed in place, or starting tilted, for yam_arena task files |
| `demo.py` | The scripted full-score demo and the control runs |
| `test_scoring.py` | Offline checks: every scoring outcome, the window's geometry, the latch's size, the demo's plan (no GPU) |

`sashkit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with the
standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sash-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.
`SASH_LOT=B` in the environment selects layout B (the window and block 1 cm nearer the robot and 2 cm to its left).

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1200              # writes demo_*_cam.mp4, setup.png, result.json
SASH_LOT=B python demo.py task.py demo_b                       # the same on layout B
python demo.py task.py gentle_out --mode gentle                # a control run: still | gentle | onehand | oneside
python -m pytest test_scoring.py                               # offline checks
```

1. Both hands swing, open, to a point above and behind the ends of the top rail, then slide the lower finger of each
   open gripper under the rail's interior-top edge, 15.5 cm either side of the sash's centre line. The hands lean
   60 degrees from straight down toward the window, fingers one above the other; the upper finger stays clear above
   the rail.
2. The two lower fingers raise the sash together about its pivot bars, 3 degrees every quarter second, from lying flat
   to 12 degrees short of upright. Nothing is gripped. While the sash leans more than the hands, its edge lies on the
   fingers' pads; nearer upright, the fingertips bear on its interior face 15 mm below the edge.
3. The fingertips push the sash home over 1.5 s. They are sent 8 degrees past upright, because the arms are compliant
   and lag (by up to 31 mm here). The noses meet the jamb liners, are pressed in 8 mm, and click out into the slots
   as the sash closes against the upper sash.
4. The hands ease off, draw back 9 cm, then move down and back toward the robot. They are more than 3 cm from every
   part of the sash from step 335 and 15 cm from it when the trial ends. The latched sash stands by itself.

The demo runs open loop: it reads no simulator state, only the published layout in `task.py`. The sash turns about its
fixed pivot bars, so where the rail is at every step follows from the layout.

| Demo | Score | Sash off the block | 0.25 | 0.5 | Noses pressed in | 0.75 | Upright, latches out | Grippers 3 cm clear | 1.0 | Trial ends |
|---|---|---|---|---|---|---|---|---|---|---|
| Layout A | 1.0 | step 135 | 185 | 250 | 270 to 280 | 280 | 285 | 335 | 355 | 371 |
| Layout B | 1.0 | step 135 | 185 | 250 | 270 to 280 | 280 | 285 | 335 | 355 | 371 |

(Steps are read from a trace kept every 5 steps. 355 steps is 18 s; the run limit is 1,200. From step 285 to the end
the sash stays at -0.4 degrees with both latches at rest; the score is 0.75 until the grippers have been clear for
1 s.)

## Control runs

All on layout A, with the same script apart from the change named.

| Run | Score | What happened |
|---|---|---|
| `--mode still`: nothing is done | 0.0 | The sash stays on the block. |
| `--mode gentle`: raised to the liners, held 2 s, laid back down, no push | 0.5 | The sash stops 10.6 degrees short of upright on its noses; neither latch moves (0.0 mm). |
| `--mode onehand`: the whole job with the left hand alone | 0.0 | The hand sags up to 32 mm below its commanded path and the sash stays on the block (tilt never below 89.9 degrees). |
| `--mode oneside`: raised by both hands, pushed home by the left hand alone | 1.0 (step 355) | Both noses are pressed in and both click out: the score does not depend on which hand pushes. |
| Dream-1, 1,200 steps | 0.0 | |
| Dream-1, 200-step test | 0.0 | |

With the latch springs set to 100, 300 and 700 N/m in place of 400, the same lift and push also brought the sash
upright with both latches out.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Tilt is how far the
lower sash leans in from upright (0 upright, 90 lying flat toward the arms). On a real window it is read with a
protractor or level on the sash, and the latches are checked by eye or with a ruler.

- 0 if the sash ever came off its pivots (its pivot axis more than 1 cm from where the shoes hold it).
- Success: the sash upright (tilt between -1 and 1.5 degrees; it closes against the upper sash) with both latch noses
  out in their slots (each latch within 1 mm of its rest position), still (its tilt changing less than 0.3 degrees
  over the last 1 s), and let go: both grippers at least 3 cm from every part of the sash (frame, glass, latch
  housings, tabs and bolts), measured each step from points along both fingers and the palm of each gripper. It
  scores 1.0 once all of this has held for 1 s (20 control steps in a row), when the episode ends; until then, 0.75.
  A sash that is latched but still held, or with a gripper resting on it, scores 0.75.
- Otherwise the furthest it got: 0.75 if it was pushed to within 3 degrees of upright (both noses are then pressed in
  past the jamb liners, which they first meet about 10 degrees from upright); 0.5 within 20 degrees; 0.25 within
  60 degrees (top rail lifted about 14 cm); else 0.

### Scoring a real trial

From the last second of the video and the end state. Hands away: both grippers at least 3 cm from every part of the
sash through that second (a ruler held to the nearest finger). The sash upright: a protractor or level on it reads
between -1 and 1.5 degrees. Both latches out: each tab back at its rest position within 1 mm, its nose in the jamb
slot. All three give 1.0. Otherwise the score is the furthest the sash got during the trial, read from the video
against 60, 20 and 3 degrees from upright, and 0 if it came off its pivots.

## Runtime and seeds

A trial is 1,200 steps (1 minute of simulated time). On an L40S a full Dream-1 trial took about 3 minutes including
about 15 s for Isaac Sim to start; the scripted demo ends at step 371. The layout is fixed (`-E object_jitter=0`);
there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes, cylinders and wedges, at full size: the lower sash is 38 x 30 x 3.5 cm (vinyl profile
and 1.8 cm insulated glass), the size of the lower sash of a small (about 18 x 26 in) single-hung window. Each tilt
latch is one moving part (finger tab, bolt and nose) on a prismatic joint along the top rail with a spring drive and
12 mm of travel, and a fixed housing on the rail.

What is taken from a real part and what is estimated:

| Quantity | Value | Where it comes from |
|---|---|---|
| Latch housing length and width | 80.2 x 33.3 mm | Part listing: "Length: 3-5/32 in." "Width: 1-5/16 in." |
| Nose standing out of the sash's side | 11 mm | Part listing: "Latch Extension: 7/16 in." |
| Housing height above the rail | 4 mm | Estimate (the listing gives no height) |
| Finger tab | 22 x 12 mm, top 8 mm above the rail | Estimate |
| Nose thickness and ramp | 12 mm, 45 degrees | Estimate |
| Latch spring | 400 N/m: 3.2 N with the nose pressed in level with the liner | Estimate |
| Friction of the latch on the liner | 0.2 | Estimate |
| Friction of the vinyl | 0.4 | Estimate |
| Moving part of a latch | 20 g | Estimate |
| Sash weight | 1.8 kg | Estimate |
| Jamb slot | 14 mm wide, 12 mm deep, liner 3 mm clear of the sash | Estimate |

The push needed at the top rail, about 11 N, is calculated from those estimates (two noses at
3.2 N x (tan 45 + 0.2) / (1 - 0.2 x tan 45) = 4.8 N each, plus about 1.3 N for the sash's weight at 9 degrees), not
measured.

The pivot bars rest in fixed shoes, as real balance shoes lock in the track when the sash tilts; the spring balances
that carry a raised sash are left out, since the sash is only tilted. The upper sash is fixed, as in a single-hung
window. The sash's bottom rail stands 8 mm clear of the sill (a real one is bevelled and seals on a weatherstrip).
The finger tabs are blue so that they can be seen; real ones are the colour of the latch. Light angular damping is
added to the parts. No contact force cap is enforced in simulation.

## Building it for real

**The latches.** One pair of screw-on spring-loaded tilt latches. A stock part with the dimensions used here: All About
Doors (allaboutdoors.com), "Spring Loaded Vinyl Tilt-in Finger Latch, Pairs", SKU 503060-KIT
(https://www.allaboutdoors.com/Spring-Loaded-Vinyl-Tilt-in-Finger-Latch-Pairs-Choose-Color_2). Its listing says:
"Spring Loaded vinyl tilt in finger latch." "Used on Aluminum and Vinyl Single or Double hung windows." "Screws to top
of sash." "Screw Hole Spacing: 2-1/8 in." "Length: 3-5/32 in." "Width: 1-5/16 in." "Latch Extension: 7/16 in."

**The window, built from the dimensions in `task.py`** (this reproduces the simulated one):

- Frame, 47 cm wide, 65 cm tall, 10 cm deep, in 18 mm plywood or PVC trim board: a sill 4.2 cm thick, a head 4 cm
  thick, two jambs 4.2 cm thick. Each jamb has a slot 14 mm wide and 12 mm deep along its inner face, centred 4 cm
  from the frame's interior face. A block fills the bottom of each slot up to 6.9 cm above the table (the shoe).
- Upper sash, fixed in the frame: 38 x 30 x 3.5 cm, its interior face 3 mm behind the lower sash's exterior face, its
  lower rail overlapping the lower sash's top rail by 4 cm.
- Lower sash: 38 x 30 x 3.5 cm, 1.8 kg (PVC board stiles 4.5 cm wide, bottom rail 5.5 cm, top rail 4 cm, round an
  18 mm glazing panel, weighted to 1.8 kg). A 12 mm round steel bar stands 11 mm out of each side, 2.5 cm above the
  bottom edge; the bars rest on the shoes, 7.5 cm above the table.
- The latches screwed to the top of the top rail, one at each end, noses standing 11 mm out of the sash's sides. Cut
  the slots to suit the noses of the part in hand.
- A foam-padded wooden block, 7 x 24 x 5.7 cm, on the table under the tilted sash's top rail.
- The frame screwed to a board clamped to the table, its interior face 38 cm in front of the midpoint between the arm
  bases, centred on it.

**A stock window instead.** Vinyl single-hung and double-hung tilt windows have this mechanism. One small enough for a
table, in stock when checked on 19 November 2026: Kinro 14 x 27 in White Vinyl Vertical Sliding Window, item
KVS1427V (https://mobilehomepartsstore.com/parts/KVS1427V.html), rough opening 14.25 x 27.25 in, with "Tilting Bottom
Sash" listed as "Yes". It is narrower than the window modelled here (36 cm against 47 cm), so the sizes in `task.py`
would be set to its sash. Its latches were not examined.

Trials are safe to run: the sash weighs 1.8 kg and can only fall back onto the padded block.

## Assets

All geometry is defined in `task.py` from boxes, cylinders and wedges; no external assets.
