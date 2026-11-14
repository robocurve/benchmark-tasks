# Wipe a monitor lead, cord and both end pieces, outward from its blue band, never wiping back

A free grey patient-monitor lead (6 mm PVC cord, 60 cm) lies straight across the table: its grey plug in front of
the left arm, its red patient clip in front of the right arm, and a blue band on the cord 14-16 cm from the plug.
Both grippers' finger pads are covered with disinfectant wipe. The robot must wipe the whole lead: the cord from the
plug to the clip and the plug and clip bodies, outward from the blue band to each end, every part once and never back
over a part already wiped. Then it lays the whole lead on a clean white drape and lets go.

Why: the Philips lead-set guide says "Clean and disinfect the ECG cable and accessories after each use" and, for the
MX40 lead sets, "Wipe the entire outer surfaces of the cable to remove soil". Only the connector pins get a separate step
(alcohol, connector pointed down), so the pins are not part of this task, but the plug and clip housings are. A
damp-wiping guide adds the one-direction rule: "Avoid using a back and forth motion to prevent the cloth picking up
dirt going one way and then dropping it off on the way back" and "wiping each part of the surface ONCE is enough".

Start point, stated plainly: none of these sources says where on a lead to start or which way to wipe it, so the blue
band is a task choice, not a rule from a manual. What the task takes from real practice is the one-direction rule: wipe
each part once and never back over cleaned surface. Starting in the middle and wiping outward to both ends is one way
to satisfy that rule with a free lead and two hands (see below); wiping from one end to the other in a single pass
would satisfy it too, but a closed hand moving toward the hand that holds the lead cannot go all the way, so the
scoring splits the lead at the band.

The tested ability is continuous sliding contact along a free deformable object in one direction, with the anchoring
role changing between the two hands. The lead is free, so a closed hand moving along it just drags it unless the
other hand holds the far end: to wipe the clip half and the clip outward, one hand holds the plug while the other
slides toward and over the clip; to wipe the plug half and the plug, the hand on the clip holds it while the other
slides from the band toward and over the plug. The wiping hand has to stay closed on the thin cord while following
it, open just to the body's width to pass over each end piece without dragging it, must not open and hop forward
(gaps stay unwiped) and must not scrub back and forth.

Why the blue band: a hand closed on the cord needs about 15 cm of clearance from the other hand, so a stroke can
start no nearer than that to the held end. The band marks that point; from it each half is wiped outward, away from
the hand that anchors it, so the two strokes meet at the band and nothing is wiped twice beyond a short overlap (the
damp-wiping guide asks passes to overlap by 2-5 cm).

## Closest tasks

- TK-7P97ME (finish painting a panel without repainting a coworker's work) scores grid-cell coverage of a rigid flat
  panel by a roller with a no-repaint rule; here the "surface" is a free cord that must be anchored by the other hand
  while a closed pinch slides along it, and the anchor role switches between hands.
- TK-GM789T (check engine oil with a dipstick) scores a rag touching a rigid blade pulled from its tube; here the pads
  close round a free deformable cord and two end bodies and the score depends on stroke direction and anchoring.
- TK-LMMBEC (swipe a card through a reader in one pass) scores a single one-way pass of a rigid card through a fixed
  slot; here the object being passed over is itself free and deformable and has to be held taut by the other hand.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Setup and materials (real trial)

Positions in cm from the midpoint between the arm bases on the table (x forward, y to the robot's left).

- Lead: grey 6 mm round PVC cord, 60 cm, lying straight along y at x 40 from y 6.25 to y -53.75; grey plug
  35 x 16 x 12 mm (12 g) on its left end (centre y 8), red patient clip 25 x 15 x 15 mm (10 g) on its right end
  (centre y -55). Blue tape band 2 cm wide on the cord 14-16 cm from the plug end of the cord (y -7.75 to -9.75).
- Drape: white cloth on a 3 mm board, 18 x 76 cm, x 12-30, y -56 to 20, taped down.
- Wipes: PDI Sani-Cloth Plus pre-moistened disinfectant wipes, large-canister sheet 6 x 6.75 in (about 15 x 17 cm). Before each trial, cut one fresh wipe in half for each gripper; fold each half into an 8-layer pad
  about 4 x 4 cm and 3 mm thick, lay one pad over each finger pad so it covers the pad's gripping face and wraps round its tip, and
  hold it with a rubber band round the outer face of the finger. The wipes are fitted before the trial starts, so
  whatever a gripper closes on, it grips through wipe; the robot never picks a wipe up. Keep the wipe box off the table.
- Pad gap g0: with the pads fitted, close each gripper fully on nothing and note its finger gap g0 (the two pads'
  pressed thickness, about 6 mm). The simulation has bare fingers; real-trial gap thresholds are given next to the
  simulation's below.
- Tracer: before the trial, dust the cord and the two long side faces of the plug and of the clip (the faces the
  closing pads meet: plug 35 x 12 mm, clip 25 x 15 mm, each side) with UV fluorescent tracer powder; mark the cord
  every 1 cm with a fine pen (58 marks, 1.5 cm to 58.5 cm from the plug end); draw a line round the plug and
  the clip 8 mm from the cord face.
- Start: both arms in their ready pose (grippers about 26 cm above the table at x 32, in front of their bases).
  The right gripper hovers over the middle of the lead in the top-view image and hides part of it.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "Wipe the whole grey monitor lead with the wipe pads, its cord and both end pieces, wiping outward from the blue band and never back over any part already wiped, then lay the lead on the white drape and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

The demo chooses its motions only from the fixed, known starting layout and the robot's own measured joint angles
(TCP positions computed from them); object poses from the simulator are used only for scoring and the run log. While
one hand holds an end piece and the other slides along the taut cord, the slide position along the cord is the
measured distance between the two hands minus a constant 1.8 cm; when a hand lets go, the other end piece is
re-located from the anchoring hand, 63 cm away along the straight lead (plug centre to clip centre). Sequence: the
left hand takes the plug and lifts it 2.5 cm; the right hand closes lightly on the cord at the blue band (pads 5 mm
apart on the 6 mm cord) and steps toward the clip in 1.5 cm moves, slowing to 5 mm moves near the end, until its
pads are about 1.5 cm short of the clip face (closed pads must never run into an end piece). It opens and lifts;
the left hand pulls the recoiling lead straight (plug 4 cm further left). The right hand sets down wide open with its pads
straddling the clip's cord face, closes to 21 mm (bare fingers, about 3 mm clear of each side, the thickness of the
real pad; closing centres the clip between them) and slides in one pass over the clip and 2.2 cm off past its tip, the left hand still holding the plug. It opens, lifts, and takes the
clip by its centre. Both hands carry the lead 8 cm to the left; the left hand sets the plug down and lets go, closes
lightly on the cord at the blue band (the right hand lifts the clip 2.5 cm so the cord near the band is clear of the
table) and steps toward the plug until its pads are about 1.5 cm short of the plug face. It opens, the right hand pulls the lead straight
(clip 4 cm further right), and the left hand sets down astride the cord 1.2 cm short of the plug face, closes to
22 mm and slides in one pass over the plug and off past its tip, the right
hand still holding the clip (in the recorded run the plug lay 1 cm off the line and is nudged about 1 cm along
before it centres between the pads). It opens, re-takes the plug by its centre, both lay the lead on the drape, open and move
clear. Recorded run (`demo/`): 1,033 steps (51.7 s), score 1.0, `success()` true.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

Terms:
- **Closed on the cord**: finger gap at most 7 mm (cord diameter + 1 mm; real trial: 7 mm + g0) with part of the
  cord between the pads.
- **Holds** an end piece: finger gap at most its larger cross size + 3 mm and its centre within 2.5 cm of the TCP.
- **Stroke**: one continuous closure of a hand on the cord. Its direction is set once it has slid 1 cm along the
  cord. Its **anchor** is the end piece behind it (the plug for a stroke toward the clip, the clip for a stroke toward
  the plug), and the stretch it passes over (the pads' 2 cm footprint, swept) counts only while the other hand holds
  that anchor.
- **Wiped mark**: the cord has 58 marks, one per cm (1.5 to 58.5 cm from the plug end). The centimetre of cord
  next to each end piece's face carries no mark and is not scored: closed pads running along the cord cannot reach
  it without driving into the end piece (in the simulation that blows the contact up); the body pass, whose pads
  straddle the cord face when they close, passes over it with the pads opened to the body's width. A stroke wipes the marks
  within 0.5 cm of the stretch it passed over once it has slid at least 2 cm. A stroke toward the clip counts only
  marks from 13.5 cm on, a stroke toward the plug only marks up to 16.5 cm: each side of the blue band must be wiped
  outward. Opening and re-closing further on starts a new stroke, so hops leave gaps.
- **On a body**: the finger gap fits the end piece (from its smaller cross size - 2 mm to its larger + 7 mm: plug
  10-23 mm, clip 13-22 mm; the 7 mm is the two 3 mm wipe pads the bare simulated fingers lack, plus 1 mm; real trial:
  at most the larger cross size + g0 + 1 mm, i.e. the pads touching the body), the body's centre line lies between the pads (within 5 mm of their mid-plane) and within
  the pads' height, and the pads overlap the body's length. u is the pad centre's position along the body from its
  middle, positive toward the free tip; the body runs from u = -L/2 (cord face) to +L/2 (tip), L = 35 mm plug, 25 mm
  clip.
- **Wiped body**: one continuous closure on the body that starts with the pad centre at most 8 mm onto it
  (u <= -L/2 + 8 mm: the 2 cm pads straddle the cord face) and slides outward until the pad centre reaches the tip
  (u >= +L/2; the pads' footprint has then passed over the whole length), with the other hand holding the other end
  piece at every step of it.

Score:
- **0** if at any time (latched for the whole run): a stroke goes more than 3 cm back against its direction along
  the cord, or its hand comes more than 3 cm closer to the anchor end piece than it has been during the stroke (a
  closed stroke back, which pushes the cord instead of sliding on it); or a stroke passes over more than 4 marks that were already wiped before it began
  (wiping back over clean cord; the strokes may overlap by up to 4 cm, as passes should overlap); or a hand on a
  body that closed straddling the cord face goes more than 1 cm back toward the cord (u falls 1 cm below its highest
  value in that closure); or a new
  closure on a body already wiped slides 1 cm or more along it (wiping it again).
- **0** if the plug is not on the drape at the end, or a gripper is within 3 cm of the cord, plug or clip at the end
  (not released), or the gripper state cannot be read.
- Otherwise, with c = fraction of the 58 marks wiped:
  - **0.3**: c from 0.5 to under 0.9 (for example only the stretch from the blue band to 55 cm).
  - **0.6**: c of 0.9 or more, but not all of: every mark wiped, both bodies wiped, the whole lead on the drape (for
    example the plug-side stroke stopped 6.5 cm short of the plug, or the whole cord wiped but the clip or the plug
    body skipped).
  - **1.0**: all 58 marks wiped, the plug body and the clip body both wiped, and the whole lead (cord, plug and clip)
    lies on the drape.
- `success()` additionally requires the lead to be at rest. `distance()`: 1 - (marks wiped + bodies wiped) / 60
  (1.0 once a 0-rule fired).
- How it is seen: wiping is a path, not a pose in the final photo. In the video, a stroke is a closed gripper sliding
  along the cord while the other gripper holds the end behind it; the strokes should start at the blue band and end
  about 1.5 cm short of the clip face and the plug face. A body pass is the same hand opening, setting down astride the
  body's cord face, closing to just over the body's width and sliding outward over the body and off past its tip,
  the other hand still holding the far end.
- Real trial: a mark counts as wiped when the tracer is gone from it under a UV torch and the video shows the stroke
  and its anchor as above. A body counts as wiped when the tracer is gone from both long side faces along their whole
  length and the video shows one pass from the cord face off past the tip with the far end held. Gap thresholds are
  the real-trial ones above (pads fitted). Backward strokes and passes and the final layout are read from the video and
  a photo of the drape: a back-slide is read against the 1 cm pen marks under the pad edge; for the bodies, a pen
  line round each body 8 mm from its cord face and one at its tip: the middle of the 2 cm pad must be at or behind the
  first line when the pads close and reach the tip line before they open.

## Shortcut variants (test only, `WIPE_CHEAT=<name> python cabledemo.py task.py out`)

| Variant | What it does | Score | Why |
|---|---|---|---|
| `none` | does nothing | 0.0 | nothing wiped, plug not on the drape |
| `onearm` | right hand slides from the band toward the clip with the plug not held, then lays the lead | 0.0 | no anchor held, so no mark counts; plug not on the drape |
| `hover` | full demo motions with the fingers open (pads never close on the cord) | 0.0 | no closure on the cord, 0 marks wiped |
| `hops` | clip-side half done by opening, moving 10 cm and closing again | 0.0 | each closure slides under 2 cm, so the clip half earns nothing; c = 0.26 from the plug-side stroke |
| `hops25` | the same with 2.5 cm hops | 0.0 | c = 0.05 |
| `scrub` | closed hand pushes 8 cm back toward the plug mid-stroke, then on again | 0.0 | 0-rule: hand came more than 3 cm closer to the anchor |
| `back` | opens, goes back 8 cm, closes and wipes on | 0.0 | 0-rule: re-wiped 5 cm of already-wiped cord |
| `hold` | whole correct wipe and lay, but the hands never let go | 0.0 | not released (every mark was wiped) |
| `midonly` | only the middle stretch: blue band to 55 cm, then lay and let go | 0.3 | c = 0.76 |
| `short2` | plug-side stroke stops 6.5 cm short of the plug | 0.6 | c = 0.93, four marks by the plug unwiped |
| `noclip` | whole correct run but the clip body is never passed over (the hand opens at the end of the cord stroke and takes the clip) | 0.6 | whole cord wiped, clip body not wiped |
| `noplug` | whole correct run but the plug body is never passed over | 0.6 | whole cord wiped, plug body not wiped |
| `clipin` | clip body passed over the wrong way, from past its tip back toward the cord | 0.0 | the inward pass is not credited (a pass must close straddling the cord face and slide outward); in the recorded run it also pushed the clip so the anchoring grip missed it, the plug-side stroke earned nothing and the plug ended off the drape |

## Runtime and seeds

- Run limit 1,600 steps (80 s at 20 Hz). One trial takes ~5-15 min wall time on one GPU including the ~1-2 min Isaac
  Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts from the same state.
