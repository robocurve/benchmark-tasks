# Ticket a shirt through its care label only: lift the label clear, push a tagging gun's needle through it alone, fire

A folded cotton T-shirt lies flat on the table with its hem turned back at one side seam, so its sewn-in care label lies on top. A tagging gun lies on its side beside it with a price ticket on its needle and a clip of fasteners loaded. The robot lifts the label clear of the shirt with one hand, pushes the gun's needle through the label alone with the other, squeezes the trigger to fire one fastener, draws the needle back, and lets the label down with the ticket hanging from it.

**Where it comes from.** Stores ticket clothes with a tagging gun (an Avery Dennison Mark III standard gun is a common kind): the trigger drives a plunger that pushes a plastic T-end fastener down the gun's hollow needle. Clothes are ticketed through the care label or a seam so that the fabric itself is not holed. Retail salespersons have the core task "Ticket, arrange, and display merchandise to promote sales", and stockers and order fillers "Mark stock items, using identification tags, stamps, electric marking tools, or other labeling equipment" (O*NET 41-2031.00 and 53-7065.00, USDOL/ETA, CC BY 4.0; O*NET is a trademark of USDOL/ETA). Avery Dennison and Mark III are named for identification only; this task is not affiliated with them.

**What this task tests.** The label is sewn along one edge and lies on the shirt. It must stand 2 cm clear of the body fabric where the needle crosses it; in the demo one hand takes it by its free edge and holds it up. The other aims a 2 mm needle through the label's few millimetres of cloth while missing the shirt just below it, squeezes only once the needle is through, and draws the needle back. A needle into the shirt's body, at any moment, is a hole in the garment and scores 0.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Put the price ticket on this shirt through its care label only: lift the label clear of the shirt with one hand, push the tagging gun's needle through the label alone with the other, fire one fastener, draw the needle back, then let the label down and let go of it."
```

A trial runs up to 2,400 steps (120 s of simulated time at 20 Hz). One fixed layout; no seed changes the scene.

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                   # the full-score reference, filmed
python demokit.py task.py OUT --no-record --no-cameras --mode nolift        # failure check: the label left lying on the shirt
python check_score.py                                                       # the scoring table on hand-built records, no simulator
```

The demo is a scripted reference: it reads the label's and the gun's poses from the simulator, and a policy gets only the cameras. The left hand comes in from beside the label's free edge with its fingers tilted 60 degrees from straight down and its jaws closing across the label, pinches the label 12 mm in from its curled free edge, and swings it up about its seam to about 40 degrees. The right hand takes the gun from the table by its grip, its fingers across the grip's back and the trigger at 20 N (under the trigger's 25 N preload, so the gun does not fire in the carry), and brings the needle in level toward the label. It pierces the label 38 mm out from the seam, where the label stands 2.4 cm clear of the shirt, and stops the needle's tip about 3 mm past the label's face. It squeezes, up to 50 N, until the trigger has travelled 2.5 mm and fired one fastener into the label, and eases off at once (held on its stop, the trigger would let the gun slip in the hand). It draws the needle back, and holds the gun up away from the shirt. The left hand lets the label down, opens its jaws 12 mm, and draws them out off the label's edge. The ticket hangs from the label on the fastener's barb.

**Result** (scripted, GPU physics on an NVIDIA GeForce RTX 3090 Ti, 20 Hz, filmed at 640 x 360): 1.0 in 456 steps (22.8 s of simulated time); the film's first frame is the start state.

| Failure check | What it does | Score |
|---|---|---|
| `nolift` | the label is left lying on the shirt and the needle is pushed at it as in the demo | 0.0 in 360 steps (GPU physics): the label stayed at 8 degrees, never 2 cm clear, so the needle never counted as through it; the shirt was not pierced |
| `check_score.py` | the firing gate on hand-built needle poses (through the label's face 2.4 cm clear, from either side: through; 1.3 cm clear, slipped under the label from its free edge, beside it, or under it without crossing: not through), the needle's entry on hand-built paths step by step (pushed through the face: through; slid in sideways from beside the label or from past its free edge: not through; pushed through then dragged out sideways through the label's edge: torn, 0), the let-go check on hand-built fingertips (a right hand pinning the label down: not let go), and the scoring table on hand-built records (a pierce, a torn label or a dropped gun scores 0 whatever else is done) | every case as expected. A run that pierces the label, draws back and fires with the tip slipped under the label from its edge stays at 0.5: the label was once through, but the fastener fires while the needle is not through it now, and only the first squeeze counts |

## Scoring

| Score | When |
|---|---|
| 1.0 | one fastener fired while the needle was through the label alone (pushed in through the label's face inside its outline, its tip and its base either side of the face, the needle crossing it inside its outline where the label stood at least 2 cm clear of the shirt), the shirt never pierced, the needle drawn back (its tip 4 cm or more from the label's middle), the label let down under 30 degrees and let go (every fingertip of both hands 5 cm from the label's middle), all held 1 s. Only the first squeeze counts |
| 0.5 | the needle through the label alone, but no fastener fired while it was through, not drawn back, or the label not let go |
| 0.25 | the label lifted 2 cm at its free edge, nothing more |
| 0 | at any moment: the needle's tip inside the shirt's body or within 4 mm above its top face (a hole), the label pulled past 110 degrees from the shirt (torn at its seam) or the needle dragged sideways out through the label's edge, or the gun dropped off the table |

On a real trial: a top camera, both wrist cameras and a side camera level with the shirt film the trial; the label's lift where the needle crosses it is read on the side camera against a 2 cm mark; afterwards the label is checked by hand for the fastener through it, and the shirt's body for any hole.

## Building it for a real trial

- **Shirt:** a cotton T-shirt, folded to 24 x 30 cm, its hem turned back at the side seam so the care label lies on top, the shirt taped to the table along its far edge so it does not slide.
- **Label:** the shirt's own woven or satin care label, about 30 x 60 mm, sewn along its short edge. A label that lies flat can be curled up at its free edge by folding it once over a pencil; the simulator's label rests 8 degrees up (its free edge about 8 mm up).
- **Gun:** an Avery Dennison Mark III standard tagging gun (or the same class), a standard needle, a clip of 50 mm standard fasteners, a 30 x 50 mm card price ticket on the needle. A spare needle for the trials: needles bend.
- **Safety:** the needle is sharp; keep hands out of the station while the robot holds the gun, and fit a needle guard between trials.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Gun mass and size | 100 g, 125 x 25 x 130 mm | **Not measured.** A UK retailer's listing gives these; weigh and measure the gun |
| Trigger preload and travel to fire | 25 N preload, fires at 2.5 mm of a 4 mm travel | **Not measured.** A force gauge on the trigger's middle: the force where it starts to move, and the travel when the fastener is pushed out |
| Label's curl | rests 8 degrees up, springs back at 0.01 N m per radian | Lay the shirt flat and measure the free edge's height; lift it 3 cm with a thread and a small force gauge |
| Label's tear | pulled past 110 degrees from the shirt | Not a force: the scorer counts any pull past folding the label right back over its seam |
| Needle through the label | its tip crosses the label's face where the label stands 2 cm or more clear of the shirt | On the real label, a fastener through the label alone and no hole in the shirt, both checked by hand |

## What the simulator leaves out

Cloth is not simulated: the folded shirt is a stiff panel and the label a thin flap hinged at its sewn edge. The needle has no collider; its passage through the label and a pierce of the shirt are read from where its tip is. The scorer remembers the needle's entry: it counts as through only after its tip has crossed the label's face inside the outline, so a needle slid in sideways from beside the label or past its free edge never counts. The fastener is shown once the trigger has fired with the needle through the label; how the fastener's T-end opens behind the label is not modelled. The ticket hangs from the label as a shape, without its own physics.

## Assets

Every shape in the scene (shirt, label, gun, ticket, fastener) is built in code from boxes and cylinders in `task.py`; no third-party meshes, textures or fonts are used. The YAM arms and the station come from `yam-isaac-lab-arena`.

## Licence

MIT
