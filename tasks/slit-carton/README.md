# Slit a taped carton's 40 cm seam with a box cutter, short of the product 6 mm below, then draw the blade back in

The knife starts with its blade concealed in the handle. The robot pushes the blade out on its slider, slits a taped carton's seam from one top edge to the other so both flaps come free, without the blade reaching the product packed 6 mm under the tape or cutting the carton beside the seam, then draws the blade back in.

**Ability tested:** a depth-limited slit over the contents, with a knife whose blade has to be pushed out first and drawn back in after. Knife tasks such as cutting a held cord or sawing a loaf cut through their work with a knife that is ready to use. Here the cut's depth is set by what lies under the tape: pushed out, the blade stands 20 mm out, so with the knife's nose on the board the tip goes 20 mm x sin(angle) deep; it cuts the tape above about 2 degrees and reaches the product above about 17 (asin 6/20). With the nose resting on the board the angle sets the depth; held off the board, as the demo holds it (the tip 1.5 mm under the tape at 14 degrees, the nose about 3 mm above the tape), the hand's height sets it. That depth has to hold along the whole 40 cm seam, keeping to the seam, in at one edge and out at the other, and the blade must stay out of the product on the way in, on the way out, and while it is carried.

**What is scored, and what is not.** The box cut open, nothing cut that should not be, and the blade drawn back in at the end. Whether the blade was out while it cut (it is a means), which hand did what, and where the knife ends up are not checked. The demo shows the second hand pushing the blade out, as a person does with a thumb, and drawing it back in.

**The tape stops at the two top edges.** That is a real, lighter way of taping a carton.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Use the knife to cut the box open along its taped seam without cutting what's inside, then draw the blade back in."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                                 # full score, with a video
python demokit.py task.py OUT --no-record --mode deep,half,noretract,astray,lockpush        # the failure modes below
python check_scorer.py                                                                     # the scoring table, the hold, the hand, slider, blade and board tests and the tape rule, with no simulator
```

The demo is a scripted reference. It steers on the exact poses of the knife (and its blade joint) and of the carton, read from the simulator, which a policy does not get: a policy gets only the cameras. The right arm picks the knife up off the table by the back of its handle, brings it round the carton's near corner and holds it 12 cm up, rolled blade down so the slider is on top, pointing along its own forearm. The left arm's closed fingers come down onto the spine behind the slider and push it 20 mm forward, to its stop. The right arm points its hand along the seam, tipped 14 degrees down, lowers the tip just outside the near edge, 1.5 mm under the tape's top, draws it along the seam and off the far edge, lifting it out of the tape over the last 2 cm. It brings the knife back to the left hand, which draws the blade back in, and puts the knife down where it was.

**The knife is not square to the seam.** The right hand picks the knife up lying flat with its fingers tipped 14 degrees down, so the knife sits about 13 degrees off the hand's axis, and once it is rolled blade down that offset turns it sideways: in the demo the knife points about 13 degrees off the seam while the tip runs straight along it. Turning the hand to take that out puts the far end of the seam out of the right arm's reach. The scorer checks what was cut, not which way the knife points.

## Building it for a real trial

- **Carton:** a single-wall C-flute corrugated regular slotted carton, 40 x 30 x 25 cm outside (16 x 12 x 10 in), board about 4 mm thick. Cut off the two short inner top flaps, so that the product can sit right under the two long flaps. The long flaps meet along the middle.
- **Tape:** one strip of 48 mm polypropylene packing tape along the seam, ending at the two top edges.
- **Product:** a closed folding-carton (paperboard) box that fills the carton, white or any plain colour, so that a knife mark shows on its top, its top 2 mm under the flaps' underside: 6 mm under the tape's top. Pack to that height with card sheets under it. Carton and product together 3 kg.
- **Knife:** a retractable utility knife with a standard trapezoid blade and a thumb slider on its spine, handle about 15 x 2.8 x 1.8 cm, about 150 g. Its slider button stands about 5 mm proud of the spine; pushed 20 mm forward it puts the blade out 20 mm (from the nose to the tip along the edge), to its stop, where it clicks into a notch, as it does at the drawn-in stop. It lies on its side on the table, tip forward, blade drawn in, slider facing the right arm. The blade is used at its full 20 mm, the slider's last stop: the knife is not set to a shorter stop, so the depth of the cut comes from how the knife is held: its angle with the nose on the board, or its height when the nose is off it.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Product's top under the tape's top | 6 mm | A ruler through a test slit in a spare carton, or by stacking card sheets to the height |
| Blade out | 20 mm | A ruler from the knife's nose to the tip, the slider at its stop |
| The slider's hold | 2 N of friction anywhere along its travel, plus a notch at each stop: within 1.5 mm of the stop, a weak spring (at most 0.45 N) pulls the blade onto it | **Not measured, and a simplification:** a real slider clicks into notches that take more than 2 N to leave; we set it low because the simulated arm, position-controlled, stalls against more, and the friction alone let a drawn-in blade creep 0.7 mm back out, so the notches hold it at its stops. Hook a spring scale on the button and pull it along the spine: the reading as it leaves a notch. |
| Drag while the tip is under the tape | 3 N | **Not measured.** Draw the knife through the same tape on a taped spare carton with a luggage scale on the handle, at about 4 cm/s, and set `DRAG_N` in `task.py` to the steady reading |
| A freed flap's rise on its crease | about 10 degrees (the crease spring rests at 15, less the flap's own weight) | **Not measured, and no published source.** It is our assumption that a fresh crease springs back a little. Slit a spare carton's tape and read each flap's angle with a phone level, then set the crease spring in `task.py` so the simulated rise matches. The simulator counts a flap free at 5 degrees, so that must sit below the measured rise |

## Safety for a real trial

The robot pushes out and holds an open 20 mm blade. Keep people out of the arms' reach while it runs, with a guard or a marked zone, and keep an emergency stop in reach of the operator. Handle and reset the knife with cut-resistant gloves, check the blade is drawn in before anyone takes it from the table, and start new motions at low speed until they have been watched once.

## Scoring

See `scoring` in `task.json`, and `table_score`, `board_cut` and `Trial` in `task.py`, which `check_scorer.py` tests on hand-built values. Three checks, each by eye or photo in a real trial:

- **The box cut open:** the tape is slit along the whole seam, edge to edge; lift each flap by its free edge to 30 degrees (a protractor, or the free edge 7.4 cm up): it must rise without tearing tape. Tape torn or peeled by a gripper does not count.
- **Nothing cut that should not be:** no knife mark anywhere on the product box's top, and no cut in the carton's cardboard outside the seam's 48 mm tape strip: not in a flap's face beside the tape, not in a wall.
- **The blade back in:** at the end, none of the blade past the knife's nose.

All three, held for 3 s: 1.0. The box opened without the rest: 0.5. Otherwise 0, or if the carton tips over.

In the simulator, the tape holds each crease hinge locked until every 2 cm piece of it is cut, and a piece counts as cut when the blade's exposed edge has passed through it at least 0.5 mm under the tape's surface. Freed, the flaps spring up on their creases, and the box counts as open once both have risen 5 degrees or more: the simulator's sign of free. The blade has no collider, so a cut is read from where its corners go: any corner below the product's top, over the product, is a mark on the product; any corner inside a wall, or inside a flap's board (in the flap's own frame, as it stands) more than 24 mm from the seam, is a cut beside the seam. Board under the tape strip counts as part of the seam. The blade counts as back in when less than 0.5 mm of it is out.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | The left hand pushes the blade out 20 mm; the seam slit edge to edge; no mark on the product (the blade's closest approach to it 3.0 mm) and no cut beside the seam; both flaps risen 10 degrees; the blade drawn back in, 0.0 mm out; then the knife put down | 1.0, at step 1363 (68 s) |
| `deep` | The seam's first third cut 7 mm deep with the knife at 20 degrees: into the product and the near wall. Its second stroke falls short of the far end, so the box is not opened | 0.0 |
| `half` | The seam cut 60 percent of the way, the blade drawn in: the box is not opened | 0.0 |
| `noretract` | The whole cut, the knife put down with its blade still out | 0.5 |
| `astray` | Before the seam, a 5 cm cut in a flap's face 4 cm from the seam, outside the tape: a cut beside the seam | 0.5 |
| `lockpush` | 5 N pushing up on each taped flap's free edge for 3 s | the most a flap rose: 0.001 degrees |
| Dream-1 | One run of 2,400 steps (120 s) at 320 x 176, seed 1, one model session throughout: the right arm reached toward the carton's near corner; the knife was never picked up and the box stayed shut | 0.0 |
| `check_scorer.py` | The scoring table, the 3 s hold, the blade test, the board test, the tape rule | 23 cases agree |

The demo's motion is deterministic, and with the scorer in `task.py` it reaches 1.0 at step 1363, before the knife is put down (the film goes on to put it down). None of the failure runs' outcomes turns on which hand pushed the blade out or where the knife ended up, so each scores as shown under the scorer in `task.py`. The knife pointed up to 13 degrees off the seam (see above); that is not scored.

No seed is needed: nothing in the scene is random. Runtime per trial: the demo's 1,363 steps take 4 to 10 minutes on one GPU, at 2 to 7 steps per second; a run to the 2,400-step limit takes 6 to 18 minutes.

## Assets

All meshes are made by the task's own code from plain shapes. No third-party assets, no textures.

## Licence

MIT
