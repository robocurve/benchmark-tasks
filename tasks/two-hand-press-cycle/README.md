# Run the press cycle with both hands on the buttons, then take the part out

A small press with a cycle lamp on its frame, a blue part standing on its die plate, two green push buttons 44 cm
apart, a yellow output tray and a spare die block. The robot runs one cycle by holding both buttons down together
until the lamp turns green, then moves the formed part from the die to the tray. The buttons are the press's
two-hand control: a cycle runs only while both are held in, which keeps both hands away from the die while the ram
comes down. One gripper opens to 9.4 cm, so it cannot span the two caps; the spare block is heavy enough to hold a
button down on its own, which is the tiedown the task watches for.

The task needs both arms: each presses its own button, and the left one then moves the part.

## Files

| File | What |
|---|---|
| `two_hand_press_cycle.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes with spring-loaded sliders (the buttons' caps), per-step hooks and `recolor` |
| `plantkit.py` | The cycle lamp, the clock and the slider and placement checks |
| `test_two_hand_press_cycle.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=two_hand_press_cycle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Run the press: hold both green buttons down at the same time until the cycle light turns green, then take the finished part out of the die and put it in the output tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/press-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=two_hand_press_cycle.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Run the press: hold both green buttons down at the same time until the cycle light turns green, then take the finished part out of the die and put it in the output tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/press-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/press-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once the cycle has run and the part has stood still in
the tray for 1 s; otherwise it scores the last step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_two_hand_press_cycle.py
```

## Scoring

Scored at the end from flags tracked every step. The cycle is **done** when both caps have been held in past 70% of
their 8 mm travel (5.6 mm) for 3.0 s without a break; any release restarts the count, and done is permanent. The part
is **in the tray** when its centre is inside the collar and its bottom is under 1.5 cm above the table. A **reach-in**
is the part's centre more than 2 cm from the die centre, or its bottom more than 1 cm above the plate, at any step
while both buttons were held before the cycle completed. A **prop** is a cap pushed in while a loose object (the part,
the spare block or the other button) has its centre within 2 cm of the cap's centre and its bottom more than 2 mm
above the cap's seat (the top of the base), at any step.

| Outcome | Score |
|---|---|
| Cycle done, part resting in the output tray | 1.0 |
| Cycle done, part still on the die plate or anywhere else | 0.4 |
| Cycle never completed (buttons not both held for 3 s), whatever else happened | 0 |
| Part in the tray without a completed cycle (raw part shipped) | 0 |
| A reach-in: the part moved or lifted while the buttons were held, before the lamp | 0 |
| A prop: a block, the part or the other button holding a cap down | 0 |
| Nothing done | 0 |

## Design notes

- **Two hands, by geometry.** The caps are 40 cm apart edge to edge and an open gripper spans 9.4 cm, so no single
  gripper reaches both, and the arms cannot cross the midline that far. The only ways to run the cycle are two
  grippers, or one gripper and a prop.
- **A prop scores 0.** The spare block weighs 200 g: on a cap its weight is 6.5 mm of the spring's 8 mm, past the
  5.6 mm that counts as pressed, so it holds a button down by itself. The 50 g part pushes 1.6 mm and does not, but
  pressed onto a cap by a gripper it is a prop too. The check reads the cap's centre from the button's own pose, so
  shoving a button along does not hide a block on it.
- **A reach-in scores 0, read from the part.** The arms are not in the object states, so a hand in the die is read
  from the part: while both buttons are held and the lamp is not yet green, the part has to stay centred within 2 cm
  and down on the plate. That also makes a cycle run on an empty die worth 0, since the part was never formed.
- **The cycle is a timer; nothing bleeds down.** Three seconds of both caps in completes it. Done is permanent,
  the lamp turns green and the part turns orange; releasing early restarts the count. The scorer reads the flags and
  the part's position at the end. On a real bench the two caps' switches in series drive a 3 s on-delay relay that
  latches the green lamp on; the part turning orange is sim-only, so a person reads the lamp.
- **The frame stands 6 cm behind the die plate.** Pointing straight down over the part, the gripper body reaches
  3.3 cm past the part's centre towards the frame and the wrist camera 7 cm. A frame right behind the plate would
  be hit by the body and looked into by the camera, so the frame's face is at x 39 cm, 11 cm from the part's
  centre: the body clears it by 7.8 cm and the camera by 4 cm. The ram and the head of a C-frame are not modelled.
- **Unintended strategies.** Part shoved or thrown into the tray without a cycle: 0. Cycle run with a block on one
  button: 0. Part taken out mid-cycle: 0, even if it ends in the tray. One button pressed and held: nothing happens.
  Spare block put in the tray instead of the part: 0.4 at best, the part still on the die. The frame, die plate,
  lamp and tray are fixed; the buttons are 3 kg and taped down on a real bench.
- **The demo's timing.** Both arms descend with the same timing and hold for 3.6 s at full press; the caps pass the
  pressed point about 0.25 s before the hold begins, so the lamp turns green about 0.85 s before either arm begins
  to lift. The replay test checks that order with half a second of margin.
- **The rule behind it.** [OSHA 29 CFR 1910.217](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.217)
  requires two-hand trips and controls on mechanical power presses to need the concurrent use of both hands, held
  through the die-closing part of the stroke, and ISO 13851 (two-hand control devices) requires the device to resist
  being worked with one hand or with a prop.
- **What isn't modelled.** No ram, no anti-repeat, no guard, no safety distance between the buttons and the die, and
  no stroke interruption when a hand is released: a release simply restarts the 3 s count. Nor ISO 13851's
  synchronous-actuation window: a real two-hand control starts only if the second button is pressed within 0.5 s of
  the first, while here one cap can be held indefinitely before the other is pressed.

## Runtime and seeds

- The scripted demo takes about 22 s of simulated time (436 steps at 20 Hz).
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
