# Close both valves and open the bleed before taking the test cap off

A pipe spool with a block valve at each end, a bleed valve in the middle, a test port with a cap on it, a pressure
lamp and a tray. The robot is told to take the cap off: close both block valves, open the bleed, wait for the lamp to
go out, then lift the cap into the tray. That order is the point. A line shut off at one end is still fed from the
other, and a line shut off at both ends still holds its pressure until it is bled, so the cap comes off only after all
three handles are thrown and the lamp has gone out. The sim keeps two things from the whole trial: whether the bleed was
ever opened to a live line, and whether the cap ever moved while the lamp was lit.

Beside the single-valve verify-zero-pressure task, this one asks for a two-valve proved isolation with a bleed between
the valves, and it needs both arms: the left closes the left valve and moves the cap, the right closes the right valve
and opens the bleed.

## Files

| File | What |
|---|---|
| `double_block_and_bleed_before_removing_the_cap.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes with sliders (the three handles), and per-step hooks |
| `plantkit.py` | The lamp, the clock and the slider and placement checks |
| `test_double_block_and_bleed_before_removing_the_cap.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=double_block_and_bleed_before_removing_the_cap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the test cap off the spool: close the left valve and the right valve, open the bleed valve, wait until the pressure light goes out, then lift the cap off and put it in the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dbb-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=double_block_and_bleed_before_removing_the_cap.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Take the test cap off the spool: close the left valve and the right valve, open the bleed valve, wait until the pressure light goes out, then lift the cap off and put it in the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dbb-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/dbb-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once the cap sits still in the tray for 1 s with both block
valves closed, the bleed open and the lamp out, the cap never moved while the lamp was lit, and the bleed was never
opened to a live line; otherwise it scores the last step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_double_block_and_bleed_before_removing_the_cap.py
```

## Scoring

A block valve is closed, and the bleed open, when its handle is past 70% of its 3 cm travel (2.1 cm). The lamp goes
out 2.0 s after both block valves are closed and the bleed is open, and comes back on at once if any handle goes back;
on a real bench, limit switches at the handles' 2.1 cm marks drive it through a 2 s off-delay relay, or an operator
switches it on the same rule. For credit, the cap is on its seat when its centre is within 3 cm of the port, its bottom
within 2 cm of the seat's top and it leans under 35 degrees; it is in the tray when its centre is inside the 8 cm
opening and its bottom under 3 cm above the table, standing or lying. The two hazard latches read tighter than that:
the bleed counts as opened once its handle passes 30% (9 mm), and the cap counts as moved once its bottom is 5 mm up,
its centre 1 cm across, or it leans past 35 degrees.

| Outcome | Score |
|---|---|
| Both block valves closed, bleed open, lamp out, cap lifted after the lamp went out and set in the tray | 1.0 |
| As above, but the cap ends off its seat and not in the tray (held, or down on the table) | 0.6 |
| Both block valves closed, bleed open, lamp out, cap still on its seat | 0.6 |
| Both block valves closed, bleed shut (or the lamp not yet out) | 0.3 |
| One block valve closed, or neither | 0 |
| Bleed handle past 9 mm while a block valve was not closed, at any time, whatever else was done | at most 0.2 |
| Cap moved (5 mm up, 1 cm across or knocked past 35 degrees) at any time while the lamp was lit, including a valve reopened or the bleed shut again after the cap came off | 0 |
| A valve body moved over 3 cm or tipped over 20 degrees | 0 |
| Nothing done | 0 |

## Design notes

- **Order is what's scored.** The end state alone can't tell a right trial from a wrong one, so the step hook keeps
  two flags for the whole trial. `vented_live`: the bleed handle passed 30% while a block handle was short of 70%.
  That caps the score at 0.2, under the 0.3 for an honest stop with both block valves closed and the bleed shut.
  `removed_live`: the cap moved at a step when the lamp was lit. That scores 0, under every honest partial result.
- **The latches are tighter than the credit checks.** Credit reads the cap loosely (3 cm, 2 cm, 35 degrees) so an
  untidy but safe placement still counts, and the bleed at 70%. The latches read 1 cm, 5 mm and 30%, so a cap nudged
  on a live line, or a bleed cracked a third of the way open on one, is caught long before either would earn or lose
  credit on its own.
- **Reopening or re-shutting after the cap is off is removing it live.** The lamp follows the handles every step, so
  a block valve put back, or the bleed shut again, with the cap off relights it with the port open: `removed_live`
  latches and the trial scores 0.
- **The lamp is a timer.** It goes out 2.0 s after the third handle is thrown. The bleed push in the demo ends at
  11.1 s, so the lamp is out by 13.1 s at the latest; the left arm starts for the cap at 14.1 s and its grip closes on
  the cap at 19.5 s, over 6 s after the lamp is due out. A robot that closes everything and lifts the cap at once
  loses everything.
- **Nothing springs back.** The handles have no springs and stay where they are pushed; the scorer reads them at the
  end and the hook reads them every step.
- **Why the pipe is in lengths.** A loose valve body (3 kg, so it stays put when pushed) can't share space with a
  fixed box, so each body stands on its own base plate down to the table and the spool is four fixed lengths that stop
  1 mm short of the bodies. The pipe reads as one spool with a valve body sitting in it.
- **Pushing along x.** The fingers close along y, so a push along x leads with the finger body. Its half-extent along
  x, measured on the gripper's meshes under the straight-down grip, is 0.8 cm at the pad tips, 2.0 cm at 2.2 cm above
  the TCP and about 2.8 cm by 4 cm up. The handles are 4 cm tall and the TCP runs 1.8 cm above the body top, so a
  handle's top stays within 2.2 cm of the TCP and meets the 2.0 cm face: the push starts 5 cm behind the handle's
  face in TCP terms (3 cm clear of it), runs the 3 cm of travel and 6 mm past the stop. The face first touches the
  handle's top edge, 11 cm above the table on the blocks and 10.5 cm on the bleed; each body's base plate is 2 cm wider
  than the body all round, so tipping it from there takes 12 N (blocks) or 11 N (bleed), and the overshoot's few
  newtons can't.
- **The lamp is out of the cameras' way.** The wrist camera rides 8.0 cm ahead of the TCP and 13.4 cm above it, so it
  leads the gripper over whatever it pushes towards. The lamp therefore sits behind the pipe between the bleed and the
  left valve, on a 4 cm post, where neither arm's camera passes within 5 cm of it, and off the midline it shows over
  the pipe from the top camera. A test traces both cameras through the demo.
- **Unintended strategies.** Pushing the cap 1 cm off its seat or knocking it over on a live line is removing it: 0.
  Throwing the cap into the tray after the lamp is out counts if it lands inside, as a real cap would. Shoving a valve
  body a few millimetres while pushing its handle changes nothing: the handle is read in the body's own frame. A body
  pulled 3 cm or tipped 20 degrees is a wrecked valve: 0. A handle short of 2.1 cm is not thrown. Opening the bleed
  first and then doing everything right is capped at 0.2.
- **The rule behind it.** HSE's [HSG253](https://www.hse.gov.uk/pubns/books/hsg253.htm), The safe isolation of plant
  and equipment, sets out double block and bleed as a proved isolation (category II): two closed valves in series with
  a bleed between them, the closure proved at the bleed before the line is broken into. OSHA's
  [29 CFR 1910.147(d)(5)](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.147) requires stored
  energy to be relieved before work starts.
- **What isn't modelled.** No fluid or pressure: the lamp's timer stands in for a gauge, and moving the cap live
  sprays nothing. The cap rests on the port rather than screwing on. On a real bench the handles are marked OPEN and
  CLOSED (SHUT and OPEN on the bleed) so a photo scores them.
- **The arms end clear of the scene.** The right arm parks over its own side once the bleed is open, and the left arm
  moves off the tray after setting the cap down, so the end photo shows the handles, the lamp and the tray.

## Runtime and seeds

- The scripted demo takes about 29 s of simulated time (586 steps at 20 Hz); the right arm is done at 14 s.
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
