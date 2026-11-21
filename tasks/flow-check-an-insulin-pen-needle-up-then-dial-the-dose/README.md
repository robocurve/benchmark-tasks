# Prime an insulin pen needle-up, then dial 24 units

An insulin pen (the maker's training version of a spring-driven FlexTouch pen) lies in its tray with a needle on.
The maker's instructions require a 2-unit flow check before every injection: turn the dose selector to 2 units, hold
the pen with the needle pointing up, press and hold the dose button until the counter shows 0, and see a drop at the
needle tip; then turn the selector to the prescribed dose. The robot dials with the pen level, tips it upright in the
hand, presses and holds the 5.1 N dose button with the needle up (the demo pushes the button end down on the mat; any
method counts), tips it back and dials 24 units, never letting the bare needle touch anything.

Instruction: "Do the insulin pen's 2-unit flow check, then dial a 24-unit dose and hold the pen ready."

## Run and score a trial

    # scripted demo (the task file's demo) on the Isaac Lab arena
    python -m yam_arena.run --task tasks/flow-check-an-insulin-pen-needle-up-then-dial-the-dose --policy yam_scripted --steps 2400
    # tests (kit tests 1-7)
    cd tasks/flow-check-an-insulin-pen-needle-up-then-dial-the-dose && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q

- Run limit: 2,400 steps (2 minutes at 20 Hz). Runtime per trial: about 60 s for the demo (it ends at 1.0 near 58 s);
  each run prints its `task_score`.
- One fixed layout, no randomisation: seed 0, object jitter 0.
- The score is read at the end of the run by `score()` in `flow_check_an_insulin_pen_needle_up_then_dial_the_dose.py`
  (top of the file: the pen's mechanism, the scene, the pen's rules, the scorer, then the demo).

## The dose window

The pen's dose window sits on top of the barrel by the selector: two seven-segment digits whose segments are recoloured
every control step from the selector's click count (2 units a click). While the spring pushes a dose out, the window
counts the units left down to 0, as the real counter does. The numerals are drawn 14 mm tall (the real window's are about
4 mm) so the wrist camera can read 2, 0 and 24 at 224 px. The task file's trace prints the window's value every 0.5 s.
After the last turn the demo's right hand holds its wrist camera 9 cm from the window, looking down at it at 45 deg,
for about 2 s. In the Isaac run the right wrist camera, downscaled to 224 px, reads 2 at 11.5 s (after the first
click), 0 at 37-38 s (after the flow check, as the hand lines up on the selector) and 24 from 56.0 s to the end at
58.4 s; the window showed the selector's count at every 0.5 s trace tick.

## Runs

- Isaac (GPU, yam-isaac-lab-arena), this task file: scripted demo 1.0 (2 units set at 11 s, the flow check's press
  at 24.5 s, 24 units at 52.5 s, 1.0 from 56.5 s). Dream-1 0.0 over the full 2 minutes (the counter never left 0).

## Scoring (from task.json)

0 at any time if the needle touches anything, the pen falls, one press pushes out more than 2 units, or a seventh
press; 0.25 the counter set to 2 with the pen lifted 2 cm off the tray; 0.5 a flow check (2 to 0 with the needle within
30 degrees of straight up throughout, and a drop at the tip); 0.75 after it, the counter at 24 units; 1.0 the counter at
24 and the pen held still for 2 s with the right gripper 3 cm clear.

## Real trials

- Pen: Novo Nordisk Demonstration Pen for FlexTouch (2-unit increments, 3 mL of liquid test medium; the maker supplies it
  to healthcare professionals for training, per its instructions sheet, which names NovoFine needles for it), NovoFine
  32G 6 mm needles. A clinical skills lab can order it through a Novo Nordisk representative. Fit silicone pad covers on
  the gripper fingers.
- Needle-up: a side video with a tape scale behind the pen; the needle end at least 13 cm above the button end
  throughout the press. Drop: a close-up photo of the needle tip after the press. Counter: read in the dose window.
- Reset: dial back to 0 by turning the selector backwards (nothing is pushed out), wipe the tip, lay the pen back in
  the groove. Used needles go in a sharps container; keep people clear of the arms during a trial.

## Safety

The needle is a sharp: fit silicone pad covers on the gripper fingers, keep people clear of the arms during a
trial, put used needles in a sharps container, and recap only with the one-handed scoop method. The pen's button needs
only 5.1 N, so the real trial is safe for the arms and the pen; the test medium is not insulin.
