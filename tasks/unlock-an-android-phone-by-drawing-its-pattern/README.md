# Unlock an Android phone lying loose on a desk by drawing its Z pattern with one fingertip, never lifting it

**Instruction:** "Unlock the phone by drawing its pattern with one fingertip, without lifting it: along the top row of dots from left to right, diagonally down to the bottom-left dot, then along the bottom row from left to right. Then take your hand off the phone."

A Google Pixel 7a lies loose and screen up on a desk, showing stock Android's lock-screen pattern pad (nine dots in a
3 x 3 grid). The robot unlocks it by drawing the stored pattern, a Z through seven dots, with one fingertip that stays
on the glass from the first dot to the last, touches no other dot, and does not push the phone away.

## Scoring

Read from the phone (in the real trial: its screen and the overhead video, where Make pattern visible shows the drawn line; lifting submits a pattern). LEAD of an attempt: how many of its dots match the pattern from the start, counting the one still being drawn. Score 0 if the phone turned over (tilted over 45 degrees) or left the desk (20 mm below it). Otherwise the lower of two numbers. Screen: 1.0 unlocked with no earlier attempt; 0.75 unlocked after any earlier attempt (wrong dot, too short or lifted early); 0.5 not unlocked and some attempt has LEAD 4-7; 0.25 best LEAD 1-3; 0 no attempt began on the first dot (includes doing nothing). Phone: 1.0 centre within 20 mm of its start, turned at most 10 degrees, and for 1 s every fingertip 20 mm or more above the glass or 10 mm outside the phone, phone still; 0.75 in place but the hand not yet off; 0.5 moved more than 20 mm or turned more than 10 degrees. Success: 1.0 held 1 s more.

Dots are numbered 1-9 from the top left, row by row; the pattern is 1-2-3-5-7-8-9. Every outcome has one score:

| Outcome | Score |
|---|---|
| Unlocked at the first attempt, phone in place, hand off | 1.0 |
| Unlocked after a wrong, short or lifted attempt | 0.75 |
| Unlocked, phone slid over 20 mm or turned over 10 degrees | 0.5 |
| Not unlocked; an attempt went 1-2-3-5 or further in order | 0.5 |
| Not unlocked; best attempt 1, 1-2 or 1-2-3 | 0.25 |
| No attempt starting on dot 1, nothing done, phone turned over or off the desk | 0 |

Why tapping scores low: lifting submits the pattern, so tapping the dots one by one enters seven one-dot patterns,
each refused as too short; the phone stays locked and the best attempt matches only dot 1 (0.25).

**Real trial, read from the phone and the video.** The phone itself decides: it opens to the home screen or shows
"Wrong pattern" with the drawn pattern in red. With Settings > Security > Make pattern visible on, the overhead
video shows the line drawn through the lit dots, so which dots were joined, in what order, and whether the finger
lifted early can be read frame by frame (screen recording often stops on the lock screen). Phone movement: a
masking-tape outline and a ruler; 10 degrees is the long edge 26 mm off its line at the far end.

## The pattern pad (simulated and real)

The simulated pad is a port of Android 14's `LockPatternView` (frameworks/base,
`core/java/com/android/internal/widget/LockPatternView.java`, android14-release) and the keyguard's check:

- Hit test (`detectCellHit`): a touch point hits a dot when it is closer than `min(cellWidth, cellHeight) / 2 x
  lock_pattern_dot_hit_factor` to its centre; the factor is 0.6 (`core/res/res/values/dimens.xml`), so 0.3 x the
  dot pitch: 5.7 mm here.
- A dot already in the pattern is never hit again (`checkForNewHit`).
- Gap fill (`detectAndAddHit`): a move two rows and/or two columns away from the last dot (not a knight's move)
  first adds the dot between them if it is not used yet.
- Touch down starts a new pattern (a hit only if it lands on a dot); every move is hit-tested; lifting submits the
  pattern if it has any dot.
- Keyguard (`KeyguardPatternViewController`, `LockPatternUtils`): fewer than 4 dots is shown as wrong but not
  counted; a wrong pattern of 4 or more counts as a failed attempt; after 5 failed attempts the pad is locked for
  30 s. A wrong pattern stays drawn in red for 2 s.
- Only one finger is followed (Android reads the first pointer); a second touch is ignored.

Sizes on this phone (estimates from Android's layout values; check with one screenshot on the phone): 1 dp = 0.155
mm (1080 px across a 63.6 mm screen at density 2.625); the pattern view is square and about 57 mm wide, so the cells
and the dot pitch are 19 mm and the hit radius 5.7 mm; dots 14 dp (2.2 mm), lit dots 30 dp (4.6 mm), line 22 dp
(3.4 mm). The pad's centre is taken 30 mm below the screen centre.

Each target dot must be passed within 5.7 mm; the unused dots 4 and 6 lie 13.4 mm from the diagonal.

In simulation the screen is live: every control step the task finds the fingertip end faces from the joint readings,
decides which touch the glass (within 0.2 mm), runs the rules above and redraws the screen (drawn-only meshes under
the phone's body). The console prints `PLLOG` lines and each pattern event as a `[pattern]` line.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=unlock_an_android_phone_by_drawing_its_pattern.py \
    --instruction "Unlock the phone by drawing its pattern with one fingertip, without lifting it: along the top row of dots from left to right, diagonally down to the bottom-left dot, then along the bottom row from left to right. Then take your hand off the phone." \
    --policy yam_scripted --scorer task_score --max-steps 1200
```

- Scene `station`, 20 Hz, 1,200 steps. `--scorer task_score`: 1.0 on success, otherwise the score at the end. The
  demo reads the phone's pose once at the start, so seeded shifts of the phone are fine.
- Offline checks: `pytest` in this folder (scorer on the goal, partial and cheat layouts; demo reach and length).

## The scripted demo

The right gripper closes its two fingertips together, goes over dot 1, presses 3.5 mm past the glass (a light press;
commanded exactly to the glass, a sagging arm hovers and registers nothing) and slides 1 → 2 → 3, pauses 0.8 s on the
corner, slides the diagonal 3 → 5 → 7, pauses, slides 7 → 8 → 9, then lifts straight off and moves away. The left arm
stays parked. The plan takes 23.8 s. Pressed on the glass, the fingertip's friction holds the compliant arm back: run open loop, the touch
lagged up to 10 mm behind the path, cut the first corner and lit only dots 1 and 2. So the demo watches its own
fingertip, as a person watches their finger: every step while touching it reads the touch point from the joint
readings and shifts the rest of the path by an integral correction (a quarter of the error per step, at most 1 mm a
step). In the recorded demo the touch passed within 1.3 mm of every pattern dot and 13.4 mm or more from dots 4 and 6; the phone unlocked at 18.7 s and did not move; success at step 419. An earlier run of the same demo code with the phone shifted up to 10 mm by seed 7 also succeeded in 3 of 3 episodes.

Wrong-way run (tapping each dot, lifting between): 0.25, for the reason above.

Scores: Dream-1 run: 0.0; demo: 1.0.

## Real-trial touch hardware (required)

Bare robot fingertips do not register on a capacitive screen. Close the gripper and wrap both end faces together in conductive fabric tape (for
example Adafruit 3960, conductive nylon fabric tape, about US$4: https://www.adafruit.com/product/3960), and run a
thin wire from the tape to the robot's grounded frame (for example an alligator-clip lead, Adafruit 4100, about
US$3: https://www.adafruit.com/product/4100). The two taped faces make one contact patch of about 15 x 8 mm, close to
a fingertip's; the panel reports one touch at its centre. Only the taped faces are conductive. Before the trial,
check with a touch-test app that the tip registers a touch, a drag and a release. This follows the task folder `pinch-zoom-map-on-tablet-to-fit-route`, whose live-screen kit (fingertip faces from the joint readings,
touch rule, drawn-only screen layers, per-step loop, episode memory) this task adapts.

## Does the phone need steadying?

No mat, tray or holder. One sliding fingertip drags the phone one way: about 0.35 x 2 N = 0.7 N against about
0.4 x (1.9 + 2) N = 1.6 N of desk friction (estimates). In all nine simulated episodes (four at a 5 mm press, four at the demo's 3.5 mm, one tapping) the loose phone
moved 0.01 mm or less and did not turn; pressing much harder or catching its edge would move it. A person may steady
it with the other hand's fingers on its long edges; here the left arm cannot reach them (over 0.45 m from its base),
so the demo leaves the phone loose.

## Real-world parts

- Google Pixel 7a, any colour, no case (dimensions and mass: https://en.wikipedia.org/wiki/Pixel_7a,
  https://www.tomsguide.com/news/google-pixel-7a; about US$200 used, estimate), set up as in the setup field. Any
  Android 12-15 phone of similar size works if the pad's position is taken from its own screenshot.
- Conductive fabric tape and a ground lead (about US$10).
- Masking tape for the phone's outline on a laminate desk.

## What is simplified

See the fidelity note. Also: "Wrong pattern" is drawn as a red bar where the text appears; clock and home screen are
plain shapes. Reset: see the reset notes.
