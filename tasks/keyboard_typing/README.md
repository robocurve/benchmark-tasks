# Type a word with capital letters on a computer keyboard

Robots working with equipment made for people often have to type on an ordinary keyboard. Here the robot types
**RoboCurve** on a large-key keyboard with blank caps: it has to find each key from the standard layout, press
it within a few millimetres, and type the two capitals as chords, with Shift held down while the letter is pressed.

The task is the task file `keyboard_typing.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1), plus `yampatch.py` beside it (see
[Implementation](#implementation)). It needs no other files or packages.

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

| Object | Size | Material | Where |
|---|---|---|---|
| keyboard housing, taped to the table | 13.7 x 38.7 x 1.2 cm (x by y by z) | dark grey ABS | centred at (22.5, 0) cm, long side across the table |
| 61 key caps, blank | 23 x 23 mm, 8 mm tall (wider keys below); 25 mm pitch | ABS; light grey (characters, space bar) or dark grey | on the housing, 4 mm travel |
| computer with an empty text field | any | | screen out of the cameras' view |

The keys follow the standard US (ANSI) layout of a keyboard's main block, five rows, from far to near:

| Row | Keys (width in units of 25 mm if not 1) |
|---|---|
| number row (x 27.5 cm) | `` ` `` 1 2 3 4 5 6 7 8 9 0 - =, Backspace (2) |
| (x 25 cm) | Tab (1.5), Q W E R T Y U I O P [ ], \ (1.5) |
| (x 22.5 cm) | Caps Lock (1.75), A S D F G H J K L ; ', Enter (2.25) |
| (x 20 cm) | Left Shift (2.25), Z X C V B N M , . /, Right Shift (2.75) |
| bottom row (x 17.5 cm) | Ctrl, Win, Alt (1.25 each), space bar (6.25), Alt, Win, Menu, Ctrl (1.25 each) |

Every row is 15 units (37.5 cm) wide and starts at the housing's left edge. A key types when pushed down 2.5 mm of
its 4 mm travel and types again only after it has come back up. Shift types the shifted character (a capital, `!`
for `1`, ...); Caps Lock toggles capitals; Backspace deletes the last character. On a real station, any large-key
keyboard with this layout and pitch works, with its legends covered by blank matte stickers.

## Scoring

The text in the text field at the end of the trial:

- **1.0** if it reads exactly `RoboCurve`. The trial ends early once it does with no key held down for 1 s.
- Otherwise, counting from the first character for as long as the text spells the word in either case: **0.05**
  per character, plus **0.2** for each of the R and the C typed as a capital. Examples: `robocurve` 0.45,
  `Robocurve` 0.65, `RoboC` 0.65, `RoboCurv` 0.8, `RoboCurvex` 0.85, `Xrobocurve` or an empty field 0.

On a real station the text is read off the screen (a photo). In simulation the keyboard's key travel is read every
control step and turned into text as a US layout would type it (`Typist` in `keyboard_typing.py`); run with
`--scorer task_score`. `KEYBOARD_DEBUG=1` prints every keystroke and the text so far.

## Why both arms

A capital needs Shift held while the letter is pressed. The natural way, as in touch typing, is one hand on Shift
and the other on the letter: in the demo the right arm holds Right Shift while the left arm presses R and C. Two
arms also halve the travel between keys. (A single arm could also type the capitals with Caps Lock, toggling it on
and off around each capital.)

## Running

Needs Linux with an NVIDIA RTX GPU (Isaac Sim needs RT cores) and about 200 GB of disk. Install the pinned
adapter, which installs Isaac Lab-Arena and Inspect Robots (from the root of this repository):

```bash
mkdir -p .deps
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git .deps/yam-isaac-lab-arena
git -C .deps/yam-isaac-lab-arena checkout 0059d02a1a91bc6834c1d0846022891df0c78730
ARENA_DIR="$PWD/.deps/IsaacLab-Arena" .deps/yam-isaac-lab-arena/setup.sh
```

In each shell:

```bash
source .deps/IsaacLab-Arena/yam.env
export ISAACSIM_ASSET_ROOT=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0
```

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/keyboard_typing/keyboard_typing.py \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Type 'RoboCurve' on the keyboard, with a capital R and a capital C." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir runs/keyboard-typing-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted` (and `--log-dir runs/keyboard-typing-demo`).
`inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz).
- **Seeds and layout:** one fixed layout. The keyboard is fixed to the table, so seeded resets (`--seed`,
  `-E object_jitter`) do not move it; any seed gives the same trial. Several epochs in one run are fine: the typed
  text is cleared at every reset.
- **Runtime:** the scripted demo succeeds after 616 steps (30.8 s simulated) with `task_score` 1.0;
  the run took about 2 minutes wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo types letter by letter with the arm on the letter's side of the keyboard (left: Q-T, A-G, Z-B; right:
the rest), pressing each key with closed fingers, while the other arm waits out to its side. For R and C the right
arm presses and holds Right Shift first and lets go after the left arm's press.

## Simulation notes

- Each key cap is a rigid box on a frictionless vertical slide with a linear return spring (150 N/m, 0.6 N at full
  travel; real keys need about 0.5 to 0.7 N). No key wobble or stabilisers; the housing is a rigid box.
- The computer and screen are not modelled; the text is tracked from the keys' travel. Caps Lock has no light.
- Physics runs at 240 Hz, control at 20 Hz.

## Implementation

The stock task-file format has no articulated objects and no reset hook. `yampatch.py` adds both at runtime, without
changing the adapter's files: it spawns the keyboard as a USD articulation generated when the scene is built (one
fixed housing and a spring-loaded prismatic joint per key), calls the task's `on_step` once per control step to
read the keys' joint positions, and calls `on_reset` after every episode reset to clear the text. It wraps
`yam_arena.scene.task_object`, `yam_arena.scene.object_states` and `YamArenaEmbodiment.reset` of the pinned adapter.

## Media

- `media/setup.png`: the setup, top camera (640 x 360).
- `media/demo.mp4`: the scripted demo, full score (1.0): top camera above, the wrist cameras below, 640 x 360 per
  camera.
- `media/dream.mp4`: the Dream-1 run at Dream's settings (320 x 176), score 0.0.
