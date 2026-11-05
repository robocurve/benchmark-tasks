# Set up a machine from its setup sheet and start it with a two-hand control

Operating machinery is a core job on shop floors, in labs and in kitchens. Before a run, the operator sets the
machine up from a printed setup sheet (the program, its set points and options) on the machine's control panel,
and then starts it. Presses, cutters and other dangerous machines are started with a **two-hand control**: by safety
standards (ISO 13851) both hands must be on two buttons, pressed together and held, so that neither hand can be in
the machine. Here the robot reads a setup sheet lying on the table, finds the line for the program it is asked for,
works three kinds of controls (rocker switches, a detented dial, friction sliders) to the stated settings, and
starts the machine with the two-hand control only once it is set up. The two start buttons are 26.5 cm apart, so
one gripper cannot press both: the start needs both arms.

The task is the task file `busybox_manual.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1), with the panel's code in `busybox/` and `yampatch.py`
beside it (see [Implementation](#implementation)). It needs no other files or packages to be installed.

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

| Object | Size | Material | Where |
|---|---|---|---|
| control panel (busybox), loose | case 25 x 37 x 5 cm (x by y by z), 2.5 kg | dark grey case, grey printed module plates, plastic controls | centred at (30, 0) cm |
| setup sheet, flat | 20 x 32 cm, 1 mm | white card, black print | centred at (6.5, 0) cm, between the arm bases and the panel, 1 cm short of it, text upright from the arms' side |

The panel's top holds six 11 x 11 cm modules in two rows, left to right as seen from the arms:

| Row | Module | Parts |
|---|---|---|
| far | switches | 4 white rocker switches (33 x 15.6 mm) in 5 mm black frames, rocking 20° either way; far end down is ON; a red lamp above each, lit while on; numbered 1 to 4 from left to right |
| far | dial | black bar knob 42 x 12 x 18 mm on a 40 mm disk, white pointer; 8 detents 45° apart; numbers 1 to 8 clockwise, 1 pointing away from the arms |
| far | sliders | 2 black knobs 12 x 20 x 16 mm, 72 mm travel along x against 1 N of friction; marks 1 (near) to 5 (far) printed between them |
| near | START | big button, 56 mm white cap labelled START, 5 mm travel |
| near | keypad | display reading WATER above six 23 mm keys with Greek letters (not used) |
| near | buttons | four 26 mm buttons, red and green (far), yellow and blue (near), 4 mm travel |

A serial sticker (KD4XR7) is on the case's left side face. **Start:** dial at 3, left slider at 2, right slider
at 4, switches 1 and 3 on, 2 and 4 off.

**Building the panel for a real station:** a plywood or ABS case with printed plate labels and off-the-shelf
panel parts of the sizes above: four illuminated (red) rocker switches; an 8-position rotary selector switch
(45° detents) with a bar knob; two 75 mm slide potentiometers or friction sliders with 12 x 20 mm knobs; four
26 mm momentary push buttons (red, green, yellow, blue); a 56 mm mushroom-head momentary start button with a
white cap labelled START; and a small keypad with a display (unused, may be dummy). Read out by eye (or wired to a
microcontroller for logging), the settings and the two-hand start are judged as in [Scoring](#scoring).

The setup sheet reads:

| PROG | DIAL | LEFT SLIDER | RIGHT SLIDER | SWITCHES ON |
|---|---|---|---|---|
| P1 | 2 | 1 | 5 | 2 3 |
| P2 | 5 | 4 | 2 | 1 4 |
| P3 | 7 | 3 | 3 | 1 2 3 |

followed by "Other switches off." and "To start: press START and GREEN together, hold 2 s". The instruction asks
for program P2, so every setting has to change from the start.

## Scoring

- **0.15** each for the dial, the left slider (its nearest mark), the right slider (its nearest mark) and the four
  switches as P2 says, judged at the end of the trial.
- **0.4** for a two-hand start made while all four were right: START and the green button pressed down (beyond
  60 % of their travel) within 0.5 s of each other and held down together for 2 s. A start with wrong settings
  earns nothing; correcting them and starting again counts.
- **1.0 (success, ends the trial):** started while set up, and still set up 1 s later.
- **0** if the red, yellow or blue button or a keypad key was pressed at any time.

On a real station the settings are read off the panel (a photo) and the start from the video. In simulation the
panel's joint positions are read every control step (`busybox/state.py`); run with `--scorer task_score`.
`BUSYBOX_DEBUG=1` prints every button event, control change and start.

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
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/busybox-manual/busybox_manual.py \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the machine up for program P2 as the setup sheet says, then start it." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir runs/busybox-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted` (and `--log-dir runs/busybox-demo`).
`inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz).
- **Seeds and layout:** one fixed layout. The panel is spawned as an articulation and the sheet is fixed, so
  seeded resets (`--seed`, `-E object_jitter`) move neither; any seed gives the same trial. Several epochs in one
  run are fine: every control and the start are reset with the episode.
- **Runtime:** the scripted demo succeeds after 509 steps (25.5 s simulated) with `task_score`
  1.0; the run took about 1 minute wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo: the left arm first holds its wrist camera 20 cm above the setup sheet for 1.5 s to read it (the
whole sheet is in view and legible at 640 x 360), then rocks switch 3 off (pressing its near end) and switch 4 on (its far end); the right arm
waits out to its side until the sheet has been read, then pushes the left slider from 2 to 4 and the right slider from 4 to 2 with its closed fingers, then grips the dial's
bar and turns it from 3 to 5; finally both arms press START and green together and hold them for 2.8 s.

## Simulation notes

- The case and all parts are rigid. The case stands loose on the table.
- Switches are bistable through a weak spring pulling each rocker to the side it is on, and the dial has spring
  detents: joint position drives whose targets are updated every control step. The sliders are held by 1 N of joint
  friction. Buttons are spring-loaded (about 2 to 3 N at full travel) and count as pressed beyond 60 % of their
  travel and as released again above 30 %. Lamps switch with the switches.
- The parts do not collide with the case or each other; joint limits bound their travel.
- Presses are sampled at the 20 Hz control rate. Physics runs at 240 Hz.
- The setup sheet is a thin fixed box with the print as a texture. The machine behind the panel is not modelled:
  the start is judged from the two buttons and the settings.

## Implementation

| Path | What |
|---|---|
| `busybox_manual.py` | the task file: the panel's configuration, the setup sheet, scoring and the scripted demo |
| `busybox/design.py` | the panel's dimensions, layout, joint mechanics and configuration (`BusyboxConfig`) |
| `busybox/textures.py`, `busybox/usd.py` | its label textures (PIL) and articulated USD (`pxr`): a case and 17 jointed parts |
| `busybox/state.py` | tracks the controls, button presses and hold times from the joint positions |
| `busybox/sim.py` | the panel as an Arena articulation, and its per-step detents, springs and lamps |
| `yampatch.py` | adds to the task-file format at runtime (without changing the adapter's files): spawning a custom Arena object (the panel) and USD files (the sheet), a per-step hook (`on_step`) and a reset hook (`on_reset`); it wraps `yam_arena.scene.task_object`, `yam_arena.scene.object_states` and `YamArenaEmbodiment.reset` of the pinned adapter |

The panel's USD and textures are built once per configuration into `~/.cache/busybox`
(`$BUSYBOX_ASSET_DIR`), the sheet into the system's temporary folder.

## Media

Rendered with `inspect-robots video <log>.json` (top camera):

- `media/setup.png`: the setup, the demo run's first top-camera frame (640 x 360).
- `media/demo.mp4`: the scripted demo, full score (1.0), 640 x 360.
- `media/dream.mp4`: the Dream-1 run at Dream's settings (320 x 176), score 0.0.

## Licences

`assets/fonts/`: DejaVu Sans Bold and DejaVu Sans Mono Bold, Bitstream Vera licence (see `assets/fonts/LICENSE`).
