# Seat a DIP chip in a ZIF socket the right way round and close the lever

Loading a chip into a device programmer, the way an electronics technician burns firmware into a batch of
microcontrollers or EPROMs. The chip has to go in with its notch toward the socket's lever (pin 1 at the lever end),
which means reading a notch at one end and turning the chip half a turn in the air. The socket has a guide boss that only
fits the notch: the wrong way round, the chip sits 2 mm high at one end. Only then does the other hand push the
lever down, which clamps the pins.

| File | What it is |
|---|---|
| `zif_socket_chip.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `bonds.py` | `Hinged`: a task-file box on a revolute joint with limits and a two-detent drive (the lever). Also `Bonded` and `Sliding`, unused here |
| `parts.py` | `Compound`: a task-file box whose collider is several smaller boxes in one body (the chip with its notch and pins, the socket with its slots and boss) |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`) and their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_zif_socket_chip.py` | Offline checks: the file loads, every scoring outcome and the lever memory, the fits and play of the pins, slots and boss, the demo's order and reach (no GPU) |
| `task.json` | The task's details for the board |

## The socket, the chip and the lever

Positions are centres in metres from the midpoint between the arm bases (x forward, y to the robot's left).

| Part | Physics | Size |
|---|---|---|
| chip | one rigid body of five boxes: the body with a notch at one end (two cheeks beside the notch), and two pin rows under it as bars | body 15 x 52 x 5 mm, notch 6.5 mm wide and 7.5 mm deep, pin rows 2 x 48 x 3 mm, 6 g |
| socket | one fixed body of eight boxes: base, two rails, a middle ridge and end walls round two pin slots, the guide boss, the lever's cam block | 25 x 77.5 x 15 mm, slots 6 x 53 x 3.5 mm (2 mm of play across each pin row, 2.5 mm along), boss 2 x 1.5 x 2 mm |
| lever | a 4 x 4 x 45 mm bar on a revolute joint about x at its bottom end, 0 to 90 degrees, 8 g | a drive of 0.05 N m/rad with 0.0075 N m s/rad pulls it to 0 (open) or 90 degrees (closed), whichever is nearer: an over-centre spring, so it stays where it was pushed and swings shut once past 45 degrees |

The seated chip's body rests on the socket's top, its pins 0.5 mm clear of the slots' floor. The boss sits inside
the notch wherever the play puts the chip. Turned the wrong way round, the chip's solid end lands on the boss and the
chip sits tilted, 2 mm high at that end, its pins still in the slots. `parts.py` gives every chip and socket box a
2 mm contact offset and no rest offset, so the play is real.

## What it looks like

Each box keeps its collider, and `looks.py` draws a mesh in its place:

| Box | Mesh |
|---|---|
| `chip` | a black 40-pin DIP: epoxy body with the notch, a pin-1 dot and markings, 40 tinned legs |
| `lever` | a steel rod with a red tip |
| `socket` | the blue ZIF socket with 40 gold contacts in its slots, the boss, the cam block with a white mark and the hinge pin |
| `board` | the green programmer board with its traces, solder pads, controller chip, USB socket, crystal, capacitors, LEDs and header |
| `mat` | the light grey anti-static mat |

The chip's and the lever's meshes fit inside their boxes (a test checks it), so what the cameras see is what
collides. The board's parts are drawn only: none is taller than 1.1 cm, and none is under the arms' paths. To change
them, edit `looks/make_looks.py` and run it in Blender 4.2 or later: `blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=zif_socket_chip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the black chip in the blue socket with its notch toward the socket's lever, then push the lever down to close it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zif-socket-chip-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=zif_socket_chip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the black chip in the blue socket with its notch toward the socket's lever, then push the lever down to close it." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/zif-socket-chip-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
  Keep it at 0: jitter would move the lever off its hinge.
- The demo: the right arm grips the chip across its width (a partial grip, about 5 N: at 10 N the light chip jitters between the pads), lifts it, turns the grip
  half a turn while carrying it (through the middle, so the turn stays about the vertical), lowers the pins into the
  slots, lets go and parks on the right. Then the left arm, fingers closed, sweeps the lever's top along its arc
  toward the chip until it passes 45 degrees and snaps shut, and goes back. It reaches 1.0 and ends on
  success at step 397 (19.9 s at 20 Hz). A trial takes about 2 minutes 15 s on an L40S, including startup.
- The same demo without the half turn (`TURN = 0`) puts the chip in the wrong way round: its solid end sits on the
  boss, and the trial scores 0.25 even with the lever closed.
- Videos: `inspect-robots video ~/yam-eval/zif-socket-chip-demo/<log>.json` writes one MP4 per camera next to the
  frames.
- Offline checks: `python -m pytest -q test_zif_socket_chip.py`.

## Scoring

Heights are above the table top unless named otherwise. The socket's top is 1.2 cm above the board.

- **In the socket:** the chip's centre within 2.5 mm across and 3 mm along of the socket's middle, and both ends
  of its body (2 mm in from each end) less than 3.5 mm above the socket's top: its pins are down in the slots.
- **Seated:** in the socket, the notch within 20 degrees of pointing at the lever end, and both ends of the body
  within 0.5 mm of the socket's top.
- **Lever closed:** turned 85 degrees or more from open (within 5 degrees of flat).
- The scorer remembers whether the chip was seated at the moment the lever closed. Reopening the lever forgets it.
- **0** if the chip has fallen off the table (centre more than 2 cm below the top) or the state is not finite.
- **1.0 (success, held 1 s):** the chip seated, the lever closed, and the chip was seated when the lever closed,
  the chip and the lever still.
- **0.5:** the chip seated, the lever open.
- **0.25:** the chip in the socket the wrong way round (notch within 20 degrees of pointing away from the lever),
  lever open or closed, or the lever closed while the chip was in the socket but not seated (a chip pushed down
  after the lever closed stays at 0.25, as a real closed ZIF socket doesn't take it).
- **0** otherwise, including doing nothing, closing the lever on an empty socket, and a chip held over the socket.

In a real trial, a photo from above shows the notch's side and the lever, and a feeler gauge or a ruler checks that
the chip sits flat on the socket.
