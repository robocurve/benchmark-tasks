# Install a processor: lift the socket lever, open the shield, seat it on its keys, shut both

A server board lies on the table with an empty, shut processor socket, and a processor lies on a stand beside it,
turned a quarter turn. The robot opens the socket the way a server technician does: it pulls the socket lever up out
from under its catch and lifts it all the way, swings the steel processor shield open by its lip, sets the processor in
the socket with its two notches on the socket's keys, shuts the shield and presses the lever back down under its catch.
The steps are from Dell's PowerEdge R260 service video (an LGA1700 server): "To install the processor, align and place
the processor in the socket", then "Lower the processor shield, then lower the lever and push it under the tab to lock it
into place."

The hard part is physical. The socket is a linkage that has to be worked in the order its parts allow: the lever lies
over the shield, so the shield cannot open until the lever is lifted, and the lever cannot go back down until the
shield is shut. The shield and lever swing about hinges and have to be carried along their arcs, the lever has to be
pulled out from under a sprung catch and later pressed back under it, and the 45 by 37.5 mm processor has to go down
square into a pocket with 0.25 mm of play a side, turned the right way so its notches meet the keys.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. The standard `yam_arena` embodiment runs it: importing
the task file installs the socket (`mechanism.py`) and the processor (`parts.py`) into the scene, so nothing needs
installing.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=install_processor.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the processor in the empty socket. Pull the socket lever's handle up out from under its catch and lift the lever all the way open, then open the steel processor shield by its lip. Set the processor in the socket with its two notches on the socket's keys and its gold triangle at the socket's gold mark. Then shut the shield and press the lever back down under its catch." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/install-processor-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=install_processor.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the processor in the empty socket. Pull the socket lever's handle up out from under its catch and lift the lever all the way open, then open the steel processor shield by its lip. Set the processor in the socket with its two notches on the socket's keys and its gold triangle at the socket's gold mark. Then shut the shield and press the lever back down under its catch." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/install-processor-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/install-processor-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the processor is seated with the shield and lever
shut and everything has been still for 1 s; otherwise it scores the last step.

## Scoring
The processor is **seated** when its lowest point is within 0.5 mm of the board, it is flat within 1 degree, its centre
is within 0.8 mm of the pocket's, it is turned within 2 degrees of the socket's orientation and it is still. The
shield is **shut** within 3 degrees; the lever is **down** within 2 degrees, which it only reaches under the catch
(laid on top of the catch it sits at about 4 degrees).

| State | Score |
|---|---|
| Processor not seated: on its stand, on the shut shield, riding on the keys turned the wrong way, tilted, off-centre | 0 |
| Seated, shield open | 0.4 |
| Seated, shield shut, lever up | 0.7 |
| Seated, shield shut, lever down under its catch, held 1 s | 1.0 |

Possible scores: 0, 0.4, 0.7, 1.0. The start (socket shut and empty) scores 0, and nothing earns credit until the
processor is in. On a real board: a top photo for the lid square in the frame and the lever under its tab, and a side
view for the processor flat.

## Design notes
- **Where the mechanism comes from.** `mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request
  #223 (MIT License; the notice is at the top of the file). It builds a fixed base with hinged and sliding links as an
  Isaac Lab articulation and reports each joint's position to `score()`. `parts.py` (ours) adds a loose part made of
  several boxes, here the processor with two notches in its edge, and hooks into the scene the same way.
- **How precise it has to be.** The pocket leaves the processor 0.25 mm a side, and the keys stop it turned the wrong
  way. Set down 1 mm off centre either way across the socket, it did not seat (0; official sim, 2026-11-06).
- **The interlock.** Opened with the lever still down, the shield moved 0.3 degrees and stopped against the lever
  (official sim, 2026-11-05), so the order is forced: lever, shield, processor, shield, lever.
- **The lead-ins.** The frame's inner top edges and the keys are rounded (1 mm), as a real socket's plastic guides are.
  With square edges, a processor turned 0.7 to 1.4 degrees in the grip jammed at 4.3 degrees of tilt on the way in.
- **The catch.** A real lever's wire flexes sideways out from under a fixed tab. Here the lever is rigid and the catch
  is a sprung pin that moves aside, so the lever snaps under it when pressed down and pulls out when lifted firmly. Under
  the catch the lever sits at 0 to 0.7 degrees; laid on top of it, at about 4, which scores as up.
- **Why the processor starts turned.** On its stand it lies a quarter turn from the socket with its notches facing away
  from the arms, so placing it means turning it the right way: turned the other way it rides on the keys and scores 0.
- **The processor's size** is LGA1700's 37.5 by 45 mm (Wikipedia's LGA 1700 article). The lid size, thicknesses and
  the 30 g mass are approximate.

## Runtime and seeds
- Per trial on one L40S: the scripted demo reaches 1.0 at step 1,051 (52.5 s of simulated time, about 5 min of wall
  clock with stored frames); Dream-1 uses all 3,000 steps (about 11 min) and scored 0.0: it reached toward the socket
  with both arms but never seated the processor (official runs, 2026-11-06).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_install_processor.py` checks the scoring without a GPU (`python -m pytest -q test_install_processor.py`): the
  start, every credit level, the seat, shield and lever limits either side, a lever laid on the catch, a processor
  turned the wrong way, on the shut shield, tilted, off-centre or moving, the keys against the notches, the interlock,
  the catch's clearances, and the demo's reach. A mutation check (14 deliberate breaks of the scorer and layout, each
  run against the tests) catches 14 of 14.
