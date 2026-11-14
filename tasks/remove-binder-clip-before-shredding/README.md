# Remove the binder clip before shredding

Instruction: "Shred these old invoices."

Three folded invoices lie broad-face down on a desk, held by a full-size binder clip. The clip raises one edge only a few millimetres; there is no paper rack. A personal shredder and a dish for clips stand beside them. The robot must acquire the thin stack, remove the sprung clip, orient the paper and feed it into the slot. Feeding the metal clip jams the shredder.

## Run and score

Run from this folder with the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Shred these old invoices." --scorer task_score \
  --max-steps 1200 --store-frames --log-dir runs/demo
```

For Dream-1 use `--policy dream`, `-E cam_width=320 -E cam_height=176`, and `--log-dir runs/dream`.

The command reports `task_score`. `task.json` defines the complete numeric scoring. Failures are latched until reset. Full credit requires one continuous second of the completed, released state; the same condition controls termination. Partial credit is 0.3 for removing the clip while retaining the paper over the desk or in the operating shredder. Paper or clip falling below the desk invalidates the trial. The release check includes the grippers' actual collision hulls, including open fingers.

The scripted demonstration lifts the stack by the clip's body, turns it upright and supports the paper with the other arm. It then changes its grip to squeeze the clip's handles, removes and puts down the clip, and feeds the paper edge-first. All motion is produced through the embodiment's normal arm joint actions. `flat_demo.py` supplies Cartesian wrist orientations to the stock waypoint player; no object is attached or repositioned by the demonstration.

`TASK_DEMO` selects diagnostic plans: `nothing`, `pickup`, `cliponly`, or `failure` (feeding with the clip attached). `TASK_SCORE_DIR` enables per-step scoring measurements; it does not affect the controller or score.

## Simulation fidelity

The three folded sheets are one rigid body. The real clip bends the paper locally; here the stack lies at the small slope imposed by the clip and desk contacts. The clip's free articulation has a sprung sliding jaw with a 5 N force limit. The handles' squeeze is detected geometrically and opens the jaw; friction holds the paper, with no attachment joint between clip and paper.

The full-size shredder head retains its slot dimensions. The collection bin is shortened so the slot is 12 cm above the desk. This changes waste capacity and working height, but not the spring-clip manipulation, paper dimensions or insertion tolerance. All dimensions and materials are in `task.json`.

The powered feed is approximated by a velocity drive at 4 cm/s after an aligned paper edge reaches the sensor 1.5 cm into the slot. The drive squares the stack to the throat over 0.25 s and cancels its weight while feeding; it never jumps the paper or helps acquire it. A clip entering the opening stops the feed permanently and lights the jam indicator. The paper settles below the throat when fully consumed and strips become visible through the bin window. The cuts and individual shreds are not simulated.

For a real trial use a real shredder with an operator at its power switch; stop if metal or fingers approach the cutters and score zero. Replace the paper and reattach the clip between trials.

## Runtime and seeds

The scripted demonstration reaches 1.0 at 709 steps (35.45 seconds simulated). Its exclusive GPU run at 640 x 360 took 209.62 seconds including startup; its slowest 30 steps took 9.40 seconds. Dream-1 reached 0.0 after 1,200 steps (60 seconds simulated), in one continuous session, taking 344.02 seconds including startup. The run limit is 1,200 steps. `object_jitter=0` fixes the layout; the scripted controller is deterministic and needs no random seed.

## Files and assets

`task.py` defines the scene, clip and shredder behavior, and scoring. `flat_demo.py` defines physical arm motions. `shred_usd.py` generates meshes and colliders into `build/`; `scenekit.py` and `usdkit.py` supply task-local USD helpers. `steptime.py` measures step runtime.

All meshes are built from boxes, cylinders, prisms and tubes. The textures are drawn by `make_textures.py`. The generic invoice uses DejaVu Sans under the [DejaVu Fonts License / Bitstream Vera Fonts License](https://dejavu-fonts.github.io/License.html); DejaVu changes are public domain. Only the rendered image is included, not the font file. No other third-party assets are used. The paper is A4 folded into thirds (210 by 99 mm), the binder clip is 41 mm wide, and the shredder's 230 mm feed length follows personal 10-sheet machines.
