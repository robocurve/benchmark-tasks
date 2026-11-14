# Pull a jammed card out of a printer with both hands, then resume printing

Clearing a paper jam is one of the most familiar printer chores at home and in the office. The table top stands for
a desk. A portable inkjet printer stands with its back toward the robot, and a card (a DL
card, 210 x 99 mm, fed landscape) has jammed in the feed slot of its rear tray: it stands in the slot with its top 5 cm
sticking out, and the printer's two feed rollers, seen at the slot's mouth, still grip it near both edges. The OK
button is on the printer's top. The instruction asks for the jam to be cleared and printing resumed; it does not say
how.

The procedure the task checks is the printer maker's. Canon's online manual for its GX5000 series,
[What to Do When Paper Is Jammed][manual] ("Removing Jammed Paper through Feed Slot of Rear Tray"): "Hold jammed paper
firmly with both hands and pull it out slowly." "Slowly pull out the paper so as not to tear it." Then: "Press
printer's OK button. The printer resumes printing." With the rollers gripping both edges, a card pulled up by one
edge, or by its middle, skews in the slot and tears beside a roller; pulled up by both edges at once, evenly, it comes
out whole. So clearing the jam takes both arms pulling together, one at each edge, above the two rollers. The task is the task file
[`pull_a_jammed_card_out_with_both_hands.py`](pull_a_jammed_card_out_with_both_hands.py) for the `yam_arena`
embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres, upright, and cannot join them or put one on a spring.
The printer (its case round the feed slot), the card (three strips, each with its printed lines), the two feed rollers
on their arms and the OK button are bodies built from several shapes, and the rollers' arms, the button and the card's
seams are springs, so `bodykit.py` adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator
starts, without changing yam_arena, and gives `score()` each body's pose.

What is modelled and what is not: the printer is the size of a portable inkjet printer (30 x 18 x 7 cm; Canon's PIXMA
TR150 is 32 x 18.5 x 6.6 cm), and its rear-tray feed slot takes the 21 cm-wide card with 1 mm to spare at each end
(Canon's inkjets take custom sizes up to 21.6 cm wide through the rear tray). The card is 3 mm thick, thicker and
stiffer than real 300 g/m2 card (about 0.4 mm), so that a gripper can hold it in the simulation; it does not bend. It is
three strips joined along two seams, and a seam lets its strips slide past each other only up and down, on a spring of
1 N per mm. A seam sheared 1.5 mm counts as torn. Each feed roller is a roller on a spring arm that presses its edge of
the card against the slot's far wall with about 4 N, so drawing that edge out takes about 4.5 N; the rollers sit at the
slot's mouth, where they can be seen. The OK button is a key on a spring. The printer does not print: pressing OK after
the card is out stands for resuming printing.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. The card
has torn once one of its seams has sheared 1.5 mm or more: that sets the score to 0 for the rest of the run. Otherwise
the score is the best stage reached: 0.25 the card raised 1.5 cm or more; 0.5 out of the printer (its bottom edge
above the printer's top); 0.75 put down on the table in front of the printer (its lowest point within 1.2 cm of the
table, every strip's middle in front of the printer's back, at rest, no gripper holding it). Full score: that, with the
OK button pressed (3 mm or more) after the card came out, both grippers 6 cm or more from the card and from the
button, still for 1 s. Pressing OK while the card is still in the printer does not count. `-E object_jitter=0` keeps
every object exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pull_a_jammed_card_out_with_both_hands.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Clear the paper jam and resume printing." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/paper-dream
```

The scripted demo, which reaches full score (each gripper takes the card by its top edge above a roller, and together
they pull it straight up out of the slot, slowly; they bring it toward the robot, clear of the printer, lower it until
it nearly touches the table and let it go; the left arm moves away, and the right presses the OK button and moves
away):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pull_a_jammed_card_out_with_both_hands.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Clear the paper jam and resume printing." \
  --scorer task_score --max-steps 540 --store-frames --log-dir ~/yam-eval/paper-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/paper-dream/*.json; do inspect-robots video "$log"; done
```

`PAPER_DEBUG=1` prints, every 10 steps (`PAPER_DEBUG_EVERY=5` for every 5), how far each strip of the card has risen,
the card's lowest point, how far the seams have sheared, whether the card is out, held or put down, how far the OK
button is pushed and whether the press counted, how close the nearest gripper is, where each gripper is, the rollers'
angles and the stage reached.

`PAPER_ONE=1` runs the demo the wrong way: the right gripper alone pulls the card up by its right edge. The card skews
in the slot, the left roller keeps the left third where it is, and the card tears along its left seam: the middle and
right parts come up 6 cm while the left third stays in the slot. The run scores 0.

## Runtime per trial

On one RTX PRO 6000 (Blackwell): about 1.5 minutes for the scripted demo while rendering 1280 x 720 video (about 420
steps, 21 s of simulated time), and about 7 minutes for a Dream-1 trial of 2,400 steps. Isaac Sim's first start on a
new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A portable inkjet printer with a rear feed slot, on a desk, and a DL card (210 x 99 mm) of 300 g/m2 card stock, about
0.4 mm thick: the simulation's 3 mm rigid card made of three strips on sprung seams is a stand-in for it (see What is
modelled above). To set the jam, feed the card landscape into the rear tray's slot, start a print and switch the
printer off as the rollers draw it in, once its top stands 5 cm out of the slot; check that a light pull on one
corner does not free it, and set it the same way before every trial. Whether the card came out whole, and whether
both edges were pulled together, is judged from the card itself and from the overhead and wrist videos.

[manual]: https://ij.manual.canon/ij/webmanual/ErrorCode/GX5000%20series/EN/ERR/err_jam0100.html
