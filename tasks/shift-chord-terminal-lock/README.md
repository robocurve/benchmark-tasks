# Enter the code on the card at a reception terminal and leave it secure

## Instruction

Enter the code on the card and leave the terminal secure. Let go and keep both fingers at least 3 cm from the panel.

## Context

Receptionists operate computers (O*NET 43-4171.00 lists the work activity 'Operate computers or computerized equipment', https://www.onetonline.org/link/summary/43-4171.00); BLS: receptionists held about 947,500 US jobs in 2025 (https://www.bls.gov/ooh/office-and-administrative-support/receptionists.htm). Unattended screens pose security risks: 'Lock your device if you have to step away from the screen' (CISA Project Upskill Checklist, https://www.cisa.gov/resources-tools/resources/project-upskill-checklist). This instrumented terminal tests holding a sprung modifier while another hand presses, then securing via Ctrl+L (an instrumented shortcut, not a universal standard). 24 mm caps at 45 mm pitch, enlarged for grippers. Closest cards: TK-KWWUPX laptop typing; TK-F4NTMM two-hand start; TK-RGZ4JH date stamp. New: case cue, modifier overlap, exact submission, and security chord.

## Setup

Coordinates in cm: x forward, y left from the midpoint between arm bases, z above a hardwood desk (world z 0.7475 m). The layout is the same in every trial. All body yaw angles 0. Materials and fixed fixtures each use a 25 kg stand-in with friction 1.2 (static=dynamic): charcoal ABS deck 11 x 30 x 2.5, bottom origin (22,1.5,0); glossy black monitor origin (36.5,-2,0), foot 6 x 14 x 3 centred locally (0,0,1.5), post 3 x 3 x 5 centred (0,0,4), panel 2 x 28 x 15 centred (0,0,11); cream card 6.5 x 15 x 0.3 at (24,28,0). Eight plastic caps 2.4 x 2.4 x 1.4, centres z 3.7, tops z 4.4; undersides 5 mm above deck. Each cap is a negative-z prismatic plunger, 5 mm travel to hard stop, DOWN at 3 mm and UP below 1.5 mm, damping 5 N s/m, friction 1.2. Shift (19.5,14) and Ctrl (24,14): blue, 0.5 g each, 8 N/m, unpressed gravity sag about 0.61 mm. Other caps 2 g, 100 N/m, sag about 0.20 mm: A (19.5,-2), b (19.5,-6.5), 7 (19.5,-11), Enter (24,-2), L (24,-6.5), Erase (24,-11); Enter green, Erase red, letters grey. Nearest pitch 45 mm, cap gap 21 mm; modifier-to-character gap in y is 160 mm or more. Marker legends: upward arrow for Shift, C for Ctrl, return arrow for Enter, back arrow for Erase, A/b/7/L glyphs for the others; five-by-five pixels 3.8 mm pitch. Card glyphs A, b, 7 are 35 mm across, centres (24,32.5), (24,28), (24,23.5), top points away from the robot; no case annotation or extra symbol. Monitor face x 35.45; buffer cells y -9.5,-5, -0.5 at z 11, pixels 4.5 mm pitch; status lamp y 6.5,z 11, 2 cm square: green unsecured, red secured. All 352 marker instances are created once at reset and only moved, unused instances parked at world z=-10 m. Terminal accepts at most three text characters; further text is ignored until Erase removes one. A nonmodifier is counted at its UP, delayed until two steps after DOWN if needed. Shift/Ctrl count if physically DOWN at any sample within +/-2 steps of that nonmodifier DOWN (0.1 s at 20 Hz). Shift makes a letter uppercase; Shift+7 produces ampersand. Ctrl+L secures and freezes input; Ctrl+Erase permanently invalidates the trial; other Ctrl chords add no text. Enter without Ctrl submits; a wrong code or case permanently invalidates. Two nonmodifiers simultaneously DOWN are void, not faults. Grippers start closed at ready (10,+/-14,12). Presses use yaw 0, TCP z 3.9 cm: pad bottoms 2.6 cm, 15 mm beyond the DOWN cap height, against the stop.

## Scoring

Final state plus latched events. ACCEPTED: Enter submitted exactly Ab7. SECURED AFTER: Ctrl+L counted after ACCEPTED; early security freezes input. Permanent zero if Enter rejects a code, Ctrl+Erase counts, or a fixed body origin moves more than 5 mm. RELEASED: every key displacement below 1.5 mm, both grippers >=90% open and both TCPs >=3 cm from deck and cap boxes. STILL: every cap displacement changes <=1 mm over the last 20 samples (1 s at 20 Hz). Ladder: 1.0 ACCEPTED, SECURED AFTER, RELEASED, STILL, no fault. 0.75 ACCEPTED and SECURED AFTER with a finishing condition missing. 0.5 ACCEPTED without later security. 0.25 an unsubmitted buffer starts with Ab, or security occurred before acceptance. Else 0. After security all character events are ignored; release/stillness are still measured. Doing nothing scores 0. Keys held down cannot earn full credit. Possible scores: 0.0, 0.25, 0.5, 0.75, 1.0. Real trial: screen/lamp photo and video showing physical modifier overlaps, end release, and fixture reference marks.

## Run limit

1,200 steps (60 seconds at 20 Hz)

## Reset notes

Put deck, monitor and card at their listed origins, yaw 0. Return all eight caps to rest; clear buffer, acceptance, security, faults, pending events and history. Lamp green and screen empty; card shows Ab7. Grippers closed at ready. Reuse marker instances and park unused ones at z=-10 m.

## Fidelity

An instrumented eight-key terminal, not a full computer. Plunger displacement drives input; the screen, legends and card are moved marker pixels. Caps are 24 mm at 45 mm pitch, taller and wider-spaced than a desktop keyboard. The 5 mm stop absorbs excess press depth; light 8 N/m modifiers avoid gravity-triggering. Ctrl+L is a task-specific configured shortcut. Case is a glyph cue. Modifier sampling uses a +/-2-step window, and simultaneous character presses are void.

## Scores

Dream-1 run: 0.0. Scripted demo: 1.0 (modifier held while the code is entered, then locked with Ctrl+L). One arm left idle: 0.0.

## Execution

`bash run.sh settle`, `bash run.sh demo`, `bash run.sh ablation`, `bash run.sh dream`. Select the idle arm with `ABLATION_ARM=left` or `ABLATION_ARM=right`. Run unit tests with `python -m pytest tests`.
