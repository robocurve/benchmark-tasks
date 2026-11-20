# Pump a soft volleyball to 0.3 bar: wet needle in square, held straight while the other hand pumps

Instruction: `Pump the ball up to 0.3 bar on the pump's gauge, then take the pump off and set it down.`

A bench. An indoor volleyball is strapped down in a low ring stand, soft: 0.2 bar, under the 0.30-0.325 kg/cm2 (0.294-0.319
bar) the volleyball rules ask for. Its valve faces the robot's left. In front of it lie a hand ball pump with a dial
gauge and its inflation needle screwed on, and a wet sponge. Pumping up a ball before training or a match is
something every player, parent and coach does, and it takes two hands.

The instruction gives only the target and the ending. The robot has to work out the rest:

- **Wet the needle.** Balls are printed "moisten needle before inflation": a dry needle drags on the valve's rubber and
  wears or tears it. The sponge is there for that: dip the needle in it before it goes into the ball.
- **In square.** The needle goes in only through the valve, straight along its axis, until the pump's head sits on
  the ball. Pushed in beside it or at a slant it bends or punctures the valve.
- **Held straight, two hands.** One hand holds the pump, needle in the valve, while the other strokes the T-handle.
  The air pushes back on the handle, up to 12 N near 0.3 bar, so every stroke pushes and pulls on the pump; the
  holding hand keeps the needle in and in line, because levered sideways it bends and rips the valve. Each full
  stroke adds about 0.0075 bar, so it takes some fifteen strokes.
- **To the reading.** The pump's gauge shows the line pressure while the needle is in: stop at 0.3 bar (the green band
  is 0.29-0.33). Over-pumping is wrong too.
- **Off and down.** Draw the needle straight out, lay the pump on the bench and let go.

## Scene

Positions in cm from the bench point below the midpoint between the arm bases (x forward, y left), heights above the
bench.

| object | type | size | material (real part) | position |
|---|---|---|---|---|
| ball | fixed (strapped into its stand) | 21 across (65-67 cm round) | indoor volleyball, white with blue and yellow panels; valve: black dot in a grey ring | centre x 27, y -17.5, 10.7 high; valve facing +y, 10.7 high |
| ring stand | fixed | 13 across, 2 high | orange plastic ball stand, clamped to the bench | under the ball |
| strap | on the ball | 2.5 wide | black hook-and-loop strap, over the ball's top and under the stand, across the ball (clear of the valve) | around the ball in the plane x-z through its centre |
| pump | jointed: barrel -weld- head, barrel -slide- rod -weld- T-handle | barrel 4 across x 18, head 3.2 x 2, rod 1 x 16, T-handle 7 x 2.5 x 2.5; 22 mm bore, 12 cm stroke (45 ml); 200 g | blue plastic single-action hand ball pump with a 5 cm dial gauge (0-0.6 bar, green band 0.29-0.33) on the barrel's top 11 cm behind the head | lying along y at x 12, needle end at y -5.5 pointing -y, handle all the way in |
| inflation needle | on the pump's head | 3.5 long, 2 mm | steel ball needle | screwed into the head |
| sponge | fixed | 7 x 5 x 2.5 | wet yellow cellulose kitchen sponge, taped to the bench with double-sided tape | x 12, y -11 |

Sizes: FIVB Official Volleyball Rules, rule 3.1: circumference 65-67 cm, 260-280 g, inside pressure 0.30-0.325
kg/cm2 (294.3-318.82 mbar, 4.26-4.61 psi). Ball volume 4.85 L. Hand ball pumps are about 30 cm long with a 2-3 cm
bore; the standard inflation needle is about 3.5 cm.

Real pump and gauge: an assembly. A small hand ball pump with a threaded nozzle; an inline ball pressure-gauge adaptor
screwed onto the nozzle; the inflation needle threaded into the adaptor's far end. The needle then sits directly on the
pump's head (no hose) with the gauge between them, as in the scene. Example adaptor: Summit Global ball pressure gauge
adaptor + needle, 2 pack (https://www.bunnings.com.au/2pk-summit-global-ball-pressure-gauge-adaptor-needle_p0813114);
alternative: the Net World Sports ball pressure gauge reader, which also screws onto a pump's nozzle
(https://www.networldsports.co.uk/ball-pressure-gauge-reader.html). A different bore or stroke changes only how many
strokes it takes. Before trials, tape a green band on the dial where it reads 0.29-0.33 bar: pump a ball and read the
ball with a Fox 40 Digital Pressure Gauge (WDPG: a needle ball gauge, 0-15 psi, reads in psi, bar, kg/cm2 and kPa;
https://www.ump-attire.com/Products/WDPG/Fox-40-Digital-Pressure-Gauge). The trial is scored by the Fox 40 reading of
the ball, not by the pump's dial.

Force check (before trials), against the simulation's values:

- Needle: with the ball (strapped in its stand) on a kitchen scale, push the wetted needle slowly through the valve
  (about 1 cm/s) and read the peak change. Simulation: about 0.6 N at 1 cm/s, 1.2 N at 2 cm/s (dry: twice that);
  sideways the valve holds the needle with 8 N per cm.
- Handle: with the needle in a ball at 0.3 bar, push the handle down onto a kitchen scale and read the force as the
  air starts to pass. Simulation: 11.4 N at 0.30 bar (the gauge pressure times the 22 mm bore's 3.8 cm2), 12.2 N at
  0.32 bar, plus a light seal drag.
- If the real pump's handle force differs, a pump with that bore is the closer match; the stroke count changes with
  the bore, not the task.

Reach: the pump is picked up 12 cm in front of the bases, the valve is 27 cm out and 11 cm up, the handle is stroked
between y 16 and 28 at that height.

## Reset

Let the ball down to 0.2 bar (check with the Fox 40 gauge), set it in the stand, valve to the pump side, and strap it down. Push the
pump's handle all the way in and lay it at its marks; re-wet the sponge where it is taped.

## Scoring (real trial)

From the video, the needle and the ball afterwards (Fox 40 gauge).

Score 0 if the needle went into the ball other than through the valve, square (its line more than 3 mm from the
valve's centre or more than 12 deg off its axis), or was levered while in (more than 12 deg, or its line moved more
than 15 mm off the valve centre); in a real trial these are judged by
the end state: a bent needle (laid on a straight edge afterwards) or a valve that leaks (the ball loses 0.03 bar or
more overnight). Score 0 also unless, for the
last 2 s, the pump lies on the bench with its needle out of the ball and both grippers are open (2 cm or more), 3 cm or
more from the pump and the ball, and still.

Otherwise, by the ball's pressure (gauge):

- 0: pumped up less than 0.03 bar, or over 0.5 bar;
- 1.0: 0.29-0.33 bar, and the needle was wetted (its tip held in the sponge for 0.3 s or more) before it first went in;
- 0.5: 0.29-0.33 bar with a dry needle, or 0.25-0.4 bar;
- 0.25: otherwise (pumped, but under 0.25 or 0.4-0.5 bar).

Every outcome is covered: doing nothing scores 0; pumping with a dry needle scores 0.5; stopping early or going on
past the band scores 0.5 or 0.25; a few strokes only, 0.25; levering the needle while it is in, or pushing it in beside
the valve, 0; leaving the needle in, or still holding the pump, 0.

## Simulation notes

The ball is fixed, as the strap and the clamped stand hold the real one (here the needle exerts no force on it). Its air is a model,
isothermal: the barrel's chamber fills from the room as the handle comes out, and as it goes in the chamber's air is
compressed and, once above the ball's pressure and with the needle 12 mm or more into the valve, shared with the
ball (4.85 L) through the needle; 4 ml of dead volume (barrel end, needle, gauge) stays in the pump. The air pushes the
handle back through the slide's drive at full strength (its gauge pressure times the bore's area: 12 N at 0.32 bar),
plus a light seal drag; let go, the handle stays where it is (the cup seal's friction). The needle is drawn on the
head and does not collide; the head does, and stops on the ball. While the needle is in, a joint between the valve and the pump's head
stands for the valve's rubber: a light sideways spring (800 N/m: 8 N per cm off the valve's line) and, while the
needle slides through the valve (before the head seats on the ball), a drag along the axis (60 N s/m with the needle
wetted, 1.2 N at 2 cm/s; twice that dry). Both are estimates. Whether the needle goes in through the valve,
square, and whether it is levered while in are decided by the rules above from the pump's pose. The gauge's pointer
is driven by the model: the line pressure while the needle is in, the chamber's otherwise. Uncalibrated.

## Requirements

yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 (the official YAM launchable). Everything is in `task.py`.
