# Scoring

Instruction: **Put the knife into the holder.**

| End state | Score |
|---|---:|
| The loose knife is not inserted and released in the center slot | 0 |
| The knife is inserted; the hamster toy is still in the holder | 0.5 |
| The knife is inserted; the hamster toy rests outside the holder on the table | 1 |

The competition's
[review criteria](https://github.com/robocurve/benchmark-tasks/blob/main/AGENTS.md)
call for numeric conditions, useful partial credit and complete outcome coverage;
they do not specify a required number of score increments.

Inserted means the blade tip is 140–157 mm below the slot deck, the blade is within
10 degrees of upright and its broad face is aligned with the slot within 10 degrees.
Its axis at the slot deck is within 2 mm across the narrow direction and 5 mm along
the slot. The latter follows the 46 mm opening around the blade's 36 mm section;
a released blade may rest against an edge. The knife moves at most 10 mm/s and spins at most 0.5 rad/s; both robot TCPs
are at least 80 mm from its handle center. This excludes a knife still being held.

Removed means the entire toy collision envelope clears the holder footprint by
at least 5 mm and rests on the tabletop: its lowest collider point is within
2 mm below to 6 mm above the table, linear speed at most 10 mm/s, angular speed at
most 0.5 rad/s, and both TCPs at least 80 mm away. The entire toy must remain at
least 10 mm inside the table edges. It is a rigid hamster figurine, not a live animal.

A qualifying score must hold for one continuous second (20 control steps). The
reported score is the score at the end of the trial, not an earlier maximum.
Toy removal alone earns zero. A valid inserted knife with the toy not yet safely
placed earns at most 0.5; a fallen/off-table toy or invalid/penetrating state earns
zero. Wrong-slot, shallow, unstable or unreleased insertion earns zero.

Implementation: [scoring.py](scoring.py).
