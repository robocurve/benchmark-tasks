# Real-world parts for the X-ray sensor holder task

The task is a bite block, an indicator arm with two prongs, and an aiming ring, as in an intraoral sensor positioner. Two ways to get a repeatable real set are given. Status: neither has been built or measured by us; the printable files were generated from the simulation's dimensions and checked as watertight solids.

## Option A: an obtainable kit

Dentsply Rinn XCP-DS FIT sensor-holder kits are sold by dental dealers (for example the "XCP-DS FIT complete kit" listed by Benco, item 4330-029, and the Dentsply Rinn 559909 and 559908 kits). Their contents vary by SKU: an arm and ring, bite blocks (anterior, posterior, bitewing, endodontic) and sensor covers. Check the exact SKU's instructions before ordering.

The simulation's numbers are design values taken from a typical holder, not measurements of a Rinn kit. Before using a kit, measure it and replace the values in the task:

| Measure | Tool | Simulation's value |
|---|---|---|
| Prong diameter, prong spacing, prong length | Calipers | 3.0 mm, 8 mm apart, 18 mm |
| Bite-block hole diameter | Pin gauges | 3.2 mm (the real fit is a light press fit; see below) |
| Force to push the prongs fully in, and to pull them out | Push or fish scale | not modelled (rigid bodies) |
| Ring slot against the arm's upright | Feeler gauge | 0.5 mm play each way |
| Sensor size | Calipers | 30 x 40 x 6 mm |

## Option B: a printable equivalent with a defined press fit

| File | What it is |
|---|---|
| `make_parts.py` | Generator (needs `trimesh`, `numpy`, `manifold3d`); `python make_parts.py [hole_mm]` |
| `bite_block.stl` | Paddle 35 x 45 x 4 mm, two side rails 9 mm high with 2 mm lips, bite plate 4 x 20 x 45 mm, connector block 12 x 20 x 20 mm with two vertical holes 8 mm apart and an 8 x 16 mm lead-in over the top 3 mm |
| `aiming_ring.stl` | Ring 46 mm inside, 60 mm outside, 5 mm thick, with a 22 mm sleeve, a 3.5 x 6.0 mm slot 48 mm from the ring centre and a 12 mm tall finger tab out to 90 mm |
| `grip_hub.stl` | Plastic hub 10 x 14 x 20 mm with a 2.7 x 5.2 mm slot for the upright |

Bill of materials besides the printed parts:

| Item | Spec |
|---|---|
| Prongs | Two hardened steel dowel pins, 3 mm diameter, 18 mm exposed length (ISO 2338, h8) |
| Arm | Stainless flat bar 5 x 2.5 mm: a bridge over the prongs, a 31 mm crossbar and an 85 mm upright; the pins are pressed or soldered into the bridge |
| Printing | PETG, 0.15 mm layers, 4 perimeters, 100% infill for the connector block |

The default prong hole is **2.95 mm for the 3.0 mm pins**, a light press fit (0.025 mm interference a side), which is what makes the insertion take work in a real trial. Set and check it:

| Check | How | Target |
|---|---|---|
| Insertion force | Push the pins into the connector block with a push scale | 5 to 15 N to seat fully |
| Retention | Hold the block and pull the arm with the scale | At least 3 N before it comes out |
| Too tight (over 20 N) | Ream the holes with a 3.0 mm drill | |
| Too loose (under 2 N) | Reprint with `python make_parts.py 2.90` | |

Prongs seated fully means the arm's bridge rests on the connector block's top.

## What the simulation models

Rigid bodies cannot overlap, so the simulation cannot hold a 2.95 mm hole round a 3.0 mm pin. It uses a 3.2 mm hole (0.1 mm clearance a side) with the same lead-in, a close sliding fit, and does not model the press-fit force. The press-fit work therefore exists in the real trial (measured with the scale as above) but only as a tight fit, with the arm needing alignment to about a tenth of a millimetre, in the simulation.
