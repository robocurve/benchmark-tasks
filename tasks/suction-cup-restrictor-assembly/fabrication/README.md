# Fabricating the custom cup and restrictor specimen

These are custom mechanical assembly parts, not Schmalz product CAD and not a
vacuum-qualified end effector. STL units are millimetres. The drawings use the
same localY axis as the simulator. Translate onto a print bed without scaling;
retain the part datums when mounting. Every delivered STL is one closed solid.

## Parts and manufacturing route

| Item | Quantity | Route |
|---|---:|---|
| cup-compliant-master.stl | 1 | Flexible polyurethane resin, nominal Shore A80, or elastomer manufactured from the master. The neck/head/root webs must be flexible, not rigid PLA. |
| restrictor.stl | 1 | Turn/machine POM to the profile or print rigid resin. Clear the0.8mm through bore with an appropriate drill; do not enlarge the8.8mm shaft or10.6mm rib. |
| plate-and-stand.stl | 1 | Rigid print, or machine a4mm plate with an18mm hole and1mm-deep20-to-18mm chamfer and mount on the dimensioned stand. |
| Fixed staging blocks | As PHYSICAL_SETUP.md | Cut rigid stock to the listed sizes and secure at the task.py coordinates. |

The master joins all four head sectors to the neck with four flexible root webs.
Each web is0.6mm radial by3mm tangential, spanning localY23.5..26.5mm, with2mm
clear length between neck and head. The original head retaining corners, lower
shoulder and neck bore dimensions are retained. These connections are required
in a real part; the simulation replaces their deformation with four passive
radial springs. Bellows deformation, nonlinear material response and sealing
remain outside the model. The master is a manufacturing counterpart, not an
assertion of measured force equivalence.

Use a flexible-resin or elastomer process that can resolve the0.6mm webs and
clear the gap around them. Orient/support the print so no support bonds a head
sector to the shoulder. Wash/cure to the material manufacturer's instructions.
A rigid positive printed from this file is a reference master, not the compliant
trial component. Do not use it as the scored cup.

## Critical dimensions and assembly checks

- Plate:110mm OD,4mm thickness;18mm actual through hole. Chamfer only the cup
  entry face to20mm over1mm depth. Deburr without removing the rear retaining edge.
- Cup:40mm bellows envelope,16.6mm neck OD,9.2mm neck bore;21mm head and15.6mm tip.
  Keep the lower shoulder atY21.4mm and head capture face atY26mm, leaving4.6mm
  capture span. Do not bridge this capture groove with support material.
- Core:8.8mm shaft,10.6mm rib,13mm cap,0.8mm open bore,25.5mm total length.
  The dimensioned profile records all shoulder locations. Sanding away the rib
  produces a different, non-retaining component.
- The printed master uses continuous rotational profiles instead of the
  simulator's internal convex-wedge seams. The core drawing/STL uses96 angular
  segments for manufacture; the simulator uses32 shaft/24 rib segments. These
  are disclosed surface approximations, not an altered evaluation scene.

Before a workshop trial, check that the four sectors compress enough to pass the
18mm bore and recover without permanent set, and that the separate core rib can
pass and be captured. With a caliper and simple force gauge, check approximately
0.18N/mm per-sector radial stiffness over the used travel; this is the simulated
target, not a measured resin property. Select/qualify material and print process
rather than claiming Shore hardness alone determines stiffness. Reject brittle,
bonded or permanently deformed roots. No load or seal rating follows from these
installation checks. Do not add a latch, glue or a hidden fixing to hold parts.

## Placement and reset

The plate hole centre is(0.24,0,0.89)m with normal along worldY. Chamfer faces
negativeY, toward the loose cup. The cup starts at(0.38,-0.09,0.90)m and core at
(0.38,0.08,0.90)m with identity rotations. Base and staging supports are fixed;
there is no world support attached to the loose cup or core. Use the full
PHYSICAL_SETUP.md/task.py dimensions. Inspect the head, rib and actual bores
before each reset. Grade with SCORING.md using views of both faces and a
three-second released hold. No50-micrometre measurement, airflow equipment or
required arm strategy is part of the task.
