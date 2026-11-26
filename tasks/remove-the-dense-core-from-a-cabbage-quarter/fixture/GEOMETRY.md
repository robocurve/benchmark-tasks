# Geometry (all dimensions in mm)

No holding fixture is used. These drawings describe the modelled trimmed cabbage half, the knife and the ruler; the knife
and board are named products. The cabbage geometry is an estimate of a typical trimmed strip, not a scan. The real strip is cut as the README describes.

## Cabbage half, trimmed (cabbage frame: x along the strip, root end at x = -70; z up from the board)

```
Top view
                 leaves (three stacked layers), 140 long, up to 76 wide
   root face    +--------------------------------------------------+
   x = -70      |  \                                               |
   y = +25 *----+   \  core outline, side +1                       |
           | core    * apex (x = -30, y = 0)                         |
   y = -25 *----+   /  core outline, side -1                       |
                |  /                                               |
                +--------------------------------------------------+
               x = -70                                          x = +70
```

- Leaf layers: three, each 19 mm thick, 85 g, at z = 0.5-19.5, 20.5-39.5 and 40.5-59.5. Each is fourteen 10 mm
  slices along x with half-width 38 * sqrt(max(0.12, 1 - (x/78)^2)). Near the core each slice leaves a gap of
  1 mm beyond the widest core slice it overlaps, so no leaf and core collide at the start.
- Core: 45 g, eight 5 mm slices from x = -70 to -30, z = 1 to 59, half-width 25 * (-30 - x) / 40 at each slice
  centre. Its outline (the scored boundary) is the two straight lines from (-70, +-25) to the apex (-30, 0). The
  first slice is 46.9 mm wide; the leaves start 10 mm in from the root face, so the core's base is exposed there.
- All four bodies share one origin (x = 0, y = 0, z = 0 at the board top). Fixed joints hold them together at the
  start and are restored at every reset.
- Cut grids (`cut_geometry.py`): on each outline side, points every 2 mm along it (1 to 45 mm from the root face)
  and every 2 mm up it (2 to 58 mm above the board): 24 x 29 = 696 points per side.

## Knife (knife frame: handle centre at the origin, blade toward +x, flat on the board at the start)

- Handle 116 x 30 x 30 plus a 12 x 38 x 34 guard at x = +58; together 128 long. Mass 140 g for the whole knife
  (estimate).
- Blade from x = +64 to +216: 125 mm straight section 21 wide, then a 27 mm tapered tip; 1.8 thick, its face 4 mm
  below the handle axis. The cutting edge is the blade's y = -10.5 side.

## Board and ruler

- Board 535.9 x 368.3 x 20, top 20 above the bench; two dark 18 x 340 x 3 foot strips underneath, 250 either side
  of centre.
- Ruler 200 x 12 x 2 with ticks every 10 mm (longer every 50 mm), near the board's left edge. It is only for photos.

`root-grasp.svg` draws the root-end grasp used by the demo: the gripper pads close on the exposed core base and
clear the first leaf slice by about 4 mm.
