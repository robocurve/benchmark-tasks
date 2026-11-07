"""Deterministic geometry, in metres; these are engineering values, not official rules.

This module and its occupancy are private environment configuration. Policy gets
only the public instruction, unchanged standard YAM proprioception and RGB.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

TABLE_TOP = 0.7475
PARKING_REGION_SIZE = 0.150
MODULE_SIZE = (0.040, 0.040, 0.040)
MODULE_MASS = 0.080
CONTROL_HZ = 20
RUN_LIMIT = 2400
BASELINE_SHA = "0059d02a1a91bc6834c1d0846022891df0c78730"

INSTRUCTIONS = {
    "sliding": "Return the module from the intake pad to a free matching cradle inside the cabinet. Keep every stored module in its own cradle. Do not squeeze modules together, stack them, or force the cabinet.",
    "pullout": "Return the module from the intake pad to a free matching cradle in the drawer unit. Keep every stored module in its own cradle. Do not stack modules, remove stored modules, or force a drawer.",
    "covered": "Three opaque cases stand side by side in one row. Use the left arm to grasp each lid's central handle, lift it and slide it horizontally to the left of that case, clear of the right wrist camera's view. The right arm holds the incoming module and uses its wrist camera to check each opened case. If occupied, move the right arm away and replace that lid before checking another case. Case inspection order is unrestricted. Place the module only in a visually confirmed empty case, move the right arm clear, and replace its lid. Keep stored modules undisturbed and leave all three lids seated on their matching cases.",
}
TITLES = {
    "sliding": "Sliding-Cabinet Instrument Return",
    "pullout": "Pull-Out Instrument Return",
    "covered": "Covered-Case Instrument Return",
}


@dataclass(frozen=True)
class Part:
    name: str
    position: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float] = (0.35, 0.40, 0.44)


@dataclass(frozen=True)
class Mechanism:
    name: str
    origin: tuple[float, float, float]
    parts: tuple[Part, ...]  # relative to body origin
    mass: float
    axis: str | None = None
    lower: float = 0.0
    upper: float = 0.0
    handle: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Design:
    kind: str
    empty: int
    fixed: tuple[Part, ...]
    mechanisms: tuple[Mechanism, ...]
    cradle_origins: tuple[tuple[float, float, float], ...]
    intake: tuple[float, float, float]
    parking: tuple[tuple[float, float, float], ...]
    interior_min: tuple[float, float, float] = (-0.025, -0.025, 0.0)
    interior_max: tuple[float, float, float] = (0.025, 0.025, 0.044)

    @property
    def occupancy(self):
        return {f"cradle_{i}": None if i == self.empty else f"stored_{i}"
                for i in range(len(self.cradle_origins))}

    @property
    def instruction(self):
        return INSTRUCTIONS[self.kind]


def cradle_parts(prefix, origin=(0.0, 0.0, 0.0), *, front_open=False):
    """50mm square interior: one 40mm module, 16mm side walls, physical floor."""
    x, y, z = origin
    parts = [Part(prefix+"_floor", (x, y, z-0.003), (0.062, 0.062, 0.006), (0.16, 0.19, 0.20))]
    for side in (-1, 1):
        parts.append(Part(prefix+f"_side_{side:+d}".replace("+", "p").replace("-", "m"),
                          (x, y+side*0.028, z+0.008), (0.062, 0.006, 0.016)))
    parts.append(Part(prefix+"_back", (x+0.028, y, z+0.008), (0.006, 0.050, 0.016)))
    if not front_open:
        parts.append(Part(prefix+"_front", (x-0.028, y, z+0.008), (0.006, 0.050, 0.016)))
    return parts


def make_design(kind: str, empty: int | None = None) -> Design:
    if kind not in TITLES:
        raise ValueError(f"Unknown task: {kind}")
    count = 2 if kind == "sliding" else 3
    empty = count-1 if empty is None else empty
    if not isinstance(empty, int) or not 0 <= empty < count:
        raise ValueError("The core layout requires exactly one in-range empty cradle")
    fixed, mechanisms, origins, parking = [], [], [], []
    top = TABLE_TOP
    intake_height = 0.036 if kind == "sliding" else 0.006
    intake = (0.18, -0.08, top+intake_height+MODULE_SIZE[2]/2)
    fixed.append(Part("intake_pad", (intake[0], intake[1], top+intake_height/2),
                      (0.075, 0.075, intake_height), (0.18, 0.30, 0.32)))
    if kind == "covered":
        # +X is forward from the robot bases. Keep the three cases in the
        # shared middle workspace; 110 mm spacing leaves 4 mm between panels.
        case_xy = ((0.27, 0.11), (0.27, 0.00), (0.27, -0.11))
        # Bare-table holding/set-down coordinates. The visible grey parking
        # pads were redundant scene clutter.
        park_xy = ((-0.05, 0.16), (-0.05, 0.00), (-0.09, -0.15))
        # Place the incoming item almost level with the case row in top view.
        intake = (0.25, -0.33, top+intake_height+MODULE_SIZE[2]/2)
        fixed[0] = replace(fixed[0], position=(intake[0], intake[1], top+intake_height/2))
        for i, (x, y) in enumerate(case_xy):
            origin = (x, y, top+0.008)
            origins.append(origin)
            # One closed-bottom case, with no nested small cradle. The central
            # 50 mm scoring footprint is virtual; the physical bed is a single
            # 96 mm plate joined by overlapping fixed walls.
            shell = (0.28, 0.33, 0.37)
            fixed.append(Part(f"case_{i}_bottom", (x, y, top+0.005),
                              (0.096, 0.096, 0.006), shell))
            # Wall rim occupies 38..45 mm from centre. The two closed skirt
            # rings form one underside channel: 35.5..47.5 mm from centre,
            # leaving 2.5 mm installation clearance on each side of the rim.
            for sign in (-1, 1):
                fixed.append(Part(f"case_{i}_x_{sign}".replace("-", "m"),
                                  (x+sign*0.0415, y, top+0.0325), (0.007, 0.090, 0.055), shell))
                fixed.append(Part(f"case_{i}_y_{sign}".replace("-", "m"),
                                  (x, y+sign*0.0415, top+0.0325), (0.078, 0.007, 0.055), shell))
            body_origin = (x, y, top+0.062)
            parts = (Part("panel", (0., 0., 0.), (0.106, 0.106, 0.004), shell),
                     Part("skirt_xp", (0.0485, 0., -0.004), (0.002, 0.099, 0.008), shell),
                     Part("skirt_xm", (-0.0485, 0., -0.004), (0.002, 0.099, 0.008), shell),
                     Part("skirt_yp", (0., 0.0485, -0.004), (0.095, 0.002, 0.008), shell),
                     Part("skirt_ym", (0., -0.0485, -0.004), (0.095, 0.002, 0.008), shell),
                     Part("groove_inner_xp", (0.0345, 0., -0.004), (0.002, 0.071, 0.008), shell),
                     Part("groove_inner_xm", (-0.0345, 0., -0.004), (0.002, 0.071, 0.008), shell),
                     Part("groove_inner_yp", (0., 0.0345, -0.004), (0.067, 0.002, 0.008), shell),
                     Part("groove_inner_ym", (0., -0.0345, -0.004), (0.067, 0.002, 0.008), shell),
                     Part("handle", (0., 0., 0.014), (0.070, 0.022, 0.024), (0.08, 0.09, 0.10)))
            # PP density ~0.91 g/cm3 gives ~75 g for these solid cuboids.
            mechanisms.append(Mechanism(f"lid_{i}", body_origin, parts, 0.075, handle=(0., 0., 0.014)))
            # The skirt, not the panel, rests on the existing tabletop.
            # Preserve controller coordinates without adding pad geometry.
            px, py = park_xy[i]
            parking.append((px, py, top+0.008))
    elif kind == "pullout":
        for i, y in enumerate((0.08, 0.24, 0.40)):
            origin = (0.35, y, top+0.012)
            origins.append(origin)
            parts = cradle_parts("cradle")
            parts += [Part("tray", (0., 0., -0.006), (0.148, 0.136, 0.006)),
                      Part("front", (-0.074, 0., 0.057), (0.008, 0.150, 0.110)),
                      # Close the central under-front slit without entering the side rails.
                      Part("front_gap_filler", (-0.074, 0., -0.0005), (0.008, 0.120, 0.005)),
                      Part("handle", (-0.113, 0., 0.055), (0.070, 0.024, 0.034), (0.08, 0.09, 0.10))]
            mechanisms.append(Mechanism(f"drawer_{i}", origin, tuple(parts), 0.18,
                                        "X", -0.140, 0., (-0.123, 0., 0.060)))
            for sign in (-1, 1):
                fixed.append(Part(f"rail_{i}_{sign}".replace("-", "m"),
                                  (0.29, y+sign*0.072, top+0.006), (0.29, 0.008, 0.012)))
        fixed += [Part("cabinet_roof", (0.358, 0.24, top+0.124), (0.154, 0.48, 0.012)),
                  Part("cabinet_back", (0.434, 0.24, top+0.062), (0.008, 0.48, 0.124))]
        for y in (0., 0.16, 0.32, 0.48):
            fixed.append(Part(f"cabinet_side_{len(fixed)}", (0.352, y, top+0.062), (0.164, 0.008, 0.124)))
    else:
        for i, y in enumerate((0.30, 0.10)):
            origin = (0.335, y, top+0.036)
            origins.append(origin)
            fixed.extend(cradle_parts(f"cradle_{i}", origin, front_open=True))
            door_origin = (0.260, y, top+0.092)
            parts = (Part("panel", (0., 0., 0.), (0.010, 0.198, 0.176)),
                     Part("handle", (-0.033, 0., 0.0), (0.046, 0.024, 0.042), (0.08, 0.09, 0.10)))
            # Side parking avoids a rear door handle intersecting the front door.
            lower, upper = (0., 0.215) if i == 0 else (-0.215, 0.)
            mechanisms.append(Mechanism(f"door_{i}", door_origin, parts, 0.15,
                                        "Y", lower, upper, (-0.033, 0., 0.0)))
        fixed += [Part("cabinet_floor", (0.34, 0.20, top+0.015), (0.14, 0.412, 0.030)),
                  Part("cabinet_roof", (0.34, 0.20, top+0.188), (0.14, 0.412, 0.012)),
                  Part("cabinet_back", (0.414, 0.20, top+0.096), (0.008, 0.412, 0.192))]
        for y in (-0.006, 0.20, 0.406):
            fixed.append(Part(f"cabinet_wall_{len(fixed)}", (0.342, y, top+0.096), (0.144, 0.008, 0.192)))
        for z in (0.001, 0.183):
            fixed.append(Part(f"door_rail_{len(fixed)}", (0.260, 0.20, top+z), (0.020, 0.850, 0.004)))
        # Full-pose URDF sweeps show that the initial low cabinet forces the
        # wrist through the table. Move it forward and raise it on real solids.
        def shifted(position):
            return (position[0]+0.160, position[1], position[2]+0.060)
        fixed = [replace(p, position=shifted(p.position)) for p in fixed]
        # The first physical trial caught the intake cube during the right
        # finger's door sweep. This lower, forward pad clears both arm paths.
        fixed[0] = Part("intake_pad", (0.360, -0.040, top+0.033),
                        (0.075, 0.075, 0.066), (0.18, 0.30, 0.32))
        mechanisms = [replace(m, origin=shifted(m.origin)) for m in mechanisms]
        origins = [shifted(p) for p in origins]
        intake = (0.360, -0.040, top+0.086)
        fixed += [Part("cabinet_pedestal", (0.500, 0.20, top+0.030), (0.140, 0.412, 0.060)),
                  Part("rail_plinth", (0.420, 0.20, top+0.0295), (0.020, 0.850, 0.059))]
    return Design(kind, empty, tuple(fixed), tuple(mechanisms), tuple(origins), intake, tuple(parking))
