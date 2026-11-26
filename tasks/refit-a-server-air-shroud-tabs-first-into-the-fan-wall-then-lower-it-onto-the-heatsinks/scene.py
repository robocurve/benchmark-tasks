"""The bodies of the 2U server air-shroud task, built from parts.py's figures:
- ``chassis``: the fixed (kinematic) EIA-310-D 2U server-bay fixture (430.0 x 88.1 x 400.0 mm) carrying the front
  staging deck, the 52.0 mm mid-bay divider, two Dynatron S2 LGA4677 heatsinks, four 4-module JEDEC MO-329 DDR5 RDIMM
  banks, the 83.0 mm slotted fan bulkhead, and five Arctic S8038-7K server fans.
- ``shroud``: the free one-piece 3.0 mm ABS air shroud (418.0 x 160.0 x 78.0 mm) with two recessed blue touchpoint
  bridges inside its 78.0 mm height envelope and two 10.0 mm front tabs, starting flat on the front staging deck.
"""

from __future__ import annotations

import mechkit as mk
import parts as P
from mechkit import body

MECHKIT_HELPER = True


def shroud_start_pos(top_z: float) -> tuple[float, float, float]:
    """Shroud frame origin (bottom-centre) at the start, resting on the front staging deck."""
    return (float(P.X_START), 0.0, float(top_z + P.WALL_T + P.DECK_H + P.OVER))


def build(top_z: float) -> None:
    fric = (P.SURFACE_FRICTION, P.SURFACE_FRICTION)
    c_pos, c_shapes = P.chassis_shapes(top_z)
    body(
        "chassis",
        c_shapes,
        pos=c_pos,
        kinematic=True,
        color=P.CHASSIS_GREY,
        friction=fric,
        contact_offset=P.CONTACT_OFFSET,
    )
    body(
        "shroud",
        P.shroud_shapes(),
        pos=shroud_start_pos(top_z),
        mass=P.SHROUD_MASS,
        color=P.SHROUD_ABS,
        friction=fric,
        center=P.SHROUD_COM,
        inertia=P.SHROUD_INERTIA,
        contact_offset=P.CONTACT_OFFSET,
        contacts=True,
    )
    mk.sense_contact("shroud", "chassis")
