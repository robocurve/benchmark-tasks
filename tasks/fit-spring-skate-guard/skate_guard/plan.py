"""Hold the toe cup while fitting the spring-loaded heel, then pack the skate."""

import numpy as np

from . import config as C
from .motion import Program, rotation

DOWN = rotation("X", np.pi)
TOE_GRASP = rotation("Z", np.deg2rad(-23.)) @ rotation("Y", np.deg2rad(-33.)) @ DOWN
HEEL_GRASP = rotation("X", np.deg2rad(16.)) @ rotation("Y", np.deg2rad(-33.)) @ DOWN
PACK_TOE_GRASP = rotation("X", np.deg2rad(-55.)) @ rotation("Y", np.deg2rad(-14.)) @ DOWN
PACK_HEEL_GRASP = rotation("X", np.deg2rad(55.)) @ rotation("Y", np.deg2rad(-14.)) @ DOWN
EXTENDED = .038


def guard_targets(origin, *, extension=0., opening=.09):
    return {
        "left": (origin + C.INVERTED @ C.GUARD_TOE_GRIP, TOE_GRASP, opening),
        "right": (origin + C.INVERTED @ (C.GUARD_HEEL_GRIP - [0, extension, 0]), HEEL_GRASP, opening),
    }


def boot_targets(origin, *, opening=.05):
    return {
        "left": (origin + C.INVERTED @ C.BOOT_TOE_GRIP, PACK_TOE_GRASP, opening),
        "right": (origin + C.INVERTED @ C.BOOT_HEEL_GRIP, PACK_HEEL_GRASP, opening),
    }


def seat_targets(origin, *, extension=EXTENDED, opening=.09):
    # Offsets keep the held guard level and square across the two pad faces.
    return {
        "left": guard_targets(origin + [-.005, 0, -.006], opening=opening)["left"],
        "right": guard_targets(origin + [.005, 0, .004], extension=extension,
                               opening=opening)["right"],
    }


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    boot = C.BOOT_START.copy()
    start = C.GUARD_START.copy()
    # The pads settle off-centre on the molded flank faces. Calibrate the
    # taught tool poses to put the channel centre on the runner when held.
    fit = boot + [.015, -.009, .002]
    clear_caps = fit + [0, .014, 0]
    if mode == "demo":
        p.add(2.4, "approach the two loose guard cups from above",
              **guard_targets(start + [0, 0, .183], opening=1.))
        p.add(1.4, "descend along the two guard flanks", **guard_targets(start, opening=.76))
        p.add(.9, "pinch the molded toe and heel grips", **guard_targets(start))
        p.add(1.55, "hold the supported toe while stretching the heel against both springs",
              left=guard_targets(start + [0, .012, 0])["left"],
              right=guard_targets(start, extension=.046)["right"])
        p.add(1.4, "lift the extended guard from its work blocks",
              left=guard_targets(start + [0, .012, .155])["left"],
              right=guard_targets(start + [0, 0, .155], extension=.046)["right"])
        p.add(1.60, "align the open end roofs outside the runner tips",
              **seat_targets(clear_caps + [0, 0, .037]))
        p.add(1.30, "lower the open guard channel over the runner", **seat_targets(clear_caps))
        p.add(.65, "seat the toe under its retaining roof",
              left=seat_targets(fit)["left"], right=seat_targets(clear_caps)["right"])
        p.add(.95, "release the heel so the cups capture the blade ends",
              left=seat_targets(fit)["left"], right=seat_targets(clear_caps, opening=.86)["right"])
        p.add(.75, "let go of the fitted guard", **seat_targets(fit, opening=.98))
        p.add(1.25, "lift away from the fitted guard", **seat_targets(fit + [0, 0, .090], opening=1.))
    pickup = boot + [.015, -.006, 0] if mode == "demo" else boot
    p.add(1.85 if mode == "bare" else 1.0, "approach the boot toe and heel for packing",
          **boot_targets(pickup + [0, 0, .142], opening=1.))
    p.add(1.60, "descend alongside the boot shell", **boot_targets(pickup, opening=1.))
    p.add(.8, "grasp the structural toe box and heel", **boot_targets(pickup))
    p.add(.90, "lift the skate clear of the padded work blocks", **boot_targets(pickup + [0, 0, .072]))
    p.add(2.2, "carry the skate over the opened bag flap", **boot_targets(boot + [C.BAG[0] - boot[0], 0, .072]))
    packed = C.BAG + [0, 0, C.BAG_FLOOR + .158]
    p.add(1.35, "lower the skate over the empty gear bag", **boot_targets(packed))
    p.add(.86, "let go of the skate", **boot_targets(packed, opening=.99))
    p.add(1.38, "withdraw both hands from the packed compartment",
          **boot_targets(packed + [0, 0, .127], opening=1.))
    p.add(.86, "retract behind the bag rim", **boot_targets(packed + [-.130, 0, .122], opening=1.))
    return p.moves
