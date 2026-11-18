"""USD bodies for the desk two-hole punch, its lever and head, the foam stand, the card and its two pre-scored discs."""

from . import geometry as g
from .usd_bodies import Body

STEEL = (0.08, 0.08, 0.09)
RED = (0.80, 0.08, 0.06)
YELLOW = (0.98, 0.82, 0.12)
FOAM = (0.45, 0.47, 0.50)
CO = 0.0005


def _plate_with_holes(body, x0, x1, y0, y1, z0, z1, holes, size, color):
    """A box x0..x1, y0..y1, z0..z1 (body frame) with square through-holes of side ``size`` at (x, y) ``holes``."""
    hx = sorted({x for x, _ in holes})
    hy = sorted({y for _, y in holes})
    xs = [x0] + [v for x in hx for v in (x - size / 2, x + size / 2)] + [x1]
    ys = [y0] + [v for y in hy for v in (y - size / 2, y + size / 2)] + [y1]
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            cx, cy = (xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2
            if any(abs(cx - x) < 1e-9 and abs(cy - y) < 1e-9 for x, y in holes):
                continue
            if xs[i + 1] - xs[i] <= 0 or ys[j + 1] - ys[j] <= 0:
                continue
            body.box((cx, cy, (z0 + z1) / 2), (xs[i + 1] - xs[i], ys[j + 1] - ys[j], z1 - z0), color)


def punch():
    """Kinematic punch frame, origin at the env origin (positions are station coordinates)."""
    b = Body("punch", kinematic=True, friction=(0.4, 0.3), contact_offset=CO)
    holes = [(g.PIN_X, -g.PIN_Y), (g.PIN_X, g.PIN_Y)]
    # base and bed, with die holes through to the table (the chip tray is the cavity underneath)
    _plate_with_holes(b, g.BED_X0, g.BED_X1, -0.11, 0.11, g.TABLE, g.BED - g.BED_STEP, holes, g.DIE, STEEL)
    # back stop across the throat, 8 mm tall
    b.box(((g.STOP_X + g.BED_X1) / 2, 0.0, g.BED + 0.004), (g.BED_X1 - g.STOP_X, 0.22, 0.008), STEEL)
    # housing over the throat, with guide holes for the punches
    _plate_with_holes(b, g.CEIL_X0, g.BED_X1, -0.11, 0.11, g.CEIL_Z0, g.CEIL_Z1, holes, 2 * g.PIN_R + 0.001, STEEL)
    # side cheeks joining housing and base, outside the card's width
    for s in (-1, 1):
        b.box((g.PIN_X + 0.01, s * 0.105, (g.BED + g.CEIL_Z0) / 2), (0.040, 0.010, g.CEIL_Z0 - g.BED), STEEL)
    # lever bracket (visual only above the pivot)
    b.box((g.LEVER_PIVOT[0] + 0.004, g.LEVER_PIVOT[1], (g.CEIL_Z1 + g.LEVER_PIVOT[2]) / 2),
          (0.008, 0.028, g.LEVER_PIVOT[2] - g.CEIL_Z1), STEEL, collide=False)
    # paper guide marks: a white centre line on the bed in front of the throat (visual)
    b.box(((g.BED_X0 + g.CEIL_X0) / 2, 0.0, g.BED + 0.0002), (g.CEIL_X0 - g.BED_X0, 0.002, 0.0004), (0.95, 0.95, 0.95),
          collide=False)
    return b.save()


def stand():
    b = Body("stand", kinematic=True, friction=(0.4, 0.3), contact_offset=CO)
    b.box(((g.STAND_X0 + g.STAND_X1) / 2, sum(g.STAND_Y) / 2, (g.TABLE + g.BED) / 2),
          (g.STAND_X1 - g.STAND_X0, g.STAND_Y[1] - g.STAND_Y[0], g.BED - g.TABLE), FOAM)
    return b.save()


def head():
    """Punch head: frame at the punch tips' midpoint (x PIN_X, y 0, tip height)."""
    b = Body("head", mass=g.HEAD_M, com=(0.0, 0.02, g.PIN_LEN + 0.003), inertia=(2e-4, 3e-5, 2e-4),
             friction=(0.4, 0.3), contact_offset=CO, solver_iters=(128, 32))
    bx0, bx1 = g.BAR_X[0] - g.PIN_X, g.BAR_X[1] - g.PIN_X
    b.box(((bx0 + bx1) / 2, sum(g.BAR_Y) / 2, g.PIN_LEN + 0.003), (bx1 - bx0, g.BAR_Y[1] - g.BAR_Y[0], 0.006), STEEL)
    for s in (-1, 1):  # round 6 mm tips seen; square colliders (geometry.TIP_SIDE)
        b.cylinder((0.0, s * g.PIN_Y, g.PIN_LEN / 2), g.PIN_R, g.PIN_LEN, (0.75, 0.75, 0.78), collide=False)
        b.box((0.0, s * g.PIN_Y, g.PIN_LEN / 2), (g.TIP_SIDE, g.TIP_SIDE, g.PIN_LEN), STEEL, visible=False)
    return b.save()


def lever():
    """Lever: frame at its pivot; it reaches forward (-x) over the head to a red handle end."""
    b = Body("lever", mass=g.LEVER_M, com=(-0.05, 0.0, 0.0), inertia=(2e-5, 1e-4, 1e-4), friction=(0.8, 0.7),
             contact_offset=CO, solver_iters=(32, 8))
    x0, x1 = g.LEVER_FRONT_X - g.LEVER_PIVOT[0], 0.004
    b.box(((x0 + x1) / 2, 0.0, 0.0), (x1 - x0, g.LEVER_Y[1] - g.LEVER_Y[0], g.LEVER_THICK), STEEL)
    kx0, kx1 = g.KNOB_X[0] - g.LEVER_PIVOT[0], g.KNOB_X[1] - g.LEVER_PIVOT[0]
    ky0, ky1 = g.KNOB_Y[0] - g.LEVER_PIVOT[1], g.KNOB_Y[1] - g.LEVER_PIVOT[1]
    b.box(((kx0 + kx1) / 2, (ky0 + ky1) / 2, 0.0), (kx1 - kx0, ky1 - ky0, g.LEVER_THICK), RED)
    return b.save()


BLACK = (0.02, 0.02, 0.02)
MARK_W = 0.0008               # printed line width
MARK_HALF = 0.015             # crosshair lines 30 mm long, centred on the mark ...
MARK_EDGE_GAP = 0.001         # ... except the arm toward the leading edge, which stops 1 mm short of it (11 mm)
MARK_GAP = g.CUT / 2 + 0.0005  # the lines start just outside the opening, so they stay on the card after the punch


def card():
    """A6 card stock with two square openings where the pre-scored discs sit, and two black crosshair marks printed
    on the card itself, long enough to stay visible around a punched hole."""
    b = Body("card", mass=g.CARD_M, com=(0, 0, 0), inertia=(g.CARD_M * g.CARD_Y ** 2 / 12, g.CARD_M * g.CARD_X ** 2 / 12,
             g.CARD_M * (g.CARD_X ** 2 + g.CARD_Y ** 2) / 12), friction=(0.4, 0.3), contact_offset=CO, solver_iters=(64, 16))
    holes = [tuple(g.slug_offset(k)[:2]) for k in (0, 1)]
    # collider: card stock with CUT_CLEAR openings, so a tip up to 1.5 mm off its disc passes the card (see geometry)
    _plate_with_holes(b, -g.CARD_X / 2, g.CARD_X / 2, -g.CARD_Y / 2, g.CARD_Y / 2, -g.T / 2, g.T / 2, holes,
                      g.CUT_CLEAR, YELLOW)
    ring = (g.CUT_CLEAR - g.CUT) / 2
    mid = (g.CUT + g.CUT_CLEAR) / 4
    z = g.T / 2 + 0.0001
    for hx, hy in holes:
        # visual card stock between the CUT opening and the collider's wider one: the card shows a CUT opening
        for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            size = (ring, g.CUT_CLEAR, g.T) if sx else (g.CUT, ring, g.T)
            b.box((hx + sx * mid, hy + sy * mid, 0.0), size, YELLOW, collide=False)
        # the crosshair: four arms from the opening's edge outward
        x_edge = g.CARD_X / 2 - MARK_EDGE_GAP
        b.box((hx, hy - (MARK_GAP + MARK_HALF) / 2, z), (MARK_W, MARK_HALF - MARK_GAP, 0.0002), BLACK, collide=False)
        b.box((hx, hy + (MARK_GAP + MARK_HALF) / 2, z), (MARK_W, MARK_HALF - MARK_GAP, 0.0002), BLACK, collide=False)
        b.box((hx - (MARK_GAP + MARK_HALF) / 2, hy, z), (MARK_HALF - MARK_GAP, MARK_W, 0.0002), BLACK, collide=False)
        b.box((hx + MARK_GAP + (x_edge - hx - MARK_GAP) / 2, hy, z), (x_edge - hx - MARK_GAP, MARK_W, 0.0002), BLACK,
              collide=False)
    from pxr import PhysxSchema
    PhysxSchema.PhysxContactReportAPI.Apply(b.stage.GetDefaultPrim()).CreateThresholdAttr(0.0)  # diagnostics sensor
    return b.save()


def slug(k):
    def build():
        b = Body(f"slug{k}", mass=g.SLUG_M, com=(0, 0, 0), inertia=(g.SLUG_M * g.SLUG_R ** 2 / 4,) * 2 +
                 (g.SLUG_M * g.SLUG_R ** 2 / 2,), friction=(0.4, 0.3), contact_offset=CO, solver_iters=(64, 16))
        # a round 6 mm disc seen; a 6 mm square collider (geometry.SLUG_SIDE). The collider stops 0.3 mm above the
        # card's underside, so a disc carried in the card never catches an edge it slides over (the stand-to-bed
        # seam); once free it still falls through the 10 mm die, up to 1.5 mm off the punch's axis
        b.cylinder((0, 0, 0.00015), g.SLUG_R, g.T - 0.0003, YELLOW, collide=False)
        b.box((0, 0, 0.00015), (g.SLUG_SIDE, g.SLUG_SIDE, g.T - 0.0003), YELLOW, visible=False)
        # the crosshair's centre, printed on the disc (visual); it leaves with the disc
        b.box((0, 0, g.T / 2 + 0.0001), (2 * g.SLUG_R - 0.001, MARK_W, 0.0002), BLACK, collide=False)
        b.box((0, 0, g.T / 2 + 0.0001), (MARK_W, 2 * g.SLUG_R - 0.001, 0.0002), BLACK, collide=False)
        return b.save()
    return build


BUILDERS = {"punch": punch, "stand": stand, "head": head, "lever": lever, "card": card,
            "slug0": slug(0), "slug1": slug(1)}
