"""Collision geometry for an 8-inch lamp harp assembly."""

import math

from scoring import PITCH

FACES = (
    (0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
    (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0),
)
BRASS = (.71, .52, .26)
LIGHT_BRASS = (.81, .64, .35)
DARK = (.22, .26, .27)
LINEN = (.89, .80, .60)

SADDLE_PLATE = .006
ASSEMBLED_HARP_HEIGHT = .2032
HARP_HEIGHT = ASSEMBLED_HARP_HEIGHT-SADDLE_PLATE
LEG_PIVOT = .151
LEG_Y = .042
LIP_INNER = .0419
WING_OUTER_Y = .0475
LIP_Z = (.010, .014)
TOE_OUT = .002
TOE_IN = .0044
COLLAR_Z = .012
COLLAR_LENGTH = .022
CARRY_SLEEVE_RISE = .046
COLLAR_BORE = .00685
COLLAR_THROAT = .00265
COLLAR_OUTER = .0095
COLLAR_MOUTH = .00835
COLLAR_BEVEL = .004
POST_ROOT_RADIUS = .00262
POST_LENGTH = .008
FINIAL_BORE_RADIUS = .00332
SHADE_HEIGHT = .139
SHADE_WASHER = .0016
FINIAL_HEIGHT = .024


def sector(a, b, lower, upper):
    """Make a convex annular sector between two radial x-z profiles."""
    def xyz(angle, profile):
        return tuple((float(r*math.cos(angle)), float(r*math.sin(angle)), float(z))
                     for r, z in profile)
    return xyz(a, lower)+xyz(b, upper)


def ring(inner, outer, bottom, top, n=16, outer_top=None, inner_top=None):
    if outer_top is None:
        outer_top = outer
    if inner_top is None:
        inner_top = inner
    profile = ((inner,bottom), (outer,bottom), (outer_top,top), (inner_top,top))
    return tuple(sector(2*math.pi*i/n, 2*math.pi*(i+1)/n, profile, profile)
                 for i in range(n))


def offset(wedges, x=0., y=0., z=0.):
    return tuple(tuple((a+x,b+y,c+z) for a,b,c in points) for points in wedges)


def threads(internal):
    """Model the mating thread flanks as convex segments of a right-hand helix."""
    # The female ridge lies halfway between successive male crests, with
    # approximately 0.1 mm radial running clearance and clear axial flanks.
    n = 20
    turns = 5. if internal else 6.
    z0 = 0. if internal else .0016
    phase = PITCH/2 if internal else 0.
    root,crest = (FINIAL_BORE_RADIUS,.00284) if internal else (POST_ROOT_RADIUS,.00310)
    half_width = .24*PITCH
    half_crest = .070*PITCH

    def profile(t):
        z = z0+phase+PITCH*t
        if internal:
            return ((crest,z-half_crest),(root,z-half_width),
                    (root,z+half_width),(crest,z+half_crest))
        return ((root,z-half_width),(crest,z-half_crest),
                (crest,z+half_crest),(root,z+half_width))

    return tuple(sector(2*math.pi*i/n,2*math.pi*(i+1)/n,
                        profile(i/n),profile((i+1)/n))
                 for i in range(round(turns*n)))


def sleeve_body():
    bottom = -COLLAR_LENGTH/2
    return (
        ring(COLLAR_MOUTH,COLLAR_OUTER,bottom,bottom+COLLAR_BEVEL,16,inner_top=COLLAR_BORE)
        +ring(COLLAR_BORE,COLLAR_OUTER,bottom+COLLAR_BEVEL,-bottom,16)
        +ring(COLLAR_THROAT,COLLAR_BORE,.008,.011,16)
    )


def sleeve_blocking_band(lower, upper):
    """Return the wall height that blocks release before the toe clears its shoulder."""
    release_y = LIP_INNER-TOE_OUT
    limiting_radius = WING_OUTER_Y-release_y-.0002
    bevel_fraction = (COLLAR_MOUTH-limiting_radius)/(COLLAR_MOUTH-COLLAR_BORE)
    blocking_start = lower+COLLAR_BEVEL*max(0.,min(1.,bevel_fraction))
    return max(0.,min(upper,LIP_Z[1])-max(blocking_start,LIP_Z[0]))


def post_tip():
    return ring(.0003,POST_ROOT_RADIUS,.0072,POST_LENGTH,16,.0019)


def finial_body():
    # The short entry chamfer ends before the first internal thread flank.
    return (ring(.00354,.006,0.,.00022,18,inner_top=FINIAL_BORE_RADIUS)
            +ring(FINIAL_BORE_RADIUS,.006,.00022,.007,18)
            +threads(True))


def harp_crown():
    """Turn the upper half of an annulus into a wire arch in the YZ plane."""
    segments = ring(.04025,.04375,-.00175,.00175,24)[:12]
    rise = HARP_HEIGHT-.04375
    return tuple(tuple((depth,y,z+rise) for y,z,depth in sector) for sector in segments)


def saddle_boxes():
    boxes = [((0.,0.,-SADDLE_PLATE/2),(.021,.094,SADDLE_PLATE),BRASS)]
    for s in (-1,1):
        boxes += [
            ((0.,s*.0458,.0085),(.006,.0034,.017),BRASS),
            ((0.,s*((LIP_INNER+WING_OUTER_Y)/2),sum(LIP_Z)/2),
             (.006,WING_OUTER_Y-LIP_INNER,LIP_Z[1]-LIP_Z[0]),LIGHT_BRASS),
            ((-.00375,s*.0425,.0045),(.0015,.009,.009),BRASS),
            ((.00375,s*.0425,.0045),(.0015,.009,.009),BRASS),
        ]
    return tuple(boxes)


def saddle_entrances():
    """Bevel the inward ends of the formed cheeks, leaving the seated gap unchanged."""
    bevel = ((.0043,.035),(.0045,.035),(.0045,.038),(.003,.038))
    wings = []
    for side in (-1,1):
        for cheek in (-1,1):
            profile = bevel[::-1] if side*cheek > 0 else bevel
            wings.append(tuple((cheek*x,side*y,z) for z in (0.,.009) for x,y in profile))
    return tuple(wings)


def leg_boxes(s):
    return (
        ((0.,0.,-.067),(.0035,.0035,.140),BRASS),
        ((0.,-s*.0018,.016-LEG_PIVOT),(.0035,.0048,.004),BRASS),
        ((0.,-s*.00325,.0115-LEG_PIVOT),(.0035,.0014,.005),BRASS),
        ((0.,s*(TOE_OUT-TOE_IN)/2,.0045-LEG_PIVOT),(.0035,TOE_OUT+TOE_IN,.009),BRASS),
    )


def shade_wedges():
    skirt = ring(.1068,.108,0.,SHADE_HEIGHT,24,.071,.0698)
    hem = ring(.1055,.108,0.,.003,24)
    rim = ring(.0685,.071,SHADE_HEIGHT-.004,SHADE_HEIGHT,24)
    washer = ring(.0111125/2,.012,SHADE_HEIGHT-SHADE_WASHER,SHADE_HEIGHT,16)
    spokes = []
    for i in range(3):
        a = i*2*math.pi/3-math.pi/2
        c,s = math.cos(a),math.sin(a)
        profile = ((.011,-.001),(.070,-.001),(.070,.001),(.011,.001))
        points = tuple((r*c-w*s,r*s+w*c,SHADE_HEIGHT-.001+z)
                       for w in (-.00125,.00125) for r,z in profile)
        spokes.append(points)
    return skirt+hem+rim+washer+tuple(spokes)
