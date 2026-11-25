"""Nominal Camry pad-and-rotor geometry and two service-gauge profiles."""

import math

STEEL = (.35,.38,.41)
ROTOR = (.58,.63,.67)
FRICTION = (.18,.19,.20)
GREEN = (.23,.62,.32)
RACK = (.25,.36,.40)
RADIUS = .148
ROTOR_HEIGHT = .049
BACKING_THICKNESS = .006
THICKNESSES = (.008,.010)
GRASP = (0.,-.088,.022)
HANDLE_BASE = .004
TOOL_WIDTH = .004


def box(center, size, color):
    return tuple(float(v) for v in center), tuple(float(v) for v in size), color


def ring(inner, outer, lo, hi, n=32):
    profile = ((inner,lo),(outer,lo),(outer,hi),(inner,hi))
    return tuple(
        tuple((r*math.cos(a),r*math.sin(a),z) for a in (i*2*math.pi/n,(i+1)*2*math.pi/n)
              for r,z in profile)
        for i in range(n)
    )


# The inner edge of the factory head projects past the shank.
HEAD_LIP = .0027


def gauge(thickness):
    half = thickness / 2
    return (
        box((0., 0., 0.), (TOOL_WIDTH, thickness, thickness), GREEN),
        box((0., -(.046+half)/2, -HEAD_LIP/2),
            (TOOL_WIDTH, .046-half, thickness-HEAD_LIP), GREEN),
        box((0., -.089, HANDLE_BASE+.009), (TOOL_WIDTH, .029, .018), GREEN),
    )


def gauge_profile(thickness):
    half = thickness/2
    offset = ((-.046, -half), (-.074, HANDLE_BASE),
              (-.074, HANDLE_BASE+.018), (-.046, half-HEAD_LIP))
    ramp = tuple((x,y,z) for x in (-TOOL_WIDTH/2, TOOL_WIDTH/2) for y,z in offset)
    # Hollow molded lanyard eye at the heel, outside the grip and measuring faces.
    heel = tuple(tuple((z,y-.103,x+HANDLE_BASE+.009) for y,x,z in segment)
                 for segment in ring(.0032,.009,-TOOL_WIDTH/2,TOOL_WIDTH/2,20))
    return (ramp,*heel)


def gauge_marks(thickness):
    letters = {
        '0': ((0,0),(0,1),(1,1),(1,0),(0,0)),
        '1': ((.5,1),(.5,0)),
        '8': ((1,.5),(1,1),(0,1),(0,0),(1,0),(1,.5),(0,.5)),
        'M': ((0,0),(0,1),(.5,.5),(1,1),(1,0)),
    }
    for i, char in enumerate(f"{round(thickness*1000)}MM"):
        points = letters[char]
        for (a,b),(c,d) in zip(points,points[1:]):
            # Short bars are an approximation of the molded stock markings.
            y = -.098+(i*1.4+(a+c)/2)*.0032
            z = HANDLE_BASE+.005+(b+d)*.0024
            size = (.00009,max(abs(a-c)*.0032,.00036),max(abs(b-d)*.0048,.00036))
            if a == c or b == d:
                yield box([TOOL_WIDTH/2+.00007,y,z],size,(.19,.49,.27))
            else:
                for j in range(6):
                    step = (j+.5)/6-.5
                    yield box([TOOL_WIDTH/2+.00007,y+step*(c-a)*.0032,z+step*(d-b)*.0048],
                              (.00009,.00045,.00045),(.19,.49,.27))

# Hub coordinates use X radially toward the caliper, Y along the rotor face
# and Z axially outboard. The wheel is indexed once at reset.
WHEEL_FACE = .094
SPOKE_INNER = .087
RIM_INNER = .187
SPOKE_COUNT = 7
SPOKE_ZERO = math.radians(-30) + math.pi/SPOKE_COUNT
SILVER = (.63,.65,.68)
TIRE = (.13,.135,.145)
CALIPER = (.40,.39,.37)
KNUCKLE = (.23,.24,.25)


def wheel_opening(point):
    x,y,_ = point
    radius = math.hypot(x,y)
    if not SPOKE_INNER+.004 < radius < RIM_INNER-.004:
        return False
    period = 2*math.pi/SPOKE_COUNT
    angle = (math.atan2(y,x)-SPOKE_ZERO+period/2) % period-period/2
    half_spoke = .012 + .004*(radius-SPOKE_INNER)/(RIM_INNER-SPOKE_INNER)
    return abs(math.sin(angle)*radius) > half_spoke+.004


class SpokeAccess:
    def __init__(self):
        self.previous = None
        self.inside = False

    def update(self, point):
        previous = self.previous
        self.previous = point.copy()
        if point[2] >= WHEEL_FACE:
            self.inside = False
        elif previous is not None and previous[2] >= WHEEL_FACE:
            fraction = (previous[2]-WHEEL_FACE)/(previous[2]-point[2])
            self.inside = wheel_opening(previous+(point-previous)*fraction)
        return self.inside


def spokes():
    for i in range(SPOKE_COUNT):
        angle = SPOKE_ZERO+i*2*math.pi/SPOKE_COUNT
        c,s = math.cos(angle),math.sin(angle)
        yield tuple((c*r-s*t,s*r+c*t,z) for z in (.081,WHEEL_FACE)
                    for r,t in ((.071,-.012),(.192,-.016),(.192,.016),(.071,.012)))


def caliper_shell(radial):
    # Single-piston floating caliper. The leading ear pocket is retained
    # between the carrier horn and the larger cast bridge.
    return (
        box((radial+.056,.010,.035),(.029,.142,.135),CALIPER),
        box((radial+.017,.018,.077),(.068,.123,.019),CALIPER),
        box((radial-.017,.018,.062),(.022,.114,.028),CALIPER),
        box((radial+.009,-.094,.048),(.048,.015,.101),KNUCKLE),
        box((radial+.009,.092,.048),(.048,.016,.101),KNUCKLE),
        box((radial-.014,.005,-.015),(.096,.144,.035),CALIPER),
        box((radial+.045,-.087,.049),(.021,.018,.069),CALIPER),
    )


def installed_pad(lining):
    backing_z = lining+BACKING_THICKNESS/2
    return (
        box((.002,0.,backing_z),(.050,.128,BACKING_THICKNESS),STEEL),
        *(box((.004,s*.068,backing_z),(.024,.022,BACKING_THICKNESS),STEEL)
          for s in (-1,1)),
        box((.001,0.,lining/2),(.043,.118,lining),FRICTION),
    )
