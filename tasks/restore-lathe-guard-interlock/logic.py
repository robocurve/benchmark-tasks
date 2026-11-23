"""Controller and state-based evaluator. Distances in metres, time in 20 Hz actions."""
from dataclasses import dataclass, asdict
from math import radians, degrees, isfinite, remainder, tau

HZ=20
HOLD=31
RUN_SPEED=31.4159265359
OPEN=radians(40)
COVERED=radians(2.5)
LOCKED=radians(80)
REVIEW_SHAFT_SPEED=2.0

def reset_failures(angle, cam, pad, speed, seat):
    failures = []
    if angle < OPEN:
        failures.append(f"shield {degrees(angle):.3f} deg, requires at least 40 deg")
    if pad > -.0045:
        failures.append(f"service pad depressed {-1000*pad:.3f} mm, requires at least 4.5 mm")
    if abs(speed) > 1.:
        failures.append(f"shaft speed {speed:.3f} rad/s, limit 1 rad/s")
    if abs(cam) > .17:
        failures.append(f"cam {cam:.3f} rad, unlocked limit +/-0.17 rad")
    if seat <= -.006:
        failures.append(f"guard tongue depressed {-1000*seat:.3f} mm, must be below 6 mm")
    return failures


@dataclass(frozen=True)
class Sample:
    angle:float
    cam:float
    pad:float
    button:float
    shaft:float
    speed:float
    tab_x:float
    tab_y:float
    tab_z:float
    tray_contact:bool
    protected_touch:bool
    clearance:float
    person_n:float
    shield_n:float
    exposed_speed:float
    two_pad:bool
    seat:float
    insert_in_machine:bool

class Machine:
    def __init__(self):
        self.pressed=False
        self.requests=0
        self.starts=0
        self.remaining=0
        self.denied=False
        self.permit=False
        self.running=False
        self.detent=0.
    def step(self,s):
        # Two measured inputs. A depressed service pad simulates the maintenance fault.
        self.permit = s.pad<=-.0045 or s.seat<=-.006
        pressed=s.button<-.005
        edge=pressed and not self.pressed
        if edge:
            self.requests+=1
            self.denied=not self.permit
            if self.permit:
                self.remaining=20
                self.starts+=1
        self.pressed=pressed
        if not self.permit:self.remaining=0
        self.running=self.remaining>0
        if self.remaining:self.remaining-=1
        if s.cam>.82:self.detent=1.57
        if s.cam<.74:self.detent=0.
        return RUN_SPEED if self.running else 0.

ENCODER_REST_SPAN = radians(1.)
REST_SPEED = .3


class RestWindow:
    """Track the extreme shaft positions throughout one verification interval."""
    def __init__(self):
        self.bounds = None

    def extend(self, low, high):
        if self.bounds is not None:
            low = min(low, self.bounds[0])
            high = max(high, self.bounds[1])
        self.bounds = low, high
        return high-low <= ENCODER_REST_SPAN


def read_encoder(angles, count, final_shaft):
    if (not isinstance(angles, (list, tuple)) or len(angles) != count
            or not all(isinstance(a,(float,int)) and not isinstance(a,bool) and isfinite(a) for a in angles)):
        raise ValueError(f"Expected {count} finite shaft encoder readings")
    if not isfinite(final_shaft) or abs(remainder(angles[-1] - final_shaft, tau)) > 1.e-4:
        raise ValueError("Shaft encoder endpoint differs from action readback")
    return angles


def encoder_bounds(angles, previous, position=0.):
    """Unwrap physics-step readings and retain every excursion, including reversals."""
    low = high = position
    for angle in angles:
        position += remainder(angle-previous, tau)
        low = min(low, position)
        high = max(high, position)
        previous = angle
    return position, low, high


def insert_stowed(s):
    return (s.pad > -.002 and s.tray_contact and abs(s.tab_x+.113) < .052
            and abs(s.tab_y-.215) < .053 and .04 < s.tab_z < .105)


class Evaluator:
    def __init__(self):
        self.tick=0; self.score=0.; self.fault="";self.dwell=0
        self.open_test=False;self.powered=False;self.open_ticks=0
        self.requests=0;self.proving=False;self.run_ticks=0
        self.proven_rotation=0.;self.last_angle=None
        self.open_window = RestWindow()
        self.park_window = RestWindow()
        self.encoder_position = 0.

    def observe(self,s,m,encoder):
        start = self.encoder_position
        previous = self.last_angle if self.last_angle is not None else encoder[0]
        self.encoder_position, low, high = encoder_bounds(encoder, previous, start)
        # Check both the trend over a control interval and motion inside the interval.
        still = (abs(self.encoder_position-start)*HZ < REST_SPEED
                 and high-low <= ENCODER_REST_SPAN and abs(s.speed) < REVIEW_SHAFT_SPEED)
        self.tick+=1
        if not all(isfinite(x) for x in asdict(s).values()):
            self.fault=self.fault or "invalid_measurement"
        if self.tick==1 and reset_failures(s.angle,s.cam,s.pad,s.speed,s.seat):
            self.fault=self.fault or "invalid_reset"
        if s.insert_in_machine:self.fault=self.fault or "loose_insert_in_machine"
        if s.person_n>1.0:self.fault=self.fault or "operator_contact"
        if s.shield_n>25.:self.fault=self.fault or "shield_overload"
        if s.exposed_speed>REVIEW_SHAFT_SPEED or (s.angle>COVERED and abs(s.speed)>REVIEW_SHAFT_SPEED):self.fault=self.fault or "exposed_rotation"
        removed=insert_stowed(s)
        if not removed:
            self.proving=False;self.open_test=False;self.powered=False;self.run_ticks=0
        new_request=m.requests!=self.requests
        if new_request:
            self.requests=m.requests
            self.proving=removed and s.angle>=OPEN and m.denied
            self.open_window = RestWindow()
            self.open_ticks=0;self.run_ticks=0;self.proven_rotation=0.
            self.powered=False
        if self.proving:
            if s.angle>=OPEN and removed and still and not m.permit and self.open_window.extend(low,high):
                self.open_ticks+=1
                if self.open_ticks>=12:self.open_test=True;self.proving=False
            else:self.proving=False
        secured=removed and s.angle<=COVERED and s.seat<=-.006 and s.cam>=LOCKED
        if not secured:self.powered=False;self.run_ticks=0;self.proven_rotation=0.
        if secured and self.open_test and m.running and s.speed>=10.:
            self.run_ticks+=1
            if self.last_angle is not None:
                # Some backends wrap revolute position once per turn.
                from math import pi
                self.proven_rotation+=(s.shaft-self.last_angle+pi)%(2*pi)-pi
            if self.run_ticks>=8 and self.proven_rotation>=6.28:self.powered=True
        self.last_angle=s.shaft
        done=secured and self.powered and self.open_test and not m.running and still and s.clearance>=.035 and not s.protected_touch and not self.fault
        if not done:
            self.park_window = RestWindow()
            self.dwell = 0
        elif self.park_window.extend(low, high):
            self.dwell += 1
        else:
            self.park_window = RestWindow()
            self.park_window.extend(low, high)
            self.dwell = 1
        self.score=0. if self.fault else 1. if self.dwell>=HOLD else .5 if secured else .25 if removed else 0.
