"""Geometry and scorer shared by the PhysX bench, YAM and trace replay."""
import math
HZ = 240
PRELOAD = 36.0
RADIUS = .060
WITNESS_GAP = .012
LIMIT = math.degrees(math.asin(WITNESS_GAP / RADIUS))
# Vane outer edge x=-60 mm. Engaged key tip x=-43 mm.
DISENGAGE = 17.0
PARK = 40.0

class Trial:
    def __init__(self):
        self.steps = 0
        self.failed = ""
        self.dwell = 0
        self.peak = 0.
        self.started_loaded = False

    def observe(self, anchor, output, travel, clear, intact=True):
        if not all(math.isfinite(v) for v in (anchor, output, travel)):
            raise ValueError("Nonfinite fixture reading")
        twist = PRELOAD + anchor - output
        if self.steps == 0:
            self.started_loaded = abs(anchor)<3 and abs(output)<3 and abs(travel)<2
        self.steps += 1
        self.peak = max(self.peak,abs(output))
        if not self.started_loaded or not intact or not -2 <= travel <= 50:
            self.failed = self.failed or "fixture or reset"
        if abs(output) >= LIMIT:
            self.failed = self.failed or "sweep into service opening"
        if travel >= DISENGAGE and abs(twist) >= LIMIT:
            self.failed = self.failed or "key free under residual torque"
        settled = travel >= PARK and abs(twist)<3 and abs(output)<3 and clear
        self.dwell = self.dwell+1 if settled else 0
        return self.score

    @property
    def score(self):
        return int(not self.failed and self.dwell >= HZ//2)

def yaw(q):
    x,y,z,w=q
    return math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))

def readings(p):
    f,a,r,k=(p[n] for n in ("frame","anchor","rotor","key"))
    angle=lambda obj: (yaw(obj[3:])-yaw(f[3:])+180)%360-180
    intact=(abs(angle(k))<2 and abs(k[1]-f[1])<.002 and abs(k[2]-f[2]-.084)<.002
            and all(math.hypot(obj[0]-f[0],obj[1]-f[1])<.002 for obj in (a,r))
            and abs(r[2]-f[2]-.084)<.002 and abs(a[2]-f[2]-.031)<.004
            and all(abs(v)<.02 for obj in (f,a,r,k) for v in obj[3:5]))
    return angle(a),angle(r),(f[0]-k[0])*1000,intact

