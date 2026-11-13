"""All dimensions in metres, camera body frame; insertion is along -Z.
Manufacturing concept, not a certified physical fixture. 1 mm running clearance.
"""
# Rear shoe: asymmetric 20 mm wide tongue, 4 mm thick, 24 mm long.
# All rails are behind the optical housing, never in front of the lens.
SHOE=[('shoe_main',(0,-.022,-.010),(.020,.004,.020)),
      ('shoe_leading',(-.0015,-.022,-.022),(.017,.004,.004)),
      ('shoe_latch_top',(.009,-.022,-.002),(.002,.004,.004)),
      ('shoe_latch_bottom',(.009,-.022,-.017),(.002,.004,.006))]
SHOE.append(('shoe_bridge',(0,-.019,-.005),(.008,.002,.014)))
# Right edge notch occupies x[8,10], z[-14,-4] mm (main core width18).
SHOE[0]=('shoe_core',(-.001,-.022,-.010),(.018,.004,.020))
MOUNT=[('channel_left',(-.013,-.022,-.012),(.004,.008,.028)),
       ('channel_right',(.013,-.022,-.012),(.004,.008,.028)),
       ('channel_back',(0,-.027,-.012),(.030,.002,.028)),
       ('channel_front_left',(-.009,-.0185,-.012),(.008,.001,.028)),
       ('channel_front_right',(.009,-.0185,-.012),(.008,.001,.028)),
       ('end_stop',(0,-.022,-.027),(.030,.012,.004))]
# spring tooth engages right-side notch at z=-9 mm. Explicit discrete spring
# approximation: retracted during sliding, extends only after valid entry/seating.
TOOTH_RETRACTED=(.014,-.022,-.009)
TOOTH_ENGAGED=(.0095,-.022,-.009)
TOOTH_SIZE=(.003,.003,.004)

# Keep housing clear of rail lips: translated rear interface, unchanged optical pose.
SHOE=[(n,(p[0],p[1]-.003,p[2]),sz) for n,p,sz in SHOE if n!='shoe_bridge']
SHOE.append(('shoe_bridge',(0,-.0205,-.005),(.008,.005,.014)))
MOUNT=[(n,(p[0],p[1]-.003,p[2]),sz) for n,p,sz in MOUNT]
TOOTH_RETRACTED=(TOOTH_RETRACTED[0],TOOTH_RETRACTED[1]-.003,TOOTH_RETRACTED[2])
TOOTH_ENGAGED=(TOOTH_ENGAGED[0],TOOTH_ENGAGED[1]-.003,TOOTH_ENGAGED[2])

class InsertionGate:
 def __init__(self):self.entered=False;self.previous=None;self.latched=False
 def update(self,offset,angle_rad,speed):
  import math
  x,y,z=map(float,offset)
  corridor=abs(x)<=.0012 and abs(y)<=.0012 and angle_rad<=math.radians(3)
  # Entry must be observed before the tongue touches the mouth, then preserved.
  if not self.latched:
   if corridor and .026<=z<=.050:self.entered=True
   if not corridor:self.entered=False
   if self.entered and abs(z)<=.0012 and speed<.03:
    self.latched=True
  self.previous=(x,y,z)
  return self.latched
