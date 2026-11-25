"""Generate the water trainer fabrication STEP files with CadQuery 2.7.0.

Units are mm, axes are station axes. For moving parts z=0 is the pointer.
The fixed rack alone uses station world z. Threads use tap-drill diameters.
"""
from pathlib import Path
from math import sqrt, sin, cos, radians
import cadquery as cq

OUT = Path(__file__).resolve().parent
V = cq.Vector

def box(x0,x1,y0,y1,z0,z1):
    return cq.Solid.makeBox(x1-x0,y1-y0,z1-z0,V(x0,y0,z0))

def cyl(radius,length,start,axis):
    return cq.Solid.makeCylinder(radius,length,V(*start),V(*axis))

def hx(part,x0,x1,y,z,diameter):
    return part.cut(cyl(diameter/2,x1-x0,(x0,y,z),(1,0,0)))

def hz(part,x,y,z0,z1,diameter):
    return part.cut(cyl(diameter/2,z1-z0,(x,y,z0),(0,0,1)))

def join(*parts):
    return parts[0].fuse(*parts[1:]).clean()

def save(name,part):
    cleaned=part.clean()
    if cleaned.isValid():
        part=cleaned
    if not part.isValid() or len(part.Solids())!=1:
        raise ValueError(f'{name}: invalid or disconnected CAD')
    filename=OUT / (name+'.step')
    cq.exporters.export(part,str(filename))
    check=cq.importers.importStep(str(filename)).val()
    if not check.isValid() or len(check.Solids()) != 1 or abs(check.Volume()-part.Volume())>0.01:
        raise ValueError(f'{name}: STEP round trip failed')

# Rear lane for the fixed rack. Two front bridges join both plate sections.
carriage=box(380.5,389.5,-107.5,107.5,-80,120).cut(box(379,391,-86,-53,-81,121))
for z in (-77,87):
    bridge=join(box(375,379.5,-105,-43,z,z+8),
                box(379.4,381,-105,-87,z,z+8),box(379.4,381,-52,-43,z,z+8))
    carriage=join(carriage,bridge)
carriage=carriage.cut(box(379,391,-77,-34,40,50))
carriage=carriage.cut(box(379,391,33,64,57,86))
for y,z in [(25,68),(72,68),(32,104),(69,104)]:
    carriage=hx(carriage,374,391,y,z,3.2)
for y in (-7.5,7.5):
    for z in (-5,5):
        carriage=hx(carriage,374,391,y,z,3.2)
for y,z in [(45,0),(-46,32),(-29,32),(-99,-60),(-99,-39)]:
    carriage=hx(carriage,374,391,y,z,3.2)
save('carriage',carriage)
# Metal tooth comb machined or waterjet at true 12 mm pitch; backside attachment.
rack=box(380,386,-84,-74,921.75,1149.75)
for i in range(18):
    z=933.75+12*i
    rack=join(rack,box(380,386,-74,-56,z-1.5,z+1.5))
for z in (927.75,1143.75):
    rack=hx(rack,379,387,-79,z,3.2)
    rack=rack.cut(cq.Solid.makeCone(3.2,1.6,1.6,V(380,-79,z),V(1,0,0)))
save('rack',rack)

# The yoke outline is one 3 mm flat blank; machining is flat from both faces.
pin=join(box(380.5,387.5,-72.5,-57.5,43.5,46.5),
         box(384,398,-67.5,-62.5,43.5,46.5),
         box(394,398,-65.5,-38.5,43.5,46.5),
         box(313.5,398.5,-42,-36,43.5,46.5),
         box(314,322,-66,-36,43.5,46.5),
         box(335,338,-39,-23,43.5,46.5))
for y in (-64,-59):
    pin=hz(pin,318,y,43,47,1.6)
save('locking_yoke',pin)

# Split guide, spring seat and positive stops. Sliding yoke is laid in from above.
guide=join(box(331,380.5,-50,-24,29,39),
           box(348,378,-47,-32,39,43.38),
           box(331,335,-35,-33,39,47),
           box(346,350,-35,-33,39,47),
           box(356.8,360,-33,-23,39,50))
# Extra bearing cheeks constrain the 6 mm wide link; cover countersunk or low-head.
guide=join(guide,box(348,378,-47,-42.12,43.38,47.5),box(348,378,-35.88,-32,43.38,47.5))
for x,y in [(351,-44.6),(375,-44.6)]:
    guide=hz(guide,x,y,39,49,1.6)
# Two rear attachment pilot holes tap M3. Spring guides on a 3mm smooth post.
for y in (-46,-29):
    guide=hx(guide,369,382,y,32,2.5)
guide=hx(guide,355,361,-28,45,3.0)
save('pin_housing',guide)
cover=box(348,378,-47,-32,46.62,49.12)
# Relief mating cheeks so underside of middle strip sits at 46.62.
for ya,yb in [(-48,-42.12),(-35.88,-31)]:
    cover=cover.cut(box(347,379,ya,yb,46.5,47.5))
for x in (351,375):
    cover=hz(cover,x,-44.6,46,50,2.2)
cover=cover.cut(box(355.8,360.2,-33.2,-22,46,50))
save('pin_housing_cap',cover)

# Source-registered robot contact surfaces.
neck=join(box(300,330,34,56,-9,9),box(320,380.5,39,51,-6,6))
neck=hx(neck,369,382,45,0,2.5)
save('lift_neck',neck)
tab=box(301,329,-75,-55,35,55)
# Slot opens to y+ and x+, allowing assembly over the planar yoke.
tab=tab.cut(box(313.8,322.2,-67.4,-54.5,43.43,46.57))
for y in (-64,-59):
    tab=hz(tab,318,y,34,56,2.2)
    tab=hz(tab,318,y,51,56,4.0)
save('brake_tab',tab)

# Plug valve liquid cartridge; body bolts from outside the rack clearance lane.
body=join(box(359,377,-94,-76,-59,-41),box(365,380.5,-103,-93,-63,-36),
          box(350,369,-103,-98,-62,-38),box(336,354,-104,-101,-62,-34),box(350,353,-103,-98,-62,-38))
body=hx(body,358,378,-85,-50,12.0)
# Through housing and replaceable liner. Nipple tails fit 5 mm OD tubing stock.
body=body.cut(cyl(1.5,28,(368,-104,-50),(0,1,0)))
body=body.cut(cyl(2.5,12.0,(368,-103,-50),(0,1,0)))
body=body.cut(cyl(2.5,3.0,(368,-79,-50),(0,1,0)))
for z in (-60,-39):
    body=hx(body,370,382,-99,z,2.5)
# Stationary front annulus holds an axial M5 ball plunger. Control sweep is forward of this.
axis=(1,0,0)
ring=cyl(17.3,5,(336,-85,-50),axis).cut(cyl(8,7,(335,-85,-50),axis))
ring=hx(ring,335,342,-85,-39.5,4.2)
body=join(body,ring)
# Stationary rear annulus guides the 2 mm axial dowel in a 90 degree round-ended slot.
slotplate=cyl(17.3,3,(352.6,-85,-50),axis).cut(cyl(7.2,5,(352,-85,-50),axis))
y0,z0=-85,-50
def arcpt(a,rad):
    return (y0+rad*sin(radians(a)),z0+rad*cos(radians(a)))
slotprofile=cq.Workplane('YZ',origin=(351.8,0,0)).moveTo(*arcpt(0,11.09)).threePointArc(arcpt(45,11.09),arcpt(90,11.09)).lineTo(*arcpt(90,8.91)).threePointArc(arcpt(45,8.91),arcpt(0,8.91)).close().extrude(5).val()
slotplate=slotplate.cut(slotprofile)
for a in (0,90):
    y,z=arcpt(a,10)
    slotplate=hx(slotplate,351,357,y,z,2.18)
body=join(body,slotplate)
body=body.cut(box(347.7,352.3,-98.95,-97.8,-55,-45))
save('tap_body',body)
# Non-wetted control face and one-piece acetal plug. Cross holes in closed orientation.
plug=join(box(325,358,-89.5,-80.5,-54.5,-45.5),
          cyl(4.075,21,(357,-85,-50),(1,0,0)),
          box(316.5,325.5,-88.5,-81.5,-52.5,-27.5),
          box(302,328,-91,-79,-36,-24))
plug=hz(plug,368,-85,-55,-45,3.2)
# Closed +Z bore rotated +45 about X so that -45 operating rotation aligns it with Y
# (unoriented bore axis in closed frame has +Y,+Z components).
dr=(0,1/sqrt(2),1/sqrt(2))
plug=plug.cut(cyl(.25,12,(368,-85-6*dr[1],-50-6*dr[2]),dr))
liner=cyl(6,18,(359,-85,-50),(1,0,0)).cut(cyl(4,20,(358,-85,-50),(1,0,0)))
liner=liner.cut(cyl(1.5,18,(368,-94,-50),(0,1,0)))
save('tap_liner',liner)
# In the closed rotor frame the port axis at full open is +Z, and at half open is (+Y,+Z).
# Two tapered surface channels connect the pilot and main ports. Sweep via a ball mill
# or use an indexed hand fixture and check the resulting specimen gravimetrically.
def stations_to_solid(profiles):
    return cq.Solid.makeLoft([cq.Wire.makePolygon([V(*v) for v in pts]+[V(*pts[0])]) for pts in profiles],True)

def groove(side):
    sections=[]
    for angle in range(34,73,2):
        t=(angle-34)/38
        halfwidth=.10+.12*t
        depth=.08+.10*t
        a=radians(angle)
        def xyz(axial,r):
            return (368+axial,-85+side*r*cos(a),-50+side*r*sin(a))
        sections.append([xyz(-halfwidth,4.13),xyz(0,4.075-depth),xyz(halfwidth,4.13)])
    return stations_to_solid(sections)
plug=plug.cut(groove(1)).cut(groove(-1))
plug=plug.cut(cyl(.8,12,(350,-91,-50),(0,1,0)))
plug=plug.cut(cyl(1,12,(356.95,-91,-50),(0,1,0)))
save('tap_rotor',plug)

# Dry keyed cam. Retain axially with the plug collars. Axial ball load increases
# approaching the central ridge; the opposite downslope seats the other endpoint.
crank=cyl(13.3,4,(348,-85,-50),(1,0,0))
crank=crank.cut(box(347,353,-89.55,-80.45,-54.55,-45.45))
crank=hx(crank,347,353,-85,-40,2.0)
# Full-thickness base disk x348..352. Smooth triangular axial face on r8..13 with
# hard stops at each end. Extend ramps a little beyond both ends for ball support.
profiles=[]
for a in range(-12,103,3):
    h=max(0.05,1.6*(1-abs(45-a)/45))
    an=radians(a)
    def polar(x,r):
        return (x,-85-r*sin(an),-50+r*cos(an))
    profiles.append([polar(348.04,8.0),polar(348.04,13.1),polar(348-h,13.1),polar(348-h,8.0)])
crank=join(crank,stations_to_solid(profiles))
# Blind bore to accept 2 mm dowel projecting into rear slot, clear of the cam face.
crank=hx(crank,349,353,-85,-40,2)
crank=crank.cut(cyl(.8,30,(350,-100,-50),(0,1,0)))
save('tap_crank',crank)
