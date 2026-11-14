"""Product-informed dimensional reconstruction, not measured Ingenio CAD.

All lengths/masses below are declared prototype estimates. The jaw is driven by
a passive compliant slider-crank; there is no pan/handle attachment constraint.
The release buttons act on the lower lever through native contact.
"""
import math

import numpy as np

from resist_isaac.recovery import box, body, description

TABLE = .7475
PAN_HEIGHT = .095
RIM_RADIUS = .098  # Estimated flared outside radius of an 18 cm nominal saucepan.
RIM_DATUM_HEIGHT = .097
PAN_START = (.30, .16, TABLE)
TRIVET = (.30, -.17, TABLE)
PIVOT = np.array([-.145, 0., -.012])
CRANK = .055
ROD = .040
OPEN_ANGLE = math.radians(25)
CLOSED_ANGLE = math.radians(-5)


def ry(angle):
    return np.array([[math.cos(angle), 0, math.sin(angle)], [0, 1, 0],
                     [-math.sin(angle), 0, math.cos(angle)]])


def quat(angle):
    return [math.cos(angle / 2), 0, math.sin(angle / 2), 0]


def lathe(profile, count=96):
    points = [[r * math.cos(2*math.pi*i/count), r * math.sin(2*math.pi*i/count), z]
              for r, z in profile for i in range(count)]
    faces = [[j*count+i, j*count+(i+1)%count, (j+1)*count+(i+1)%count, (j+1)*count+i]
             for j in range(len(profile)-1) for i in range(count)]
    return dict(points=points, faces=faces)


def pan():
    # The product photo has an outward-flared pouring lip, not a 5 mm bead.
    shapes = [dict(type='cylinder', size=[.084, .002], position=[0, 0, .002],
                   color=[.55, .57, .60, 1], friction=[.45, .35])]
    bands = [((.084,.004),(.090,.084),(.0885,.0847),(.0825,.004)),
             ((.090,.084),(RIM_RADIUS,.0935),(RIM_RADIUS,.095),(.0885,.0847))]
    for band in bands:
        for i in range(64):
            a, b = 2*math.pi*i/64, 2*math.pi*(i+1)/64
            points = [[r*math.cos(t), r*math.sin(t), z] for r,z in band for t in (a,b)]
            faces = [[0,2,4,6],[1,7,5,3],[0,1,3,2],[2,3,5,4],[4,5,7,6],[6,7,1,0]]
            shapes.append(dict(type='mesh', mesh=dict(points=points, faces=faces),
                               visible=False, friction=[.45,.35], contact_offset=.00005))
    profile = [(0,0),(.082,0),(.084,.004),(.090,.084),(RIM_RADIUS,.0935),
               (RIM_RADIUS,.095),(.0885,.0847),(.0825,.004),(0,.004)]
    shapes.append(dict(type='mesh', mesh=lathe(profile), collision=False,
                       color=[.64,.66,.69,1]))
    return description('Saucepan', [dict(name='pan', mass=.65, center_of_mass=[0,0,.041],
        inertia=dict(diagonal=[.0020,.0020,.0036]),shapes=shapes)],
        solver_position_iterations=32,solver_velocity_iterations=8)


def handle():
    # Elliptical housing sections preserve hand access around the lower lever.
    sections=[(-.165,.007,.006),(-.153,.016,.012),(-.12,.018,.013),
              (-.075,.017,.013),(-.035,.018,.015),(-.014,.014,.010)]
    points=[[x,w*math.cos(t),.012+h*math.sin(t)]
            for x,w,h in sections for t in np.linspace(0,2*math.pi,16,endpoint=False)]
    faces=[[j*16+i,j*16+(i+1)%16,(j+1)*16+(i+1)%16,(j+1)*16+i]
           for j in range(len(sections)-1) for i in range(16)]
    faces += [list(reversed(range(16))),list(range((len(sections)-1)*16,len(sections)*16))]
    shell=[dict(type='mesh',mesh=dict(points=points,faces=faces),color=[.075,.075,.08,1],
                friction=[.7,.5])]
    # Fixed inner side lugs and outer central pad locate the rim. The moving
    # fork approaches its underside, following the reference's figures 3–6.
    # Vertically staggered supports permit angled insertion around the flare.
    # Their dimensions remain estimates; the topology and face angles follow
    # WO2017006039A1, figures 3–6 and support members 12, 14, 15 and 21.
    for y in (-.016,.016):
        shell += [box((.002,.004,.009),(.0142,y,-.006),(.45,.47,.50,1)),
                  box((.016,.004,.002),(-.001,y,.003),(.45,.47,.50,1))]
    shell += [box((.0025,.005,.007),(.006782,0,-.030),(.45,.47,.50,1),
                  orientation=quat(-math.atan(.006/.080))),
              box((.002,.005,.017),(-.008,0,-.013),(.45,.47,.50,1)),
              box((.009,.005,.002),(-.0005,0,-.029),(.45,.47,.50,1)),
              box((.0045,.004,.001),(-.001184,0,.00219),(.45,.47,.50,1),
                  orientation=quat(math.pi/3))]
    housing=dict(name='housing',mass=.17,center_of_mass=[-.09,0,.009],
                 inertia=dict(diagonal=[.00003,.00034,.00034]),shapes=shell)
    lever=body('lever',[box((.050,.010,.003),(.050,0,0),(.11,.11,.12,1)),
                        box((.004,.009,.004),(CRANK,0,0),(.48,.50,.53,1))],.030,PIVOT)
    lever['orientation']=quat(OPEN_ANGLE)
    pin=PIVOT+ry(OPEN_ANGLE)@np.array([CRANK,0,0])
    rear=np.array([pin[0]+math.sqrt(ROD**2-(pin[2]-PIVOT[2])**2),0.,PIVOT[2]])
    direction=(pin-rear)/ROD
    angle=-math.atan2(direction[2],direction[0])
    # The fork's arms pass beside the fixed central leg; no rail passes through it.
    fork = [box((.019,.004,.0015),(-.044,0,-.012),(.45,.47,.5,1)),
            box((.004,.0115,.0015),(-.030,0,-.012),(.45,.47,.5,1))]
    for y in (-.009,.009):
        profile = [(-.017,.0024),(-.009,-.0056),(-.009,-.0086),(-.017,-.0006)]
        vertices = [[x,y+offset,z] for offset in (-.0025,.0025) for x,z in profile]
        facets = [[3,2,1,0],[4,5,6,7]]
        facets += [[i,(i+1)%4,(i+1)%4+4,i+4] for i in range(4)]
        fork += [dict(type='mesh',mesh=dict(points=vertices,faces=facets),color=[.53,.55,.58,1]),
                 box((.009,.0025,.0015),(-.023,y,-.012),(.45,.47,.5,1)),
                 box((.0015,.0025,.005),(-.015,y,-.007),(.45,.47,.5,1))]
    jaw = dict(name='jaw',mass=.035,center_of_mass=[-.035,0,-.010],
               inertia=dict(diagonal=[.0000015,.000010,.000011]),shapes=fork)
    rods=[]
    for name,position,sign,half_width in (('rod_a',rear,1,.002),('rod_b',pin,-1,.0015)):
        length=ROD/2
        rods.append(dict(name=name,mass=.008,position=position.tolist(),
            center_of_mass=[sign*ROD/4,0,0],
            inertia=dict(diagonal=[.008*2*(2*half_width)**2/12,
                                   .008*(length**2+(2*half_width)**2)/12,
                                   .008*(length**2+(2*half_width)**2)/12]),
            shapes=[box((ROD/4,half_width,half_width),(sign*ROD/4,0,0),
                        (.45,.47,.5,1),collision=False)]))
    rod_a,rod_b=rods
    rod_a['orientation']=rod_b['orientation']=quat(angle)
    joints=[
        dict(name='lever_pivot',type='revolute',parent='housing',child='lever',axis='Y',
             position_parent=PIVOT.tolist(),orientation_child=quat(-OPEN_ANGLE),
             lower=CLOSED_ANGLE-OPEN_ANGLE,upper=0.,damping=.0004),
        dict(name='jaw_slide',type='prismatic',parent='housing',child='jaw',axis='X',
             lower=-.001,upper=.016,stiffness=1200.,target=-.0025,damping=1.),
        dict(name='rear_pin',type='revolute',parent='jaw',child='rod_a',axis='Y',
             position_parent=rear.tolist(),orientation_child=quat(-angle)),
        dict(name='spring_link',type='prismatic',parent='rod_a',child='rod_b',axis='X',
             position_parent=[ROD,0,0],lower=-.003,upper=.003,
             stiffness=20000.,damping=5.,target=.0003),
        dict(name='crank_pin',type='revolute',parent='lever',child='rod_b',axis='Y',
             position_parent=[CRANK,0,0],orientation_parent=quat(-OPEN_ANGLE),
             orientation_child=quat(-angle))]
    buttons=[]
    for sign,length,label,x in ((1,.008,'short',-.070),(-1,.014,'long',-.065)):
        name='button_'+label
        position=[x,sign*.015,.003]
        buttons.append(body(name,[
            dict(type='capsule',size=[.0025],fromto=[0,0,0,0,sign*.010,.010],
                 color=[.25,.26,.28,1],friction=[.6,.4]),
            dict(type='capsule',size=[.0025],fromto=[0,0,0,0,-sign*length,0],
                 color=[.45,.47,.50,1],friction=[.4,.3])],.005,position))
        joints.append(dict(name=name+'_pivot',type='revolute',parent='housing',child=name,
             axis='X',position_parent=position,lower=min(0,sign*math.pi/4),
             upper=max(0,sign*math.pi/4),stiffness=.015,damping=.001,target=0.))
    return description('Handle',[housing,lever,jaw,rod_a,rod_b,*buttons],joints=joints,
                       solver_position_iterations=64,solver_velocity_iterations=16)


def geometry():
    assets=dict(pan=pan(),handle=handle(),fixture=description('Trivet',[
        body('trivet',[dict(type='cylinder',size=[.12,.006],position=[0,0,.006],
              color=[.48,.30,.16,1],friction=[.8,.6])],static=True)]))
    # Open handle rests on its side; no pedestal or external restraint.
    poses=dict(pan=PAN_START,handle=(.19,-.06,TABLE+.0194073),fixture=TRIVET)
    return assets,poses
