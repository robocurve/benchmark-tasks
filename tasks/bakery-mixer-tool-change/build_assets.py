"""Original generic commercial mixer; metre-scale CAD, native physical clearances."""
import json
import math
from pathlib import Path

P = Path(__file__).parent
STEEL = [.58, .61, .64, 1]
PAINT = [.72, .73, .70, 1]
DARK = [.035, .04, .045, 1]


def box(size, pos, color=STEEL, **kw):
    return dict(type='box', size=[v/2 for v in size], position=pos, color=color, **kw)


def cyl(radius, length, pos, color=STEEL, **kw):
    return dict(type='cylinder', size=[radius, length/2], position=pos, color=color, **kw)


def rod(a, b, radius=.003, color=STEEL, **kw):
    return dict(type='capsule', size=[radius], fromto=[*a, *b], color=color, **kw)


def sector(ri, ro, z0, z1, a0, a1, color=STEEL, **kw):
    # Each narrow sector has its own convex collider; an annular hole stays open.
    pts = [[r*math.cos(a), r*math.sin(a), z] for z in [z0, z1]
           for r in [ri, ro] for a in [a0, a1]]
    return dict(type='mesh', mesh=dict(points=pts, faces=[[0,1,3,2],[4,6,7,5],
                [0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]), color=color,
                collision=True, collision_approximation='convexHull', **kw)


def annulus(ri, ro, z0, z1, n=64, omit=None, **kw):
    return [sector(ri, ro, z0, z1, i*2*math.pi/n, (i+1)*2*math.pi/n, **kw)
            for i in range(n) if not omit or not omit((i+.5)*2*math.pi/n)]


def body(name, shapes, mass=1., pos=(0,0,0), inertia=None, com=(0,0,0)):
    return dict(name=name, shapes=shapes, position=list(pos), mass=mass,
                center_of_mass=list(com),
                inertia={'diagonal': inertia or [.01, .01, .01]})


def main():
    # World position of the fixed chassis; front is -X, operator at the near edge.
    root = [.35, .03, .7475]
    spindle = [-.07, 0., .330]
    orbit = .020
    bowl_center = [-.09, 0., .070]
    bowl_height = .140
    lift = .075
    chassis = [box([.31,.28,.028],[-.005,0,.014],PAINT),
               box([.085,.13,.33],[.105,0,.19],PAINT),
               box([.25,.19,.11],[.015,0,.420],PAINT),
               box([.012,.055,.23],[.055,0,.18],STEEL)]
    # Clearance for the orbiting shaft through the underside bearing cover.
    for shape in annulus(.030,.065,.359,.371,n=32,color=DARK):
        for point in shape['mesh']['points']:point[0]-=.09
        chassis.append(shape)
    for x in [-.12,.12]:
        for y in [-.105,.105]:chassis.append(cyl(.015,.010,[x,y,.005],DARK))
    # Gear selector and capped attachment hub are prepared, not task controls.
    chassis += [rod([-.12,-.103,.38],[-.15,-.103,.35],.006,DARK),
                cyl(.025,.020,[-.120,0,.395],STEEL,orientation=[.70710678,0,.70710678,0])]
    carrier = [cyl(.030,.012,[0,0,.028],STEEL)]
    # Machined lead-in chamfer guides the close-clearance bore onto the shaft.
    tip_points=[[r*math.cos(i*math.tau/24),r*math.sin(i*math.tau/24),z]
                for z,r in [(-.010,.003),(-.005,.006)] for i in range(24)]
    tip_faces=[list(reversed(range(24))),list(range(24,48))]+[
        [i,(i+1)%24,(i+1)%24+24,i+24] for i in range(24)]
    shaft = [cyl(.006,.037,[0,0,.0135],STEEL,contact_offset=.00002),
             dict(type='mesh',mesh=dict(points=tip_points,faces=tip_faces),color=STEEL,
                  collision=True,collision_approximation='convexHull',contact_offset=.00002),
             rod([0,.005,0],[0,.010,0],.002,STEEL,contact_offset=.00002)]
    collar=annulus(.009,.016,-.00075,.00075,n=32,color=STEEL,contact_offset=.00002,friction=[.3,.2])
    # Bowl is already correctly clamped to its support for this tool-change task.
    # Original revolved wall approximated by short convex quadrilateral prisms.
    bowl = [cyl(.070,.002,[0,0,-.001],STEEL)]
    profile = [(0.,.070),(.025,.080),(.065,.098),(.105,.110),(.140,.112)]
    for (z0,r0),(z1,r1) in zip(profile,profile[1:]):
        for i in range(64):
            a0=i*math.tau/64;a1=(i+1)*math.tau/64
            pts=[[r*math.cos(a),r*math.sin(a),z] for z,radius in [(z0,r0),(z1,r1)]
                 for r in [radius,radius+.002] for a in [a0,a1]]
            bowl.append(dict(type='mesh',mesh=dict(points=pts,faces=[[0,1,3,2],[4,6,7,5],
                [0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]),color=STEEL,
                collision=True,collision_approximation='convexHull',contact_offset=.00005))
    bowl += annulus(.110,.116,.138,.142,omit=lambda a: min(a,math.tau-a)<.08)
    for y in [-.129,.129]:
        bowl += [box([.045,.025,.008],[.015,y,.107]),
                 box([.055,.020,.012],[.025,y,.095],PAINT),
                 cyl(.004,.020,[.015,y,.107]),box([.035,.012,.006],[.015,y,.119],DARK),
                 box([.130,.020,.012],[.075,y,.080],PAINT),
                 box([.020,.020,.025],[.015,y,.0935],PAINT)]
    # The lifting cradle wraps around the bowl, outside the mixing cavity.
    bowl += [box([.018,.255,.018],[.130,0,.08],PAINT),
             box([.015,.065,.050],[.145,0,.08],PAINT)]
    handle_pos=[.105,-.140,.180]
    handle=[rod([0,0,0],[-.100,0,0],.006),cyl(.012,.048,[-.100,0,0],DARK,
            orientation=[.70710678,.70710678,0,0])]
    button_xy=[[-.045,-.062],[-.045,.062]]
    buttons=[body(name,[box([.008,.028,.028],[0,0,0],color)],.010,
                 [-.115,y,.415],[.000001]*3)
             for name,(x,y),color in zip(['start','stop'],button_xy,[[.08,.48,.15,1],[.65,.07,.04,1]])]
    parts=[body('chassis',chassis,15.),body('carrier',carrier,.35,[-.09,0,.330],[.0002]*3,com=[0,0,.028]),
           body('shaft',shaft,.12,spindle,[.00002]*3),
           body('retaining_collar',collar,.020,[spindle[0],spindle[1],spindle[2]+.01275],[.000002,.000002,.000003]),
           body('bowl',bowl,1.2,bowl_center,[.008,.008,.012],com=[.015,0,.080]),
           body('lift_handle',handle,.12,handle_pos,[.0002,.0006,.0006]),*buttons]
    joints=[dict(name='orbit',type='revolute',parent='chassis',child='carrier',axis='Z',position_parent=[-.09,0,.330],static_friction_effort=.04,stiffness=0.,damping=.4),
            dict(name='spin',type='revolute',parent='carrier',child='shaft',axis='Z',position_parent=[orbit,0,0],static_friction_effort=.02,stiffness=0.,damping=.4),
            dict(name='retaining_spring',type='prismatic',parent='shaft',child='retaining_collar',axis='Z',
                 position_parent=[0,0,.01275],lower=0.,upper=.008,stiffness=400.,damping=4.,target=-.0125),
            dict(name='bowl_lift',type='prismatic',parent='chassis',child='bowl',axis='Z',
                 position_parent=bowl_center,lower=0,upper=lift,damping=2),
            dict(name='handle_hinge',type='revolute',parent='chassis',child='lift_handle',axis='Y',
                 position_parent=handle_pos,lower=0,upper=math.pi/2,
                 damping=.05,static_friction_effort=1.0)]
    for part in buttons:
        joints.append(dict(name=part['name']+'_button',type='prismatic',parent='chassis',child=part['name'],
                     axis='X',position_parent=part['position'],lower=0,upper=.004,stiffness=300,damping=1,target=-.001))
    mixer=dict(name='Mixer',bodies=parts,joints=joints,articulation=True,fixed_base='chassis',
               solver_position_iterations=64,solver_velocity_iterations=16)

    # Bayonet hub: open entry at +Y, turn clockwise 60 degrees, release into pocket.
    hub=[];n=96
    for i in range(n):
        angle=(i+.5)*math.tau/n;deg=math.degrees(angle)
        layers=[(-.020,-.0035)]
        if not 65<deg<115:
            ceiling=.008 if 125<deg<175 else .0035
            if 115<=deg<175:layers.append((ceiling,.020))
            else:layers.append((-.0035,.020))
        for z0,z1 in layers:hub.append(sector(.00635,.012,z0,z1,i*math.tau/n,(i+1)*math.tau/n,
                                          contact_offset=.00002,friction=[.4,.25]))
    beater=hub+[box([.022,.014,.030],[0,0,-.035],STEEL,friction=[1.,.8]),
                rod([0,0,-.050],[0,0,-.170],.0045),
                rod([0,0,-.055],[0,-.050,-.105],.0045),
                rod([0,-.050,-.105],[0,-.035,-.170],.0045),
                rod([0,-.035,-.170],[0,.035,-.170],.0045),
                rod([0,.035,-.170],[0,.050,-.105],.0045),
                rod([0,.050,-.105],[0,0,-.055],.0045),
                rod([0,-.050,-.105],[0,.050,-.105],.0045)]
    # Removable cage has a rear slot around the mixer neck. Three feet locate on rim.
    guard=[]
    for radius,z in [(.115,0),(.095,.015),(.070,.032),(.038,.045)]:
        for i in range(64):
            a0=i*math.tau/64;a1=(i+1)*math.tau/64
            if min((a0+a1)/2,math.tau-(a0+a1)/2)<.55:continue
            guard.append(rod([radius*math.cos(a0),radius*math.sin(a0),z],
                             [radius*math.cos(a1),radius*math.sin(a1),z],.0025))
    for i in range(1,16):
        a=i*math.tau/16
        if min(a,math.tau-a)<.55:continue
        guard.append(rod([.115*math.cos(a),.115*math.sin(a),0],[.038*math.cos(a),.038*math.sin(a),.045],.0025))
    guard += [box([.018,.024,.008],[0,.131,.028],DARK,friction=[.9,.7]),
              rod([0,.131,.028],[0,.112,0],.003)]
    for a in [math.pi/2,math.pi,3*math.pi/2]:
        guard.append(rod([.107*math.cos(a),.107*math.sin(a),-.012],
                         [.107*math.cos(a),.107*math.sin(a),0],.002))
    guard += [box([.010,.008,.005],[.112/math.sqrt(2),.112/math.sqrt(2),-.001],DARK,
                  orientation=[math.cos(math.pi/8),0,0,math.sin(math.pi/8)])]
    # Draining supports are ordinary fixed racks, not grasp attachments.
    source=[box([.13,.13,.015],[.24,.32,.755],PAINT),
            box([.018,.13,.045],[.24,.32,.785],PAINT)]
    for x in [.225,.255]:source.append(box([.008,.095,.025],[x,.32,.815],DARK))
    source += [box([.26,.25,.020],[.45,.31,.7575],PAINT)]
    geo={'mixer':mixer,'beater':dict(name='Beater',bodies=[body('beater',beater,.25,inertia=[.0011,.0009,.0003],com=[0,0,-.080])],
                solver_position_iterations=64,solver_velocity_iterations=16),
         'guard':dict(name='Guard',bodies=[body('guard',guard,.18,inertia=[.001,.001,.002],com=[0,0,.018])],
                solver_position_iterations=64,solver_velocity_iterations=16),
         'rack':dict(name='Rack',bodies=[dict(name='rack',static=True,shapes=source)])}
    cfg=dict(kind='mixer_tool_change',instruction='Fit the clean flat beater to the stopped bakery mixer: align its slotted hub, push it onto the shaft against the spring-loaded collar, turn it into the retaining pocket and release it. Seat the guard on the lowered bowl, raise the bowl with the lift handle, then run the mixer at its preset low speed for at least five seconds and switch it off. Leave both hands clear, the beater secured and the guard fitted.',
             max_steps=3600,control_hz=20,table_top=.7475,hold_s=2.,
             poses=dict(mixer=root,beater=[.24,.32,.987],guard=[.45,.31,.7825],rack=[0,0,0]),
             lift_travel=lift,lift_angle=math.pi/2,handle_knob_offset=[-.100,0,0],hub_insert_offset=-.001,hub_rest_offset=-.006,hub_yaw=-math.pi/3,
             bowl_height=bowl_height,guard_height=.1445,run_seconds=5.,shaft_rpm=136.,orbit_rpm=-60.,
             button_rest_x=root[0]-.115)
    for name,data in [('geometry.json',geo),('pilot.json',cfg)]:
        (P/name).write_text(json.dumps(data,indent=2)+'\n')
    print('Original mixer geometry written:',sum(len(b['shapes']) for d in geo.values() for b in d['bodies']),'shapes')


if __name__=='__main__':main()
