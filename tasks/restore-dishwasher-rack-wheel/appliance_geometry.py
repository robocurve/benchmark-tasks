"""Installed countertop dishwasher, open door and original rolling basket."""
import numpy as np

from yam_arena.station import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

import mechkit as m
import wheel_geometry as w


FRONT = np.array((.410, .020, TABLE_TOP))
RAIL_Z = .073
RAIL_Y = .220
AXLE_X = -.145
AXLE_Y = RAIL_Y - w.SEATED_Y
AXLE_Z = .0005
REAR_AXLE_X = .145
RACK_START = np.array((-.080, 0., RAIL_Z + w.RADIUS))
FRONT_GRIP = np.array((-.183, -.090, .095))
ROLLER_START = FRONT + RACK_START + (AXLE_X, RAIL_Y + .00365, 0.)
PICKUP_ROTATION = np.diag((1.,-1.,-1.))
INSERT_PITCH = np.deg2rad(20.)
INSERT_APPROACH = np.array((0.,-np.cos(INSERT_PITCH),-np.sin(INSERT_PITCH)))
INSERT_JAW = np.array((1.,0.,0.))
INSERT_ROTATION = np.column_stack(
    (INSERT_JAW,np.cross(INSERT_APPROACH,INSERT_JAW),INSERT_APPROACH))

LOOKS = {**w.LOOKS,
         'case': m.Look(m.rgb(.92, .91, .86), roughness=.53),
         'liner': m.Look(m.rgb(.67, .71, .72), metallic=.35, roughness=.46),
         'wire': m.Look(m.rgb(.66, .70, .69), roughness=.46),
         'seal': m.Look(m.rgb(.20, .25, .24), roughness=.63)}


def solids(rows):
    parts = tuple((name, m.box(size, at), look) for name, size, at, look in rows)
    colliders = tuple(m.col_box(name + '_contact', size, at) for name, size, at, _ in rows)
    return parts, colliders


def dishwasher():
    width, depth, height = .5499, .5001, .4379
    door = .360
    shell = [
        ('base', (depth,width,.061), (.5*depth,0.,.0305), 'case'),
        ('roof', (depth,width,.039), (.5*depth,0.,height-.0195), 'case'),
        ('control_fascia', (.018,width,.035), (.009,0.,height-.049), 'case'),
        ('back', (.033,width,.345), (depth-.0165,0.,.230), 'liner'),
        ('door_front', (door,width,.038), (-.5*door,0.,RAIL_Z-.033), 'case'),
        ('door_liner', (door-.014,width-.028,.011), (-.5*door,0.,RAIL_Z-.0105), 'liner'),
    ]
    for side, sign in (('left', 1.), ('right', -1.)):
        lip_y = RAIL_Y + (.0105 if sign > 0 else .009)
        shell.extend([
            (side+'_cabinet_wall', (depth,.025,.365), (.5*depth,sign*(width-.025)/2.,.235), 'case'),
            (side+'_bowl_rail', (.443,.028,.012), (.2225,sign*RAIL_Y,RAIL_Z-.006), 'liner'),
            (side+'_door_rail', (.326,.028,.009), (-.164,sign*RAIL_Y,RAIL_Z-.0045), 'liner'),
            (side+'_rail_lip', (.447,.004,.010), (.225,sign*lip_y,RAIL_Z+.002), 'liner'),
            (side+'_door_lip', (.326,.004,.010), (-.164,sign*lip_y,RAIL_Z+.002), 'liner'),
        ])
    shell_parts, shell_colliders = solids(shell)
    panel = m.box((.220,.048,.004), (-.246,0.,RAIL_Z-.0048))
    handle = m.box((.044,.155,.009), (-.333,0.,RAIL_Z-.057))
    details = [('detergent_dispenser_outline', panel, 'seal'), ('recessed_door_handle', handle, 'seal')]
    for i in range(5):
        button = m.box((.001,.013,.008), (-.0006, .106 + .021*i, height-.049))
        details.append((f'control_button_{i}', button, 'seal'))
    for x in (-.037,.038):
        for y in (-RAIL_Y,RAIL_Y):
            rail_mark = m.box((.003,.011,.0004), (x,y+.007,RAIL_Z+.0002))
            details.append((f'rail_mark_{len(details)}', rail_mark, 'witness'))
    return m.fixed('open_dishwasher', FRONT, shell_parts+tuple(details), shell_colliders, LOOKS, friction=(.68,.51))


def basket():
    rows = [
        ('front_grip', (.006,.402,.006), (-.183,0.,.095), 'wire'),
        ('front_lower_rim', (.387,.006,.006), (0.,.201,.022), 'wire'),
        ('rear_lower_rim', (.387,.006,.006), (0.,-.201,.022), 'wire'),
        ('back_pull_bar', (.006,.402,.007), (.183,0.,.095), 'wire'),
        ('front_crossbar', (.006,.402,.006), (-.183,0.,.021), 'wire'),
        ('back_crossbar', (.006,.402,.006), (.183,0.,.021), 'wire'),
    ]
    for sign, side in ((-1.,'right'),(1.,'left')):
        rows.append((side+'_top_rim', (.369,.006,.006), (0.,sign*.201,.095), 'wire'))
        for end, xpos in (('front',AXLE_X),('rear',REAR_AXLE_X)):
            rows.append((side+'_'+end+'_mount', (.011,.014,.040), (xpos,sign*.198,.020), 'wire'))
    visual, contact = solids(rows)
    axle = w.axle_parts((AXLE_X,AXLE_Y,AXLE_Z))
    wire = []
    for x in np.linspace(-.170,.170,13):
        row = m.tube(((x,-.201,.090),(x,-.201,.025),(x,.201,.025),(x,.201,.090)), .002)
        wire.append((f'cross_wire_{len(wire)}',row,'wire'))
    for y in np.linspace(-.185,.185,11):
        row = m.tube(((-.177,y,.031),(.177,y,.031)), .002)
        wire.append((f'long_wire_{len(wire)}',row,'wire'))
    for x in np.linspace(-.080,.144,8):
        for y in (-.156,-.064,.065,.151):
            tine = m.tube(((x,y,.032),(x+.013,y,.083)), .002)
            wire.append((f'tine_{len(wire)}',tine,'wire'))
    wheels = []
    for i, (x,y) in enumerate(((AXLE_X,-RAIL_Y),(REAR_AXLE_X,-RAIL_Y),(REAR_AXLE_X,RAIL_Y))):
        at = (x,y,0.)
        mesh = m.cylinder(w.RADIUS, 2.*w.TYRE_HALF_WIDTH, at, axis='y', segments=40)
        parts = ((f'original_wheel_{i}',mesh,'nylon'),)
        wheels.append(m.hinge(f'wheel_{i}', 'Y', at, -1440., 1440., parts, w.contacts(parts),
                              damping=.0004, friction=.0004, effort=.06, mass=.012,
                              armature=1.e-6, gravity=True))
    return m.mechanism('basket',FRONT+RACK_START,wheels,LOOKS,
                       visual+axle+tuple(wire),contact+w.contacts(axle),free_mass=.79,
                       frame_com=(.019,-.010,.060), frame_inertia=(.016,.014,.029),
                       friction=(.64,.46))


TOOL_START = np.array((.180,.340,TABLE_TOP+.020))
TOOL_HEAD = np.array((0.,0.,.028))
TOOL_FACE = -.026


def push_block():
    sizes = (
        ("press_face", (.031,.011,.020), (0.,-.0205,.004)),
        ("press_body", (.026,.035,.017), (0.,.002,.003)),
        ("upright_grip", (.028,.029,.039), tuple(TOOL_HEAD)),
        ("press_foot", (.040,.032,.009), (0.,.018,-.0155)),
    )
    visual = tuple((n,m.box(size,at),"wood") for n,size,at in sizes)
    solid = tuple(m.col_box(n+"_c",size,at) for n,size,at in sizes)
    looks = {**LOOKS, "wood":m.Look(m.rgb(.56,.34,.17),roughness=.67)}
    return m.free("press_block",TOOL_START,visual,solid,looks,.047,
                  friction=(.44,.34), com=(0.,.017,.006))


OBJECTS = (dishwasher(),basket(),w.roller(ROLLER_START),push_block())
