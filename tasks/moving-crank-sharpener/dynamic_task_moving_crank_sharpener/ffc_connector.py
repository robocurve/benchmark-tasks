"""Nominal geometry and native joints of the validated flat-flex connector."""

import math

from yam_arena.task_file import Box

from dynamic_task_moving_crank_sharpener.geometry import Solid


SLOT_DEPTH = .00335
SLIDER_TRAVEL = .00160


def connector(board, mouth_x, top_z):
    """Arrange the housing, wedge, contact leaf and guide at the board mouth."""
    housing = Box('housing',pos=(mouth_x+.00275,0.,top_z+.00135),size=(.0055,.0208,.0027),
                  mass=.00024,color=(.35,.22,.13))
    guide = Box('cam_guide',pos=(mouth_x+.0018,0.,top_z+.00015),size=(.0084,.0158,.00022),
                mass=.00014,color=housing.color)
    wedge = Box('slide_wedge',pos=(mouth_x-.00060,0.,top_z+.00082),size=(.0054,.0208,.002),
                mass=.00012,color=(.19,.22,.27))
    follower = Box('contact_follower',pos=(mouth_x+.00115,0.,top_z+.00116),size=(.0022,.0156,.0002),
                   mass=.00008,color=(.60,.53,.40))
    leaf = Box('upper_contact_leaf',pos=(mouth_x+.00185,0.,top_z+.001755),size=(.0029,.0147,.00018),
               mass=.00008,color=(.62,.53,.38))

    def housing_part(size, x, y, z):
        return Solid('box',size,(x-.00275,y,z-.00135))

    solids = {
        housing.name:(
            housing_part((.0055,.0208,.00026),.00275,0.,.00013),
            housing_part((.0055,.0208,.00077),.00275,0.,.002315),
            *(housing_part((.0055,.00225,.00158),.00275,y,.00115) for y in (-.009275,.009275)),
            housing_part((.00034,.01630,.00043),SLOT_DEPTH+.00017,0.,.00149),
        ),
        wedge.name:(
            # The short high end clears the stiffener before the slider closes.
            Solid('box',(.0022,.0155,.00016),(.0010,0.,-.00003),pitch=math.radians(24)),
            *(Solid('box',(.0033,.00160,.00125),(0.,y,.00067)) for y in (-.00955,.00955)),
            *(Solid('box',(.0037,.0011,.000085),(.0010,y,-.00049)) for y in (-.0066,.0066)),
        ),
        follower.name:(Solid('box',(.0022,.0156,.00018)),Solid('sphere',(.00022,),(0.,0.,-.00019))),
    }
    joints = {
        housing.name:{'kind':'fixed','parent':board.name,'anchor':housing.pos},
        guide.name:{'kind':'fixed','parent':housing.name,'anchor':guide.pos},
        wedge.name:{'kind':'prismatic','parent':housing.name,'anchor':wedge.pos,'axis':'X',
                    'lower':0.,'upper':SLIDER_TRAVEL,'drive':{'stiffness':12000.,'damping':3.,'max_force':2.5}},
        follower.name:{'kind':'prismatic','parent':housing.name,'anchor':follower.pos,'axis':'Z',
                       'lower':-.00002,'upper':.00046,'drive':{'stiffness':520.,'damping':.4,'max_force':.25}},
        leaf.name:{'kind':'prismatic','parent':housing.name,'anchor':leaf.pos,'axis':'Z',
                   'lower':0.,'upper':.00021,'drive':{'stiffness':9000.,'damping':.5,'max_force':1.8}},
    }
    return (housing,guide,wedge,follower,leaf),solids,joints
