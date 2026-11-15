"""Peg a flexible cotton T-shirt onto a clothesline with two passive spring pegs.

Development scene. The full-score controller and final evidence are not yet
available. Neither fixture nor peg ever attaches to cloth in software.
"""
import os
from arena_cloth import Cloth
from arena_props import SpringPeg,Fixture
from cloth_mesh import tshirt
from native_cloth import Fabric
from scene import initial_layout,TABLE,LINE_X,LINE_Z,LINE_HALF_SPAN,SUPPORT_SIZE,SUPPORT_CENTER,SUPPORT_TOP

from line_pegs import positions,BASE_ANGLE,OPEN_ANGLE
PEG_RESTS_ENABLED=False
LINE_PEGS_ENABLED=True
PEG_POSITIONS=positions()

SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=0.
INSTRUCTION='Hang the T-shirt from the clothesline using the two pegs, then let go and leave it secure and clear of the table.'
DEVELOPMENT_ONLY=True
CONTACT_REFINED=os.environ.get('ROBOCURVE_SHIRT_CONTACT_REFINED','0')=='1'
REGULAR_REFINED=os.environ.get('ROBOCURVE_SHIRT_REGULAR_REFINED','0')=='1'
if CONTACT_REFINED and REGULAR_REFINED:raise ValueError('Select one garment mesh')
SOLVER_ITERATIONS=int(os.environ.get('ROBOCURVE_SHIRT_SOLVER_ITERATIONS','32'))
if SOLVER_ITERATIONS not in (32,128):raise ValueError('Unsupported cloth solver diagnostic')
MESH=tshirt()
if REGULAR_REFINED:MESH=tshirt(spacing=.0025)
elif CONTACT_REFINED:
    from refined_cloth import contact_band_mesh
    MESH=contact_band_mesh()
if os.environ.get('ROBOCURVE_SHIRT_PAD_REFINED')=='1':
    if not REGULAR_REFINED:raise ValueError('Pad refinement requires the regular base mesh')
    from refined_cloth import pad_refined_mesh
    MESH=pad_refined_mesh()
INITIAL=initial_layout(MESH)
OBJECTS=[
    Cloth('tshirt',pos=(.345,0.,SUPPORT_TOP+.003),size=(.30,.40,.0016),
          color=(.08,.35,.72),mesh=MESH,initial=INITIAL,fabric=Fabric(solver_iterations=SOLVER_ITERATIONS)),
    Fixture('rack',pos=(LINE_X,0.,(TABLE+LINE_Z)/2),
            size=(.15,2*LINE_HALF_SPAN+.095,LINE_Z-TABLE),kind='clothesline'),
    Fixture('laundry_support',pos=SUPPORT_CENTER,size=SUPPORT_SIZE,kind='laundry_support'),
    SpringPeg('left_peg',pos=PEG_POSITIONS['Left'],size=(.070,.010,.022),base_angle=BASE_ANGLE,open_angle=OPEN_ANGLE),
    SpringPeg('right_peg',pos=PEG_POSITIONS['Right'],size=(.070,.010,.022),base_angle=BASE_ANGLE,open_angle=OPEN_ANGLE),
]


def success(objects):
    # No smoke run can accidentally become a successful competition episode.
    return False
