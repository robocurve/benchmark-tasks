"""A loose overhand in a free, full-size polyester cord."""
import json
from pathlib import Path
from rope import rope
SCENE='station'
NODES=json.loads(Path(__file__).with_name('initial-nodes.json').read_text())
CORD=rope('cord',nodes=NODES,radius=.005,mass=.0803,friction=.8,bend_stiffness=.000390625,bend_damping=.0000390625,linear_damping=.5,angular_damping=2.,contact_offset=.0005,solver_position_iterations=64,solver_velocity_iterations=16,color=(.1,.38,.85),start_color=(.9,.7,.05),end_color=(.85,.12,.1))
OBJECTS=[CORD]
def success(objects):return False
