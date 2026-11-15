"""Scene adapter; completed trials are judged with scoring.py and JUDGING.md."""
import numpy as np
from arena_panels import PanelPaper
from arena_hardware import StapleHardware
from facet_paper import box_task_mesh
from facet_paper import ShellMaterial
from paper import PaperMesh
from paper import paper_material
from full_scene import SHEET_CENTRES,scene_layout
import runtime_config

SCENE='station'
SCORE_OVER='end'
INSTRUCTION='Remove the staple without tearing either sheet. Leave the complete staple at least 10 mm clear of both sheets.'
EXTERNAL_JUDGING=True
OBJECTS=[]
paper=paper_material(runtime_config.PAPER_PROFILE)
shell_material=ShellMaterial(thickness_m=paper.thickness_m,
    density_kg_m3=paper.areal_mass_kg_m2/paper.thickness_m,
    bend_modulus_pa=paper.youngs_modulus_pa,poissons_ratio=paper.poissons_ratio,
    friction=paper.friction,
    tensile_strength_per_width_n_m=1500.*paper.areal_mass_kg_m2/.080)
layout=scene_layout(runtime_config.RESET_PROFILE,paper)
for i,xy in enumerate(SHEET_CENTRES):
    mesh=box_task_mesh(i,runtime_config.PANEL_SPACING_M);position=np.array((*xy,layout['paper_centres_z'][i]))
    mesh=PaperMesh(mesh.rest+position,mesh.faces,mesh.holes)
    OBJECTS.append(PanelPaper('paper_'+str(i),pos=position,size=(.21,.148,paper.thickness_m),mesh=mesh,material=shell_material))
OBJECTS.append(StapleHardware('hardware',pos=(.25,-.13,.77),size=(.3,.3,.05)))

def score(objects):return 0.
def success(objects):return False
