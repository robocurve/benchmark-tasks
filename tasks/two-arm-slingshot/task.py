"""Two arms playing Angry Birds: brace, draw and release a native toy slingshot."""
import importlib.util
from pathlib import Path

from resist_isaac.clearance_recovery import ClearanceRecovery, export_assets

ROOT=Path(__file__).parent
spec=importlib.util.spec_from_file_location('slingshot_scoring',ROOT/'scoring.py')
scoring=importlib.util.module_from_spec(spec)
spec.loader.exec_module(scoring)
CONDITIONS=('recoverable',)


class Slingshot(ClearanceRecovery):
    def __init__(self):
        super().__init__(ROOT)
        self.rubric=scoring.SlingshotScore(self.cfg)
        self.bands=None

    def reset(self,base):
        super().reset(base)
        self.rubric=scoring.SlingshotScore(self.cfg)
        self.draw_bands(base,self.states(base))

    def evaluate(self,base):
        from yam_arena.runtime import tcp_world
        from yam_arena.constants import FINGER_JOINTS,FINGER_OPEN
        hands=[];grips=[]
        for side in ('left','right'):
            robot=base.scene[side+'_robot']
            hands.append(tcp_world(robot))
            indices,_=robot.find_joints(FINGER_JOINTS)
            grips.append(float(robot.data.joint_pos[0,indices].mean().item()/FINGER_OPEN))
        states=self.states(base)
        self.draw_bands(base,states)
        return self.rubric.evaluate(states,hands,grips)

    def draw_bands(self,base,states):
        from pxr import Gf,UsdGeom
        # Graphics only: actual body poses define the two visible elastic bands.
        if self.bands is None:
            self.bands=UsdGeom.BasisCurves.Define(base.sim.stage,'/World/slingshot_bands')
            self.bands.CreateTypeAttr('linear')
            self.bands.CreateCurveVertexCountsAttr([2,2])
            self.bands.CreateWidthsAttr([.006])
            self.bands.SetWidthsInterpolation('constant')
            self.bands.CreateDisplayColorAttr([Gf.Vec3f(.85,.63,.18)])
        fp,fr,_=states['launcher/frame'];pp,pr,_=states['launcher/pouch']
        points=[]
        for sign in (-1,1):
            points.extend((fp+fr@[0,sign*.070,.13],pp+pr@[-.03,sign*.030,0]))
        self.bands.GetPointsAttr().Set([Gf.Vec3f(*map(float,p)) for p in points])


def make_task(condition):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return Slingshot()


def create_assets(directory,condition='recoverable'):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return export_assets(ROOT,directory)
