"""Free the screwdriver and hold it over the green marker with the left gripper 15 cm above the tabletop, without dropping it or moving the clamp base."""
from pathlib import Path
from resist_isaac.clearance_recovery import ClearanceRecovery, export_assets
from .grasp import NativeLeftGrasp

CONDITIONS=("recoverable",)

def _enable_tool_reports(path):
    from pxr import PhysxSchema, Usd, UsdPhysics
    stage = Usd.Stage.Open(str(path))
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies) != 1:
        raise RuntimeError('Expected one screwdriver body')
    PhysxSchema.PhysxContactReportAPI.Apply(bodies[0]).CreateThresholdAttr(0)
    stage.GetRootLayer().Save()


class SpringClampRecovery(ClearanceRecovery):
    def __init__(self):
        super().__init__(Path(__file__).parent)
        self._grasp = None
        self._evidence = None
        self._contact_setting = None
        self._base = None

    def build_scene(self, embodiment):
        result = super().build_scene(embodiment)
        _enable_tool_reports(Path(self._temporary.name) / 'tool.usda')
        return result

    def set_evidence_directory(self, directory):
        self._evidence = Path(directory)

    def begin_episode(self, base):
        self.close()
        self._base = base
        self._contact_setting = base.sim.get_setting('/physics/disableContactProcessing')
        base.sim.set_setting('/physics/disableContactProcessing', False)
        if bool(base.sim.get_setting('/physics/disableContactProcessing')):
            raise RuntimeError('Native contact processing remains disabled')
        self._grasp = NativeLeftGrasp(base, self._evidence)

    def left_grasp_verified(self, base):
        return self._grasp is not None and self._grasp.consume()

    def close(self):
        if self._grasp is not None:
            self._grasp.close()
            self._grasp = None
        if self._contact_setting is not None:
            self._base.sim.set_setting('/physics/disableContactProcessing', self._contact_setting)
            self._contact_setting = None

def make_task(condition):
    if condition not in CONDITIONS: raise ValueError(condition)
    return SpringClampRecovery()

def create_assets(directory,condition="recoverable"):
    if condition not in CONDITIONS: raise ValueError(condition)
    result = export_assets(Path(__file__).parent,directory)
    _enable_tool_reports(result['tool'])
    return result
