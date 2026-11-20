"""Documented fixed robot reset pose; never changes task material."""
from pathlib import Path
import json
import numpy as np

START_ACTION = np.asarray(json.loads(Path(__file__).with_name('initial-action.json').read_text())['action14'])

def initialize_robot(emb, base):
    """Place the closed robot fingers at the cap rim during reset, before frame 0."""
    import torch
    from yam_arena.constants import FINGER_OPEN
    for side, offset in [('left', 0), ('right', 7)]:
        robot = base.scene[side+'_robot']
        arm_ids, finger_ids = emb._ids[side+'_robot']
        data = robot.data.joint_pos
        position = getattr(data, 'torch', data).clone()
        position[0, arm_ids] = torch.as_tensor(START_ACTION[offset:offset+6], device=position.device, dtype=position.dtype)
        position[0, finger_ids] = float(START_ACTION[offset+6]*FINGER_OPEN)
        robot.write_joint_state_to_sim(position, torch.zeros_like(position))
        robot.set_joint_position_target(position)
    base.scene.write_data_to_sim()
    base.sim.forward()
    emb._ready = torch.as_tensor(START_ACTION[None, :], device=base.device, dtype=torch.float32)
