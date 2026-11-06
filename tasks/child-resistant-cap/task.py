"""On the station table, in front of the arms: an amber pharmacy vial with a printed pharmacy label, standing free
(4.6 cm across, 10.6 cm tall with its cap). Its white child-resistant cap (5.1 cm across) is printed PUSH DOWN & TURN
round an anticlockwise arrow.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0

INSTRUCTION = "Take the cap off the medicine bottle."

TOP = g.TOP

# -- objects --------------------------------------------------------------------------------------------------------
BOTTLE = UsdBody("pill_bottle", usd="assets/pill_bottle.usda", pos=(g.BOTTLE_X, g.BOTTLE_Y, TOP),
                 size=(2 * g.CAP_R, 2 * g.CAP_R, g.CAP_Z0 + g.CAP_H), articulation=True,
                 joint_pos=(("lift", 0.0), ("spin", 0.0), ("press", 0.0)),
                 drives=(("lift", 0.0, 0.5), ("spin", 0.0, g.SPIN_DAMPING), ("press", g.SPRING, 2.0)),
                 friction=(0.5, 0.4))
OBJECTS = [BOTTLE]

# -- monitor: the push-and-turn lock --------------------------------------------------------------------------------
_STATE: dict = {}


def _write(environment, joint: str, *, limits=None, damping=None) -> None:
    """Change one of the vial's joints in the simulation: its position limits or its drive's damping."""
    import torch

    asset = environment.unwrapped.scene["pill_bottle"]
    ids = asset.find_joints([joint])[0]
    if limits is not None:
        asset.write_joint_position_limit_to_sim_index(
            limits=torch.tensor([[list(limits)]], dtype=torch.float32, device=asset.device), joint_ids=ids,
            warn_limit_violation=False)
    if damping is not None:
        asset.write_joint_damping_to_sim_index(
            damping=torch.tensor([[damping]], dtype=torch.float32, device=asset.device), joint_ids=ids)


def _lock(environment) -> None:
    """The cap screwed on: it can't lift, and it spins freely until pressed."""
    _write(environment, "lift", limits=(0.0, 0.0))
    _write(environment, "spin", damping=g.SPIN_DAMPING)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(unscrewed=0.0, last_spin=0.0, pressed=False, depth=0.0, free=False, off=False, lift=0.0, tilt=0.0,
                  tipped=False, dragging=False)
    if environment is not None:
        _lock(environment)


@before_reset
def _restart_cap(environment) -> None:
    """Screw the cap back on before a reset (the reset moves the vial only by its root, not its joints)."""
    from scenekit import write_joint_positions

    _lock(environment)
    write_joint_positions(environment, "pill_bottle", {"lift": 0.0, "spin": 0.0, "press": 0.0})


_reset_state()


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    state = object_states(environment, ["pill_bottle"])["pill_bottle"]
    joints = state.joints
    spin = float(joints["spin"])
    turned = math.remainder(spin - _STATE["last_spin"], 2 * math.pi)     # anticlockwise from above is positive
    _STATE["last_spin"] = spin
    _STATE["depth"] = -float(joints["press"])
    _STATE["pressed"] = pressed = _STATE["depth"] > g.PRESSED
    _STATE["lift"] = float(joints["lift"])
    seated = _STATE["lift"] < g.SEATED
    if seated and not _STATE["off"]:
        # on the thread: anticlockwise unscrews only while pressed (else the ratchet slips); clockwise always tightens
        if pressed or turned < 0:
            _STATE["unscrewed"] = min(g.FREE_TURN, max(0.0, _STATE["unscrewed"] + turned))
        free = _STATE["unscrewed"] >= g.FREE_TURN
        if free != _STATE["free"]:
            _STATE["free"] = free
            _write(environment, "lift", limits=(0.0, g.LIFT_MAX if free else 0.0))
    dragging = pressed and seated and not _STATE["free"]
    if dragging != _STATE["dragging"]:                                      # pressed, the thread drags on the turn
        _STATE["dragging"] = dragging
        _write(environment, "spin", damping=g.THREAD_DAMPING if dragging else g.SPIN_DAMPING)
    if _STATE["free"] and not _STATE["off"] and _STATE["lift"] >= g.OFF:
        _STATE["off"] = True                                                # clear of the neck: it stays off
        _write(environment, "lift", limits=(g.OFF, g.LIFT_MAX))
    R = state.rotation()
    _STATE["tilt"] = math.acos(max(-1.0, min(1.0, R[2, 2])))
    _STATE["tipped"] = _STATE["tipped"] or _STATE["tilt"] >= g.TIPPED       # knocked over at any time


def success(objects) -> bool:
    """The cap is off (lifted clear of the neck) and the vial was never knocked over; held 2 s."""
    return bool(_STATE["off"] and not _STATE["tipped"])


def score(objects) -> float:
    """Partial credit (success scores 1.0): 0.5 when the cap is unscrewed (it would lift straight off) but still on
    the neck; 0 if the vial was ever knocked over (tilted past 45 degrees), or the cap is still screwed on."""
    if _STATE["tipped"]:
        return 0.0
    if _STATE["off"]:
        return 1.0
    return 0.5 if _STATE["free"] else 0.0
