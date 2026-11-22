"""The microwave's joints (assets/microwave.usdc, NVIDIA USDCraft-10K ds_R001_06, a push-button-release oven): door,
release button and timer dial.

The file's own drives are free (stiffness 0) with almost no damping, so a push would fling the door open or shut and a
touch would spin the dial. This sets, per joint, a damping that makes them move like a hand-worked door and dial, a
spring on the button, a catch that holds the door shut once it is within a few degrees of closed, and the release: a
button pressed 2 mm lets the catch go and a spring swings the door open.

Joint angles: the door is 0 shut, positive opening, up to 95 degrees. The dial's axis points into the oven, so a
positive angle turns it clockwise seen from the front. The button is 0 out, 4 mm pressed.
"""

from __future__ import annotations

import math

DOOR, BUTTON, TIMER = "door_hinge", "button_travel", "dial_spin"
NAMES = (DOOR, BUTTON, TIMER)
# N m s/rad for the revolute joints, N s/m for the button: the door shuts in about a second under a 3 N push 15 cm from
# its hinge; the dial turns under a light pinch
DAMPING = {DOOR: 0.25, BUTTON: 4.0, TIMER: 0.02}
BUTTON_K = 300.0  # N/m: pressed its 4 mm with about 1.2 N
BUTTON_PRESSED = 0.002  # m of travel that releases the catch
LATCH_N_M_PER_RAD = 2.0  # the catch: holds the door against a 0.1 N m draught
CATCH_ON = math.radians(5.0)  # within this of shut the catch takes it
OPEN_K = 4.0  # N m/rad of the release spring, towards OPEN_TO
OPEN_TO = math.radians(95.0)  # the door's stop
OPEN_DONE = math.radians(88.0)  # the spring lets go here and the hinge's friction holds the door

# (static, dynamic) joint friction effort in N m: a hand-worked door (about 1 N at 20 cm), a free button, and the model's
# own values for the dial
FRICTION = ((DOOR, 1.00, 0.80), (BUTTON, 0.0, 0.0), (TIMER, 0.030, 0.0225))

_S: dict = {"ids": None, "art": None, "state": "shut", "t_open": 0.0}


def _art(environment):
    art = environment.unwrapped.scene["microwave"]
    if _S["art"] is not art:
        _S["art"] = art
        _S["ids"] = dict(zip(NAMES, art.find_joints(list(NAMES), preserve_order=True)[0]))
    return art


def _q(art):
    q = art.data.joint_pos
    return getattr(q, "torch", q)[0]


def _set(art, name, stiffness=None, target=None):
    import torch

    j = [_S["ids"][name]]
    if stiffness is not None:
        art.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[stiffness]], device=art.device, dtype=torch.float32), joint_ids=j)
    if target is not None:
        art.set_joint_position_target_index(target=torch.tensor([[target]], device=art.device, dtype=torch.float32), joint_ids=j)


def reset(environment, door_start: float = 0.0) -> None:
    """Damping, the button's spring, the door at ``door_start`` (rad) with the catch on when it is shut: the file's own
    start pose is all shut, and the articulation's configured start angle does not always take."""
    import torch

    art = _art(environment)
    ids = [_S["ids"][n] for n in NAMES]
    art.write_joint_position_to_sim_index(position=torch.tensor([[door_start]], device=art.device, dtype=torch.float32),
                                          joint_ids=[_S["ids"][DOOR]])
    art.write_joint_damping_to_sim_index(damping=torch.tensor([[DAMPING[n] for n in NAMES]], device=art.device), joint_ids=ids)
    art.write_joint_stiffness_to_sim_index(stiffness=torch.zeros((1, len(ids)), device=art.device), joint_ids=ids)
    _set(art, BUTTON, BUTTON_K, 0.0)
    _set(art, TIMER, 0.0, 0.0)
    shut = door_start <= CATCH_ON
    _S["state"] = "shut" if shut else "free"
    _S["t_open"] = 0.0
    _set(art, DOOR, LATCH_N_M_PER_RAD if shut else 0.0, 0.0 if shut else door_start)
    print(f"[mw] door set to {math.degrees(door_start):.1f} deg, catch {'on' if shut else 'off'}", flush=True)


def angles(environment) -> tuple[float, float]:
    """(door, dial) in radians."""
    art = _art(environment)
    q = _q(art)
    return float(q[_S["ids"][DOOR]]), float(q[_S["ids"][TIMER]])


def button(environment) -> float:
    """The release button's travel in m."""
    art = _art(environment)
    return float(_q(art)[_S["ids"][BUTTON]])


def latch(environment, door: float, dt: float = 0.05) -> None:
    """The catch and the release: shut, the catch holds the door (a spring to 0) until the button is pressed 2 mm; then a
    spring swings the door to its stop and lets go once it is past 88 degrees; the catch takes it again within 5
    degrees of shut."""
    art = _art(environment)
    st = _S["state"]
    if st == "shut":
        if button(environment) >= BUTTON_PRESSED:
            _S["state"], _S["t_open"] = "opening", 0.0
            _set(art, DOOR, OPEN_K, OPEN_TO)
    elif st == "opening":
        _S["t_open"] += dt
        if door >= OPEN_DONE or _S["t_open"] > 10.0:
            _S["state"] = "free"
            _set(art, DOOR, 0.0, door)
    elif st == "free":
        if door <= CATCH_ON:
            _S["state"] = "shut"
            _set(art, DOOR, LATCH_N_M_PER_RAD, 0.0)
