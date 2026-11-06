"""On the station table, in front of the arms: a mechanical metronome standing free (22 cm tall), its tempo scale
facing the arms. Its weight is on 60 and its pendulum is still.

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
SUCCESS_HOLD_S = 0.0                                        # "ticking" already needs 5 s on its own: ends the trial

INSTRUCTION = "Start the metronome at 120 beats per minute."

TOP = g.TOP
I_START, I_TARGET = g.MARKS.index(g.START), g.MARKS.index(g.TARGET)

# -- objects --------------------------------------------------------------------------------------------------------
METRONOME = UsdBody("metronome", usd="assets/metronome.usda", pos=(g.MET_X, g.MET_Y, TOP),
                    size=(g.CASE_D, g.CASE_W, g.CASE_H), articulation=True,
                    joint_pos=(("pendulum", 0.0), ("weight", g.MARK_X[I_START])),
                    drives=(("pendulum", 0.0, g.DAMP_STOP), ("weight", g.DETENT_K, g.DETENT_D)),
                    friction=(0.6, 0.5))
OBJECTS = [METRONOME]

# -- monitor: the escapement, the weight's notches, and the score's state ---------------------------------------------
_STATE: dict = {}


def _joint(environment, name: str):
    asset = environment.unwrapped.scene["metronome"]
    return asset, asset.find_joints([name])[0]


def _set_damping(environment, damping: float) -> None:
    import torch

    asset, ids = _joint(environment, "pendulum")
    asset.write_joint_damping_to_sim_index(damping=torch.tensor([[damping]], dtype=torch.float32, device=asset.device),
                                           joint_ids=ids)


def _set_speed(environment, speed: float) -> None:
    import torch

    asset, ids = _joint(environment, "pendulum")
    asset.write_joint_velocity_to_sim_index(velocity=torch.tensor([[speed]], dtype=torch.float32, device=asset.device),
                                            joint_ids=ids)


def _hold_notch(environment, index: int) -> None:
    from scenekit import set_joint_targets

    set_joint_targets(environment, "metronome", {"weight": g.MARK_X[index] + g.SAG})


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(t=0.0, theta=0.0, notch=I_START, running=False, last_beat=-1.0, beats=[], swing=0.0, touched=0.0,
                  ticking=False, tilt=0.0, tipped=False, standing=True)
    if environment is not None:
        from scenekit import write_joint_positions

        _set_damping(environment, g.DAMP_STOP)
        _hold_notch(environment, I_START)
        write_joint_positions(environment, "metronome", {"weight": g.MARK_X[I_START]})   # exactly on its mark


@before_reset
def _restart(environment) -> None:
    """Stop the pendulum and put the weight back on 60 before a reset (the reset moves the metronome only by its root)."""
    from scenekit import write_joint_positions

    _set_damping(environment, g.DAMP_STOP)
    _hold_notch(environment, I_START)
    write_joint_positions(environment, "metronome", {"pendulum": 0.0, "weight": g.MARK_X[I_START]})


_reset_state()


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    from scenekit import _np

    S = _STATE
    S["t"] += float(environment.unwrapped.step_dt)
    state = object_states(environment, ["metronome"])["metronome"]
    asset, ids = _joint(environment, "pendulum")
    theta = float(state.joints["pendulum"])
    omega = float(_np(asset.data.joint_vel)[0, ids[0]])
    x = float(state.joints["weight"])

    # the weight's spring clip: it holds its notch, and clicks to the next once pushed past half way
    n = g.notch(x)
    if n != S["notch"] and abs(x - g.MARK_X[S["notch"]]) - abs(x - g.MARK_X[n]) > g.CLICK:
        S["notch"] = n
        _hold_notch(environment, n)

    # the escapement: each time the pendulum swings through the middle with enough swing, it ticks and nudges the
    # swing toward SWING (an energy kick of at most KICK_MAX, so the period stays the pendulum's own)
    k, inertia = g.restoring(x), g.inertia(x)
    energy = 0.5 * inertia * omega * omega + k * (1 - math.cos(theta))
    S["swing"] = math.acos(max(-1.0, 1 - energy / k))
    crossed = S["theta"] * theta < 0
    S["theta"] = theta
    if S["running"] and abs(S["swing"] - g.SWING) > g.SWING_TOL:     # a hand on it changes its energy
        S["touched"] = S["t"]
    if crossed and S["swing"] >= (0.5 * g.SWING if S["running"] else g.START_SWING):
        if not S["running"]:
            S["running"], S["touched"] = True, S["t"]
            _set_damping(environment, g.DAMP_RUN)
        S["last_beat"] = S["t"]
        S["beats"] = (S["beats"] + [S["t"]])[-9:]
        to = min(S["swing"] + g.KICK_MAX, max(S["swing"] - g.KICK_MAX, g.SWING))
        if math.cos(theta) > math.cos(to):
            _set_speed(environment, math.copysign(math.sqrt(2 * k * (math.cos(theta) - math.cos(to)) / inertia), omega))
    half = math.pi * math.sqrt(inertia / k) * g.amplitude_factor(g.SWING)      # the time between beats
    if S["running"] and S["t"] - S["last_beat"] > g.BEAT_LATE * half + float(environment.unwrapped.step_dt):
        S["running"], S["beats"] = False, []                            # a beat is late: held, or the swing died
        _set_damping(environment, g.DAMP_STOP)

    R = state.rotation()
    S["tilt"] = math.acos(max(-1.0, min(1.0, R[2, 2])))
    S["tipped"] = S["tipped"] or S["tilt"] >= g.TIPPED                  # knocked over at any time
    S["standing"] = state.pos[2] - TOP < g.LIFTED and S["tilt"] < g.UPRIGHT      # on the table, not lifted or held up
    if not S["standing"]:
        S["touched"] = S["t"]
    S["ticking"] = S["running"] and S["t"] - S["touched"] >= g.ON_ITS_OWN     # nothing has touched it for 5 s


def tempo() -> float | None:
    """The measured tempo (beats a minute) over the last few beats, while it runs."""
    b = _STATE["beats"]
    if not _STATE["running"] or len(b) < 3:
        return None
    return 60.0 * (len(b) - 1) / (b[-1] - b[0])


def success(objects) -> bool:
    """The weight on 120 and the metronome standing on the table ticking on its own for 5 s, never knocked over."""
    return bool(_STATE["notch"] == I_TARGET and _STATE["ticking"] and not _STATE["tipped"])


def score(objects) -> float:
    """Partial credit (success scores 1.0): 0.5 with the weight on 120 but not ticking on its own for the last 5 s, or
    ticking on its own with the weight one notch off (116 or 126); 0 if the metronome was ever knocked over (tilted
    past 45 degrees), or anything else."""
    S = _STATE
    if S["tipped"]:
        return 0.0
    on = S["notch"] == I_TARGET
    if on and S["ticking"]:
        return 1.0
    if on or (abs(S["notch"] - I_TARGET) == 1 and S["ticking"]):
        return 0.5
    return 0.0
