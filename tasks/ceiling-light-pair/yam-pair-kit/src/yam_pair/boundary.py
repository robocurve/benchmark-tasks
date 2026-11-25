"""The information boundary between the two stations. Pure Python.

The ``yam_pair`` embodiment hands one policy everything (Inspect Robots runs one policy per trial). A pair policy
splits it here: each station's policy gets a standard single-station YAM observation (``top_cam``, ``left_cam``,
``right_cam``; state ``joint_pos`` 14-D; its own instruction) and nothing else: no partner images, state, actions or
instruction, and no ``overview_cam``. The two 14-D actions are joined only here, into the embodiment's 28-D action.
"""

from __future__ import annotations

import numpy as np
from inspect_robots.embodiment import EmbodimentInfo
from inspect_robots.spaces import ActionSemantics, Box, CameraSpec, ObservationSpace
from inspect_robots.types import Observation
from inspect_robots_yam.config import DEFAULT_CAMERAS, action_box, observation_space
from inspect_robots_yam.packing import DIM_LABELS, STATE_KEY

from yam_pair.layout import STATIONS

ARM_CAMERAS = DEFAULT_CAMERAS  # ("top_cam", "left_cam", "right_cam")
STATION_DIM = 14
PAIR_DIM = 2 * STATION_DIM
PAIR_DIM_LABELS = tuple(f"{s}_{label}" for s in STATIONS for label in DIM_LABELS)


def camera_name(station: str, camera: str) -> str:
    """``a`` + ``top_cam`` -> ``a_top_cam``."""
    return f"{station}_{camera}"


def state_key(station: str) -> str:
    return f"{station}_{STATE_KEY}"


def station_observation(obs: Observation, station: str, instruction: str) -> Observation:
    """The single-station observation station ``station`` is allowed to see."""
    if station not in STATIONS:
        raise ValueError(f"station must be one of {STATIONS}, got {station!r}")
    images, times = {}, {}
    for cam in ARM_CAMERAS:
        name = camera_name(station, cam)
        if name not in obs.images:
            raise ValueError(f"observation has no {name!r}")
        images[cam] = np.array(obs.images[name], copy=True)
        times[cam] = float(obs.image_times.get(name, obs.state_time))
    key = state_key(station)
    if key not in obs.state:
        raise ValueError(f"observation has no {key!r} state")
    state = np.array(obs.state[key], dtype=np.float64, copy=True).reshape(-1)
    if state.shape != (STATION_DIM,):
        raise ValueError(f"{key} must be {STATION_DIM}-D, got {state.shape}")
    extra = {"env_step": obs.extra["env_step"]} if "env_step" in obs.extra else {}
    return Observation(images=images, state={STATE_KEY: state}, instruction=instruction, image_times=times,
                       state_time=float(obs.state_time), extra=extra)


def join_actions(a, b) -> np.ndarray:
    """A's and B's 14-D actions as the embodiment's 28-D action."""
    out = []
    for station, act in (("a", a), ("b", b)):
        v = np.asarray(act, dtype=np.float64).reshape(-1)
        if v.shape != (STATION_DIM,) or not np.isfinite(v).all():
            raise ValueError(f"station {station.upper()}: expected a finite {STATION_DIM}-D action, got {v.shape}")
        out.append(v)
    return np.concatenate(out)


def split_action(data) -> dict[str, np.ndarray]:
    v = np.asarray(data, dtype=np.float64).reshape(-1)
    if v.shape != (PAIR_DIM,):
        raise ValueError(f"yam_pair takes a {PAIR_DIM}-D action, got {v.shape}")
    return {"a": v[:STATION_DIM], "b": v[STATION_DIM:]}


def pair_action_box(low14=None, high14=None) -> Box:
    single = action_box(low14, high14)
    sem = single.semantics
    semantics = ActionSemantics(control_mode=sem.control_mode, gripper=sem.gripper, dim_labels=PAIR_DIM_LABELS)
    low = None if low14 is None else np.concatenate([low14, low14])
    high = None if high14 is None else np.concatenate([high14, high14])
    return Box(shape=(PAIR_DIM,), low=low, high=high, semantics=semantics)


def pair_observation_space(height: int, width: int, overview: tuple[int, int] | None = None) -> ObservationSpace:
    cams = [CameraSpec(camera_name(s, c), height, width) for s in STATIONS for c in ARM_CAMERAS]
    if overview is not None:
        cams.append(CameraSpec("overview_cam", overview[1], overview[0]))
    return ObservationSpace(cameras=tuple(cams), state_keys=frozenset(state_key(s) for s in STATIONS))


def station_info(pair_info: EmbodimentInfo, station: str) -> EmbodimentInfo:
    """What one station's policy is bound to: a standard bimanual YAM (14-D, three cameras)."""
    box = pair_info.action_space
    low = None if box.low is None else np.asarray(box.low)[:STATION_DIM] if station == "a" else np.asarray(box.low)[STATION_DIM:]
    high = None if box.high is None else np.asarray(box.high)[:STATION_DIM] if station == "a" else np.asarray(box.high)[STATION_DIM:]
    cams = {c.name: c for c in pair_info.observation_space.cameras}
    ref = cams[camera_name(station, "top_cam")]
    return EmbodimentInfo(
        name=f"{pair_info.name}:{station}",
        action_space=action_box(low, high),
        observation_space=observation_space(ref.height, ref.width, ARM_CAMERAS),
        control_hz=pair_info.control_hz,
        is_simulated=pair_info.is_simulated,
        docs=None,
        environment_id=f"{pair_info.environment_id}#station-{station}",
        environment_revision=pair_info.environment_revision,
    )
