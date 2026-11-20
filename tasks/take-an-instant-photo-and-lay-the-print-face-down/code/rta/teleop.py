"""Inspect Robots policy ``rta_teleop``: a person drives the arms (Joy-Con / gamepad / keyboard) and the run is
recorded, scored and rendered like any other policy — this is how the full-score demo videos are made.

Network protocol (UDP, JSON, one datagram per message; defaults on localhost, so run the simulator container
with ``--network host``):
  controls  host → policy, port ``port`` (7701):
    {"arm": "left"|"right"|"both", "dx": -1..1, "dy": -1..1, "dz": -1..1, "dyaw": -1..1, "grip": 0..1 (1 = open),
     "stop": bool, "home": bool}
    The policy integrates dx/dy/dz at ``speed`` m/s and dyaw at ``yaw_speed`` rad/s; the last message is held.
  frames    policy → host, port ``view_port`` (7700): per camera one datagram = 96-byte JSON header padded with
    spaces ({"cam": name, "t": s, "arm": active}) followed by a JPEG (width ``view_width``).

Each arm's TCP target starts at its park hover (``rta.plans.PARK``) with the gripper open and a straight-down
grip; targets are clamped to the bench envelope and a move that IK cannot reach within 2 cm is rejected (the
target stays). Before the task's ACT_AFTER_S the policy outputs the ready pose, like the oracle.
"""

from __future__ import annotations

import json
import socket
import time
from typing import Any

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk, Observation
from inspect_robots_yam.config import action_box
from inspect_robots_yam.packing import pack
from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ, LEFT_BASE_POS, READY, RIGHT_BASE_POS
from yam_arena.demo import grip_rotation

from rta import registry
from rta.plans import PARK
from rta.spec import ARMS, TABLE_TOP

TOP = TABLE_TOP["station"]
ENVELOPE = {"x": (0.06, 0.34), "y": (-0.33, 0.33), "z": (TOP + 0.002, TOP + 0.32)}
HEADER = 96


class TeleopPolicy(PolicyBase):
    def __init__(
        self,
        port: int = 7701,
        view_port: int = 7700,
        view_host: str = "127.0.0.1",
        view_width: int = 320,
        view_every: int = 2,
        speed: float = 0.10,
        yaw_speed: float = 1.2,
        ik_reject_m: float = 0.02,
        hold_s: float = 1.0,
    ) -> None:
        low, high = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        self.info = PolicyInfo(name="rta_teleop", action_space=action_box(pack(low, low), pack(high, high)))
        self.port, self.view_port, self.view_host = int(port), int(view_port), str(view_host)
        self.view_width, self.view_every = int(view_width), max(1, int(view_every))
        self.speed, self.yaw_speed, self.ik_reject_m, self.hold_s = float(speed), float(yaw_speed), float(ik_reject_m), float(hold_s)
        self._dt = 1.0 / CONTROL_HZ
        self._environment_id: str | None = None
        self._bases = {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
        self._sock: socket.socket | None = None
        self._out: socket.socket | None = None
        self._reset_state()

    # --- lifecycle --------------------------------------------------------------------------------------------
    def _reset_state(self) -> None:
        self._target = {arm: np.array(PARK[arm], dtype=float) for arm in ARMS}
        self._yaw = {arm: 0.0 for arm in ARMS}
        self._grip = {arm: 1.0 for arm in ARMS}
        self._q = {arm: None for arm in ARMS}
        self._ctrl: dict[str, Any] = {"arm": "left", "dx": 0.0, "dy": 0.0, "dz": 0.0, "dyaw": 0.0, "grip": 1.0, "stop": False, "home": False}
        self._active = "left"
        self._stop_t: float | None = None
        self._act_after = 0.0
        self._step = 0

    def bind(self, embodiment_info: Any) -> None:
        if embodiment_info.control_hz:
            self._dt = 1.0 / float(embodiment_info.control_hz)
        self._environment_id = embodiment_info.environment_id
        if "-station-" in (self._environment_id or ""):
            self._bases = dict(station.BASES)
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("0.0.0.0", self.port))
            self._sock.setblocking(False)
            self._out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"[rta_teleop] listening for controls on udp/{self.port}; streaming frames to {self.view_host}:{self.view_port}", flush=True)

    def reset(self, scene: Any) -> None:
        self._reset_state()
        task = registry.task_for(self._environment_id)
        self._act_after = float(task.act_after_s) if task is not None else 0.0

    # --- controls ---------------------------------------------------------------------------------------------
    def _poll(self) -> None:
        assert self._sock is not None
        while True:
            try:
                data, _ = self._sock.recvfrom(4096)
            except BlockingIOError:
                return
            try:
                msg = json.loads(data.decode())
            except ValueError:
                continue
            for k in ("dx", "dy", "dz", "dyaw", "grip"):
                if k in msg:
                    self._ctrl[k] = float(np.clip(float(msg[k]), -1.0 if k != "grip" else 0.0, 1.0))
            if msg.get("arm") in ("left", "right", "both"):
                self._ctrl["arm"] = msg["arm"]
            self._ctrl["stop"] = bool(msg.get("stop", self._ctrl["stop"]))
            self._ctrl["home"] = bool(msg.get("home", False))

    def _apply(self, t: float) -> None:
        arms = ARMS if self._ctrl["arm"] == "both" else (self._ctrl["arm"],)
        self._active = self._ctrl["arm"]
        for arm in arms:
            if self._ctrl["home"]:
                self._target[arm] = np.array(PARK[arm], dtype=float)
                self._yaw[arm] = 0.0
                self._grip[arm] = 1.0
                continue
            step = np.array([self._ctrl["dx"], self._ctrl["dy"], self._ctrl["dz"]], dtype=float) * self.speed * self._dt
            if not np.any(step) and self._ctrl["dyaw"] == 0.0 and self._grip[arm] == self._ctrl["grip"]:
                continue
            new = self._target[arm] + step
            for i, k in enumerate("xyz"):
                new[i] = float(np.clip(new[i], *ENVELOPE[k]))
            yaw = self._yaw[arm] + self._ctrl["dyaw"] * self.yaw_speed * self._dt
            q, pe, _ = ik.solve(new, ik.matrix_to_quat_xyzw(grip_rotation(yaw)), self._q[arm], base_pos=self._bases[arm], iters=60)
            if pe <= self.ik_reject_m:
                self._target[arm], self._yaw[arm], self._q[arm] = new, yaw, q
            self._grip[arm] = self._ctrl["grip"]
        self._ctrl["home"] = False

    def _joints(self, arm: str) -> np.ndarray:
        q, pe, _ = ik.solve(self._target[arm], ik.matrix_to_quat_xyzw(grip_rotation(self._yaw[arm])), self._q[arm], base_pos=self._bases[arm], iters=60)
        if self._q[arm] is None or pe <= self.ik_reject_m:
            self._q[arm] = q
        return np.r_[self._q[arm], self._grip[arm]]

    # --- frames to the operator -------------------------------------------------------------------------------
    def _stream(self, observation: Observation, t: float) -> None:
        if self._out is None or self._step % self.view_every:
            return
        try:
            import cv2
        except ImportError:
            return
        for cam, img in observation.images.items():
            a = np.asarray(img)
            if a.ndim != 3:
                continue
            if a.shape[1] > self.view_width:
                h = int(round(a.shape[0] * self.view_width / a.shape[1]))
                a = cv2.resize(a, (self.view_width, h), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", cv2.cvtColor(a, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                continue
            header = json.dumps({"cam": cam, "t": round(t, 2), "arm": self._active}).encode()
            assert len(header) <= HEADER, header  # fixed-width header: never truncate the JSON
            header = header.ljust(HEADER)
            payload = header + buf.tobytes()
            if len(payload) < 65000:
                try:
                    self._out.sendto(payload, (self.view_host, self.view_port))
                except OSError:
                    pass

    # --- policy -----------------------------------------------------------------------------------------------
    def act(self, observation: Observation) -> ActionChunk:
        self._step += 1
        t = float(observation.state_time) + self._dt
        self._poll()
        self._stream(observation, t)
        if t < self._act_after:
            per_arm = [np.r_[READY, 1.0] for _ in ARMS]
        else:
            self._apply(t)
            per_arm = [self._joints(arm) for arm in ARMS]
        meta: dict[str, Any] = {}
        if self._ctrl["stop"]:
            if self._stop_t is None:
                self._stop_t = t
            if t >= self._stop_t + self.hold_s:
                meta["request_stop"] = True
        return ActionChunk(actions=[Action(data=pack(*per_arm), meta=meta)], control_hz=1.0 / self._dt)


def rta_teleop_policy(**kwargs: Any) -> TeleopPolicy:
    """Policy ``rta_teleop``; ``-P port=7701 -P view_port=7700 -P speed=0.1`` etc."""
    return TeleopPolicy(**kwargs)
