"""No-simulator checks: layout, task loading, the information boundary, the pair policies (fake Dream sessions),
the scorer and the video tiler.  Run:  python -m pytest tests -q"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import numpy as np
import pytest
from inspect_robots import Scene
from inspect_robots.types import Observation

from yam_pair import boundary, taskfile
from yam_pair.layout import Layout

HERE = Path(__file__).resolve().parent
EXAMPLE = HERE.parent / "examples" / "shared-kitting" / "task.py"


# -- layout -------------------------------------------------------------------------------------------------------------
def test_facing_matches_prototype():
    lay = Layout()
    assert np.allclose(lay.origin("a"), (-0.4, 0, 0)) and np.allclose(lay.origin("b"), (0.4, 0, 0))
    assert np.allclose(lay.to_world("a", (0.425, 0.12, 0.8)), (0.025, 0.12, 0.8))
    # B is turned 180 deg: its local +x points to world -x, its left (+y) to world -y.
    assert np.allclose(lay.to_world("b", (0.425, 0.12, 0.8)), (-0.025, -0.12, 0.8))
    for s in "ab":
        p = np.array([0.3, -0.2, 0.9])
        assert np.allclose(lay.to_local(s, lay.to_world(s, p)), p)
    assert np.allclose(lay.bases("b")["left"], (0.4, -0.305, 0.75))


def test_side_by_side_and_yaw():
    lay = Layout("side_by_side", yaw_b=90)
    assert np.allclose(lay.origin("a"), (0, 0.5, 0)) and np.allclose(lay.origin("b"), (0, -0.5, 0))
    assert math.isclose(lay.heading("a"), 0.0) and math.isclose(lay.heading("b"), math.pi / 2)
    assert np.allclose(lay.dir_to_world("b", (1, 0, 0)), (0, 1, 0), atol=1e-12)
    assert math.isclose(lay.yaw_to_local("b", lay.yaw_to_world("b", 0.3)), 0.3)
    over = Layout().with_overrides(arrangement="side_by_side")
    assert over.sep == 1.0
    with pytest.raises(ValueError):
        Layout("diagonal")


def test_default_table_covers_both_stations():
    (sx, sy), (cx, cy) = Layout().default_table()
    assert math.isclose(sx, 1.18) and math.isclose(sy, 1.30) and abs(cx) < 1e-9 and abs(cy) < 1e-9


# -- task file ----------------------------------------------------------------------------------------------------------
def test_example_task_loads():
    task = taskfile.load(EXAMPLE)
    assert task.layout == Layout("facing", 0.8)
    assert set(task.instructions) == {"a", "b"} and task.instructions["a"] != task.instructions["b"]
    assert task.hooks["isaac_objects"] is not None and task.hooks["demo"] is not None
    assert "supply_base" in task.names and not task.free_objects


def test_reserved_names_rejected(tmp_path):
    p = tmp_path / "task.py"
    p.write_text("from yam_pair.task import Box\nOBJECTS=[Box('a_left_robot', pos=(0,0,1))]\n")
    with pytest.raises(ValueError, match="already name"):
        taskfile.load(p)


def test_instruction_resolution():
    task = taskfile.load(EXAMPLE)
    ti = task.instructions
    assert taskfile.resolve_instructions(task, "combined", None, None) == ti
    assert taskfile.resolve_instructions(task, "A text", None, "B text") == {"a": "A text", "b": "B text"}
    assert taskfile.resolve_instructions(task, "combined", "A2", None) == {"a": "A2", "b": ti["b"]}
    assert taskfile.resolve_instructions(None, "A", None, "B") == {"a": "A", "b": "B"}
    with pytest.raises(ValueError, match="station B"):
        taskfile.resolve_instructions(None, "only A", None, None)


def test_scoring_hooks_take_one_or_two_args(tmp_path):
    p = tmp_path / "t.py"
    p.write_text("from yam_pair.task import Box\nOBJECTS=[Box('x', pos=(0,0,1))]\n"
                 "def score(objects): return 0.5\ndef report(objects, pair): return {'peak': pair}\n")
    task = taskfile.load(p)
    assert task.call("score", {}, "PAIR") == 0.5 and task.call("report", {}, 7) == {"peak": 7}


# -- boundary -----------------------------------------------------------------------------------------------------------
def _pair_obs(t=0.0):
    images = {boundary.camera_name(s, c): np.full((176, 320, 3), 10 * i + j, np.uint8)
              for i, s in enumerate("ab") for j, c in enumerate(boundary.ARM_CAMERAS)}
    images["overview_cam"] = np.zeros((400, 640, 3), np.uint8)
    state = {"a_joint_pos": np.arange(14.0), "b_joint_pos": 100 + np.arange(14.0)}
    return Observation(images=images, state=state, instruction="combined", image_times=dict.fromkeys(images, t),
                       state_time=t, extra={"env_step": 3, "approvals": []})


def test_station_observation_is_isolated():
    obs = _pair_obs()
    b = boundary.station_observation(obs, "b", "B only")
    assert set(b.images) == {"top_cam", "left_cam", "right_cam"} and set(b.state) == {"joint_pos"}
    assert b.images["top_cam"].max() == 10 and b.state["joint_pos"][0] == 100 and b.instruction == "B only"
    assert set(b.extra) == {"env_step"}
    obs.images["b_top_cam"][:] = 99  # later changes to the pair observation don't leak in (copies)
    obs.state["b_joint_pos"][:] = -1
    assert b.images["top_cam"].max() == 10 and b.state["joint_pos"][0] == 100


def test_join_and_split():
    a, b = np.zeros(14), np.ones(14)
    j = boundary.join_actions(a, b)
    assert j.shape == (28,) and np.array_equal(boundary.split_action(j)["b"], b)
    with pytest.raises(ValueError):
        boundary.join_actions(np.zeros(28), b)
    with pytest.raises(ValueError):
        boundary.join_actions(a, np.full(14, np.nan))


# -- dream_pair with fake sessions ---------------------------------------------------------------------------------------
class FakeSession:
    """inspect_robots_dream's Session protocol, recording what it was handed."""

    def __init__(self, station, fail_at=None, delay=0.0, expire_at=None):
        self.station, self.fail_at, self.delay, self.expire_at = station, fail_at, delay, expire_at
        self.prompts, self.states, self.acts, self.threads, self.closed = [], [], 0, set(), False

    def check(self, episode):
        pass

    def start(self, episode):
        pass

    def observe(self, frames, state, prompt, timestamp):
        assert set(frames) == {"top_cam", "left_cam", "right_cam"}
        self.prompts.append(prompt)
        self.states.append(np.array(state))

    def act(self, commanded):
        from inspect_robots_dream.session import Chunk

        self.threads.add(threading.get_ident())
        self.acts += 1
        if self.fail_at is not None and self.acts >= self.fail_at:
            raise RuntimeError("DreamAPI refused the inference request: unsafe instruction")
        time.sleep(self.delay)
        base = 0.0 if self.station == "a" else 1.0
        renewed = self.expire_at is not None and self.acts >= self.expire_at
        return Chunk(rows=np.full((4, 14), base) + np.r_[np.zeros(6), 0.5, np.zeros(6), 0.5], rtt_s=self.delay,
                     attempts=2 if renewed and self.acts == self.expire_at else 1,
                     session_id=f"sess-{self.station}" + ("-2" if renewed else ""))

    def close(self):
        self.closed = True


def _bound_pair(**kw):
    from yam_pair.policies import DreamPairPolicy

    sessions = {}
    configs = {s: kw.pop(f"session_{s}", {}) for s in "ab"}

    def factory(s):
        sessions[s] = FakeSession(s, **configs[s])
        return sessions[s]

    pol = DreamPairPolicy(session_factory=factory, rows_per_request=4, preroll_steps=2, **kw)
    info = _pair_info()
    pol.bind(info)
    return pol, sessions


def _pair_info():
    from inspect_robots.embodiment import EmbodimentInfo

    from yam_pair.embodiment import YamPairEmbodiment

    low = np.r_[np.full(6, -3.0), 0.0] .tolist() * 4
    high = np.r_[np.full(6, 3.0), 1.0].tolist() * 4
    return EmbodimentInfo(name="yam_pair", action_space=boundary.pair_action_box(np.array(low[:14]), np.array(high[:14])),
                          observation_space=boundary.pair_observation_space(176, 320, (640, 400)), control_hz=20.0,
                          is_simulated=True, environment_id="test-env")


def test_dream_pair_routes_each_station_its_own_inputs():
    pol, ses = _bound_pair(instruction_a="A says", instruction_b="B says")
    pol.reset(Scene(id="s", instruction="combined"))
    out = [pol.act(_pair_obs(t / 20)).actions[0].data for t in range(6)]
    assert ses["a"].prompts == ["A says"] * 6 and ses["b"].prompts == ["B says"] * 6
    assert ses["a"].states[0][0] == 0 and ses["b"].states[0][0] == 100
    # pre-roll holds each station's own measured pose, then each station's rows land in its own half
    assert np.array_equal(out[0][:14], np.arange(14.0)) and np.array_equal(out[0][14:], 100 + np.arange(14.0))
    assert out[2][0] == 0.0 and out[2][14] == 1.0 and out[2].shape == (28,)
    tr = pol.transcript()
    assert tr["instructions"] == {"a": "A says", "b": "B says"} and tr["session_ids"] == {"a": ["sess-a"], "b": ["sess-b"]}
    pol.close()
    assert ses["a"].closed and ses["b"].closed


def test_dream_pair_queries_concurrently():
    pol, ses = _bound_pair(instruction_a="A", instruction_b="B", session_a={"delay": 0.3}, session_b={"delay": 0.3})
    pol.reset(Scene(id="s", instruction="x"))
    for t in range(2):
        pol.act(_pair_obs())
    tick = time.monotonic()
    pol.act(_pair_obs())  # first request for both
    assert time.monotonic() - tick < 0.5  # both 0.3 s requests overlap
    pol.close()


def test_dream_pair_error_names_station_and_stops():
    pol, ses = _bound_pair(instruction_a="A", instruction_b="B", session_b={"fail_at": 1})
    pol.reset(Scene(id="s", instruction="x"))
    pol.act(_pair_obs())
    pol.act(_pair_obs())
    with pytest.raises(RuntimeError, match="station B Dream session .*unsafe instruction"):
        pol.act(_pair_obs())
    assert "station B" in pol.transcript()["error"]
    pol.close()
    assert ses["a"].closed and ses["b"].closed


def test_dream_pair_only_a_holds_b():
    pol, ses = _bound_pair(instruction_a="A", instruction_b="B", only="a")
    pol.reset(Scene(id="s", instruction="x"))
    out = [pol.act(_pair_obs()).actions[0].data for _ in range(4)]
    assert "b" not in ses and np.array_equal(out[-1][14:], 100 + np.arange(14.0))
    pol.close()


def test_dream_pair_inspect_rollout_records_policy_error():
    """Through Inspect's own rollout: a refusal from B is recorded as a PolicyError with the partial steps."""
    from inspect_robots.controller import DefaultController
    from inspect_robots.errors import PolicyError
    from inspect_robots.rollout import rollout
    from inspect_robots.approver import AutoApprover
    from inspect_robots.types import StepResult

    class FakePair:
        def __init__(self):
            self.info = _pair_info()

        def reset(self, scene, *, seed=None):
            return _pair_obs()

        def step(self, action):
            return StepResult(observation=_pair_obs(), info={"score": 0.0, "cross_contact_n": 0.0})

        def close(self):
            pass

    class Sink:
        def log_step(self, *a):
            pass

    pol, ses = _bound_pair(instruction_a="A", instruction_b="B", session_b={"fail_at": 2})
    with pytest.raises(PolicyError) as err:
        rollout(pol, FakePair(), Scene(id="s", instruction="x"), max_steps=50, seed=None, epoch=0,
                controller=DefaultController(), approver=AutoApprover(), sink=Sink())
    rec = err.value.record
    assert rec.status == "error" and "station B" in rec.error and len(rec.steps) > 2
    assert rec.policy_transcript["a"]["requests"]


# -- scorer -------------------------------------------------------------------------------------------------------------
def test_pair_task_score():
    from types import SimpleNamespace as NS

    from yam_pair.scoring import pair_task_score

    steps = [NS(result=NS(terminated=False, termination_reason=None, info={"cross_contact_n": f, "score": 0.5,
                                                                            "report": {"collision_free": 1.0}}))
             for f in (0.0, 2.0, 0.1)]
    rec = NS(steps=steps, termination_reason="max_steps", status="success")
    s = pair_task_score()(rec, None)
    assert s.value == 0.5 and s.metadata["cross_contact_peak_n"] == 2.0 and s.metadata["inter_station_contact"]
    steps[-1].result.terminated, steps[-1].result.termination_reason = True, "success"
    assert pair_task_score()(rec, None).value == 1.0


# -- media --------------------------------------------------------------------------------------------------------------
def test_video_tiles_six_cameras(tmp_path):
    from yam_pair import media

    frames = tmp_path / "frames"
    frames.mkdir()
    obs = _pair_obs()
    for t in range(3):
        for cam, img in obs.images.items():
            np.save(frames / f"scene-0-e0_{cam}_{t:06d}.npy", img)
    out = tmp_path / "v.mp4"
    assert media.render(tmp_path, out) == 3 and out.stat().st_size > 0
    trial, cams = media.find_frames(tmp_path)
    assert trial == "scene-0-e0" and len(cams) == 7


def test_on_trial_end_writes_summary(tmp_path):
    from types import SimpleNamespace as NS

    from yam_pair.policies import ScriptedPairPolicy

    pol = ScriptedPairPolicy()
    pol.instructions = {"a": "A", "b": "B"}
    pol.on_trial_start("scene-0", 0, str(tmp_path), "run1")
    for _ in range(4):
        pol._tick()
    steps = [NS(t=i, result=NS(info={"cross_contact_n": f, "score": 0.25, "report": {"collision_free": 1.0}}))
             for i, f in enumerate((0.0, 0.7, 0.2))]
    rec = NS(steps=steps, termination_reason="max_steps", status="success", metadata={}, scene_id="scene-0", epoch=0)
    pol.on_trial_end(rec, str(tmp_path), "run1")
    s = rec.metadata["yam_pair"]
    assert s["cross_contact_peak_n"] == 0.7 and s["steps_over_threshold"] == 1 and s["final_score"] == 0.25
    assert (tmp_path / rec.metadata["yam_pair_series"]).is_file() and "step_wall_s" in s


def test_dream_pair_default_rows_and_restart_audit(tmp_path):
    from types import SimpleNamespace as NS

    from yam_pair.policies import DEFAULT_ROWS_PER_REQUEST, DreamPairPolicy

    pol = DreamPairPolicy(session_factory=lambda s: FakeSession(s), instruction_a="A", instruction_b="B")
    assert pol.rows_per_request == DEFAULT_ROWS_PER_REQUEST == 24
    assert all(p.rows_per_request == 24 for p in pol.policies.values())
    pol.close()
    pol, ses = _bound_pair(instruction_a="A", instruction_b="B", session_b={"expire_at": 2})
    pol.reset(Scene(id="s", instruction="x"))
    for _ in range(2 + 4 * 3):
        pol.act(_pair_obs())
    audit = pol.transcript()["session_audit"]
    assert audit["a"]["session_restarts"] == 0 and audit["b"]["session_restarts"] == 1
    assert audit["b"]["retried_requests"] == 1 and audit["a"]["requests"] == 3
    rec = NS(steps=[NS(t=0, result=NS(info={"cross_contact_n": 0.0}))], termination_reason="max_steps",
             status="success", metadata={}, scene_id="scene-0", epoch=0)
    pol.on_trial_start("scene-0", 0, str(tmp_path), "r")
    pol.on_trial_end(rec, str(tmp_path), "r")
    assert rec.metadata["yam_pair"]["dream"]["sessions"]["b"]["session_restarts"] == 1
    pol.close()
