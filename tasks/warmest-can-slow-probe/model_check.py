"""Offline model of the thermometer and the scorer (no Isaac): the same sensor rules and the same scoring functions as
task.py, driven by schedules of 'can X on the pad for s seconds / pad empty for s seconds'. Used to pick parameters,
to measure the proof's error rate over many hidden draws, and to score the blind baselines' measuring schedules."""
import importlib.util, math, sys, types, json
import numpy as np

# import task.py's pure functions without Isaac: stub the modules it imports
for m in ("robokit", "seg7", "yam_arena", "yam_arena.task_file"):
    sys.modules.setdefault(m, types.ModuleType(m))
rk = sys.modules["robokit"]; rk._part_state = rk.on_reset = rk.on_step = lambda *a, **k: (a[0] if a else None)
rk.part = lambda *a, **k: None
class _S:  # Seg7 stub
    def __init__(self, *a, **k): pass
    def boxes(self, first): return []
    def reset_cache(self): pass
sys.modules["seg7"].Seg7 = _S
tf = sys.modules["yam_arena.task_file"]; tf.TABLE_TOP = {"station": 0.7475}; tf.Box = lambda **k: None
spec = importlib.util.spec_from_file_location("task", __file__.replace("model_check.py", "task.py"))
T = importlib.util.module_from_spec(spec); spec.loader.exec_module(T)


class Sim:
    def __init__(self, seed, tau=None, sigma=None):
        self.temps = T.draw(seed)
        self.tau, self.sigma = tau or T.TAU, sigma or T.SIGMA
        self.noise = np.random.default_rng(2000 + seed)
        self.t, self.sensor, self.next, self.readings = 0.0, T.ROOM, 0.0, []

    def run(self, can, seconds):
        for _ in range(int(round(seconds / T.DT))):
            target = self.temps[can] if can else T.ROOM
            self.sensor += (target - self.sensor) * (1 - math.exp(-T.DT / self.tau))
            self.t += T.DT
            if self.t + 1e-9 >= self.next:
                self.next += T.REFRESH
                self.readings.append((round(self.t, 3), can, round(self.sensor + float(self.noise.normal(0, self.sigma)), 1)))

    def display(self):
        return self.readings[-1][2] if self.readings else T.ROOM


CARRY = 20.0    # pad empty between measurements (carry the can back, bring the next one), s; the demo takes about 20 s
FINAL = 10.0    # carry the chosen can to the mat


def oracle(sim, extra=10.0, cap=90.0, rounds=4):
    """Observable procedure: per can, watch the display from the moment it is let go until its own readings stop
    drifting (settled_from on what it has seen), read `extra` s more, carry it back. Re-measure leader and runner-up
    until the pooled-SD test proves a winner. Returns its pick."""
    spans = []
    def measure(c, ex):
        sim.run(None, CARRY)
        a = sim.t
        while sim.t - a < cap:
            sim.run(c, 2.0)
            rs = [r for r in sim.readings if r[0] > a]
            if T.settled_from([r[0] for r in rs], [r[2] for r in rs]) is not None:
                break
        sim.run(c, ex)
        spans.append((c, a, sim.t))
    def own():
        out = {c: [] for c in T.CANS}
        for c, a, b in spans:
            rs = [r for r in sim.readings if a < r[0] <= b]
            i = T.settled_from([r[0] for r in rs], [r[2] for r in rs])
            if i is not None:
                out[c] += [r[2] for r in rs[i:]]
        return out
    for c in T.CANS:
        measure(c, extra)
    for _ in range(rounds):
        s = own()
        best, _ = T.winner_z(s)
        if best:
            break
        missing = [c for c in T.CANS if not s[c]]
        if missing:
            todo = missing
        else:
            means = {c: np.mean(v) for c, v in s.items()}
            lead = max(means, key=means.get); rival = max((c for c in T.CANS if c != lead), key=means.get)
            todo = [lead, rival]
        for c in todo:
            measure(c, 10.0)
    s = own()
    sim.run(None, CARRY + FINAL)
    return max(T.CANS, key=lambda c: np.mean(s[c]) if s[c] else -1e9)


def quick(sim, wait=2.0):
    """The obvious procedure: each can on the pad, read the display once `wait` s after letting go, pick the max."""
    seen = {}
    for c in T.CANS:
        sim.run(None, CARRY); sim.run(c, wait); seen[c] = sim.display()
    sim.run(None, CARRY + FINAL)
    return max(seen, key=seen.get)


def rubric(sim, pick):
    """Score of the final state with `pick` alone on the mat, released (the 1.0 conditions otherwise met)."""
    T._STATE["readings"] = sim.readings
    s = T.settled(sim.readings)
    proven, z = T.winner_z(s)
    all_settled = all(len(v) > 0 for v in s.values())
    warm = max(sim.temps, key=sim.temps.get)
    if pick != warm:
        return 0.0, proven, z
    if proven == warm:
        return 1.0, proven, z
    return (0.25 if all_settled else 0.0), proven, z


if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    res = {}
    for name, pol in (("oracle", oracle), ("quick_2s", quick), ("quick_10s", lambda s: quick(s, 10.0))):
        scores, wrong_proof, times, picks_ok = [], 0, [], 0
        for seed in range(N):
            T.TEMPS = None
            sim = Sim(seed)
            pick = pol(sim)
            sc, proven, z = rubric(sim, pick)
            warm = max(sim.temps, key=sim.temps.get)
            wrong_proof += proven is not None and proven != warm
            picks_ok += pick == warm
            scores.append(sc); times.append(sim.t)
        res[name] = dict(n=N, mean_score=float(np.mean(scores)), full=int(sum(s == 1.0 for s in scores)),
                         right_pick=picks_ok, wrong_proofs=wrong_proof,
                         time_s=dict(median=float(np.median(times)), p95=float(np.percentile(times, 95)), max=float(max(times))))
    # pure false-positive rate of the proof rule: three cans at the SAME temperature, oracle schedule
    fp = 0
    for seed in range(N):
        sim = Sim(seed); t0 = sim.temps; sim.temps = {c: 6.0 for c in T.CANS}
        oracle(sim)
        fp += T.winner_z(T.settled(sim.readings))[0] is not None
    res["null_equal_temps_proof_rate"] = fp / N
    print(json.dumps(res, indent=1))
