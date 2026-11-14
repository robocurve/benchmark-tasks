"""Play Ode to Joy's opening on a touch-sensitive keyboard: a soft left-hand bass under a loud right-hand melody.

A 61-key home keyboard (Yamaha PSR-E373 size: 945 x 369 x 118 mm, full-size keys, C2 to C7) lies on the table, its keys towards the arms. The piece is
the opening of the keyboard's own lesson song "Ode to Joy (Basic)" (Song 010): right hand E5 E5 F5 G5 | G5 F5 E5 D5 in quarter notes, left hand a whole
note C4 (middle C), then a whole note G3. A touch-sensitive keyboard sets each note's loudness from how fast its key goes down, so the right hand
strikes its keys briskly and lets each up before the next, and the left presses its bass keys slowly, all the way down, and holds each through its bar.

Model (declared in code): every key is a link on a hinge near its back, with a return spring (about 0.5 N at its front at rest, 0.8 N at the bed) and a
10 mm dip at its front. Two virtual contacts under each key, as velocity-sensitive keyboards use, close at 3 mm and 8 mm of dip at the key's front: the
note sounds when the second closes, with a velocity (MIDI 1 to 127) set from the time between the two, and it ends when the key rises past the first.
The contacts are read at every physics step (240 Hz), each crossing time interpolated between steps. No audio is simulated.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Play the opening of Ode to Joy, as in the keyboard's lesson, at about 80 beats per minute. Right hand, loud: E E F G, G F E D in quarter notes, "
               "letting each key up before the next, "
               "starting on the E a tenth above middle C. Left hand, soft: middle C with the first E, held through the first bar and let up before the G below it, "
               "then that G with the second bar's G, held through the second bar. "
               "Keep a steady beat, then let every key up and lift both hands clear of the keys.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the keyboard (station frame, heights above the table)
PITCH = 0.0235  # white keys, centre to centre
WHITE_W, WHITE_L, WHITE_H = 0.0225, 0.150, 0.014
BLACK_W, BLACK_L, BLACK_RISE, BLACK_H = 0.011, 0.095, 0.012, 0.020
X_FRONT = 0.255  # the white keys' fronts
X_PIVOT = X_FRONT + WHITE_L  # every key's hinge, at its back
Z_WHITE = 0.085  # the white keys' tops
Z_PIVOT = Z_WHITE - WHITE_H / 2
BODY = (0.250, 0.619, 0.118)  # the case's front and back (x), and its height
DIP = 0.010  # the key's travel at its front
C1, C2 = 0.003, 0.008  # the two contacts, by dip at the key's front
REARM = 0.0005  # a key must rise this far above the first contact to end its note
V_SLOW, V_FAST = 0.01, 1.0  # m/s at the key's front between the contacts: velocity 1 and 127 (log between)
F_REST, F_BED = 0.5, 0.8  # N at the key's front: the return spring at rest and at the bed
NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
G4_WHITE = 18  # G4's place among the white keys from C2


def _keys():
    """Every key from C2 (MIDI 36) to C7 (96): name, MIDI number, black or not, y."""
    out, white = [], 0
    for midi in range(36, 97):
        name = NAMES[midi % 12] + str(midi // 12 - 1)
        black = "#" in name
        if black:
            y = -(white - 0.5 - G4_WHITE) * PITCH
        else:
            y = -(white - G4_WHITE) * PITCH
            white += 1
        out.append((name.replace("#", "s"), midi, black, y))
    return out


KEYS = _keys()
KEY_Y = {n: y for n, _, _, y in KEYS}
KEY_MIDI = {n: k for n, k, _, _ in KEYS}
KEY_L = {n: (BLACK_L if b else WHITE_L) for n, _, b, _ in KEYS}
SPAN = (min(KEY_Y.values()) - PITCH / 2, max(KEY_Y.values()) + PITCH / 2)
Y_C = 0.5 * (SPAN[0] + SPAN[1])
HALF_W = 0.4725  # the case: 945 mm across
SPLIT = 65  # F4: the keys are two articulations, C2 to E4 and F4 to C7, split where no black key sits between

MELODY = ("E5", "E5", "F5", "G5", "G5", "F5", "E5", "D5")
BASS = ("C4", "G3")
BASS_WITH = (0, 4)  # the melody note each bass note sounds with
SYNC_S = 0.08
HELD_BEATS = 3.0
SOFTER_BY = 25
TEMPO_BPM = (60.0, 100.0)
EVEN_WITHIN = 0.10
CLEAR_ABOVE = 0.05
WRONG_COST, WRONG_MAX = 0.1, 2
MELODY_EACH, BASS_EACH = 0.25 / 8, 0.125  # the ladder's steps below the dynamics rung: each melody note in order, each bass note on time and held

LOOKS = {
    "white": m.Look(color=m.rgb(0.93, 0.92, 0.89), roughness=0.35),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.06), roughness=0.3),
    "case": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.6),
    "panel": m.Look(color=m.rgb(0.16, 0.16, 0.17), roughness=0.55),
    "grille": m.Look(color=m.rgb(0.07, 0.07, 0.08), roughness=0.9),
    "lcd": m.Look(color=m.rgb(0.35, 0.42, 0.36), roughness=0.2),
    "button": m.Look(color=m.rgb(0.55, 0.56, 0.58), roughness=0.4),
}


def _spring(L: float) -> tuple[float, float]:
    """The return spring for a key of lever `L` (its front from the hinge): stiffness in N m/rad and the preload angle its target is set beyond rest."""
    bed = math.atan(DIP / L)
    k = (F_BED - F_REST) * L / bed
    return k, F_REST * L / k


def _key_link(name: str, black: bool, y: float) -> m.Link:
    if black:
        size, c = (BLACK_L, BLACK_W, BLACK_H), (X_PIVOT - BLACK_L / 2, y, Z_WHITE + BLACK_RISE - BLACK_H / 2)
    else:
        size, c = (WHITE_L, WHITE_W, WHITE_H), (X_FRONT + WHITE_L / 2, y, Z_WHITE - WHITE_H / 2)
    L = BLACK_L if black else WHITE_L
    k, _ = _spring(L)
    bed = math.degrees(math.atan(DIP / L))
    return m.hinge(name, "Y", (X_PIVOT, y, Z_PIVOT), -bed, 0.0, [(name, m.box(size, c), "black" if black else "white")], [m.col_box(f"{name}_c", size, c)],
                   spring=k, damping=0.01, friction=0.0, mass=0.015 if black else 0.03, effort=5.0)


def _keybed(lo: int, hi: int, name: str) -> m.Part:
    links = [_key_link(n, b, y) for n, k, b, y in KEYS if lo <= k <= hi]
    return m.mechanism(name, (0.0, 0.0, TOP), links, {k: LOOKS[k] for k in ("white", "black")}, friction=(0.6, 0.5), solver_iters=16)


def _case() -> m.Part:
    parts, cols = [], []
    x0, x1, h = BODY
    y0, y1 = Y_C - HALF_W, Y_C + HALF_W
    floor = Z_WHITE - WHITE_H - DIP - 0.002  # under the keys, clear of their fronts at the bed

    def b(nm, size, c, look, col=True):
        parts.append((nm, m.box(size, c), look))
        if col:
            cols.append(m.col_box(f"{nm}_c", size, c))

    b("base", (X_PIVOT - x0, y1 - y0, floor), ((x0 + X_PIVOT) / 2, Y_C, floor / 2), "case")
    for nm, ya, yb in (("cheek_l", SPAN[1] + 0.001, y1), ("cheek_r", y0, SPAN[0] - 0.001)):
        b(nm, (X_PIVOT - x0, yb - ya, Z_WHITE + 0.008), ((x0 + X_PIVOT) / 2, (ya + yb) / 2, (Z_WHITE + 0.008) / 2), "case")
    b("back", (x1 - X_PIVOT - 0.002, y1 - y0, h), ((X_PIVOT + 0.002 + x1) / 2, Y_C, h / 2), "panel")
    b("lcd", (0.05, 0.09, 0.001), (X_PIVOT + 0.12, Y_C, h + 0.0005), "lcd", col=False)
    for k, yy in enumerate((Y_C + 0.30, Y_C - 0.30)):
        b(f"grille{k}", (0.10, 0.16, 0.001), (X_PIVOT + 0.11, yy, h + 0.0005), "grille", col=False)
    for k in range(8):
        b(f"button{k}", (0.012, 0.018, 0.003), (X_PIVOT + 0.05, Y_C + 0.16 - 0.04 * k, h + 0.0015), "button", col=False)
    return m.fixed("case", (0.0, 0.0, TOP), parts, cols, {k: LOOKS[k] for k in ("case", "panel", "lcd", "grille", "button")})


OBJECTS = [_case(), _keybed(36, SPLIT - 1, "keys_lo"), _keybed(SPLIT, 96, "keys_hi")]
PART_OF = {n: ("keys_lo" if k < SPLIT else "keys_hi") for n, k, _, _ in KEYS}


def velocity(dt: float) -> int:
    """MIDI velocity from the time between the two contacts: log of the key's speed, V_SLOW gives 1 and V_FAST 127."""
    s = (C2 - C1) / max(dt, 1e-4)
    return int(np.clip(round(1 + 126 * math.log(s / V_SLOW) / math.log(V_FAST / V_SLOW)), 1, 127))


class Keys:
    """The keyboard's own sensing, at every physics step: each key's two contacts, and the note log (name, onset, velocity, release) a MIDI logger
    records."""

    def __init__(self):
        self.env = None
        self.reset()

    def reset(self, env=None):
        if env is not None and self.env is not env:
            self.env = env
            self._hook(env)
        if env is not None:  # each key's spring set past its rest, so that it rests on its stop with its preload
            for n, L in KEY_L.items():
                m.set_target(env, PART_OF[n], n, _spring(L)[1])
        self.t = 0.0
        self.state = {n: 0 for n in KEY_Y}  # 0 up, 1 past the first contact, 2 sounding
        self.t1 = {n: 0.0 for n in KEY_Y}
        self.last = {n: 0.0 for n in KEY_Y}
        self.log: list[list] = []  # [name, onset, velocity, release or None]
        self.steps = 0
        self.armed = env is not None

    def _hook(self, env):
        """Run `physics` after every physics step: the harness steps physics several times per control step, and a key struck fast crosses both
        contacts within one control step."""
        base = env.unwrapped
        sim = base.sim
        self.dt = float(getattr(base, "physics_dt", None) or sim.get_physics_dt())
        self.assets = {p: base.scene[p] for p in ("keys_lo", "keys_hi")}
        self.index = {p: {n: a.joint_names.index(f"{n}_joint") for n in KEY_Y if PART_OF[n] == p} for p, a in self.assets.items()}
        if getattr(sim, "_keys_hooked", False):
            return
        step = sim.step

        def stepped(*a, **k):
            out = step(*a, **k)
            KEYBOARD.physics()
            return out

        sim.step = stepped
        sim._keys_hooked = True

    def _read(self) -> dict:
        dips = {}
        for p, a in self.assets.items():
            data = a.data
            buf = getattr(data, "_joint_pos", None)
            if buf is not None and hasattr(buf, "timestamp"):
                buf.timestamp = -1.0  # fresh from the solver, not the control step's cached copy
            q = m._np(data.joint_pos)[0]
            for n, i in self.index[p].items():
                dips[n] = KEY_L[n] * math.sin(max(0.0, -float(q[i])))
        return dips

    def physics(self):
        if not self.armed:
            return
        self.t += self.dt
        self.steps += 1
        for n, dip in self._read().items():
            prev, s = self.last[n], self.state[n]
            if s == 0 and dip >= C1:
                self.t1[n] = self._cross(prev, dip, C1)
                self.state[n] = 1
                s = 1
            if s == 1 and dip >= C2:
                t2 = self._cross(prev, dip, C2)
                self.log.append([n, t2, velocity(t2 - self.t1[n]), None])
                self.state[n] = 2
                print(f"[keys] note {n} at {t2:.3f} s, velocity {self.log[-1][2]} (contacts {1000 * (t2 - self.t1[n]):.1f} ms apart)", flush=True)
            elif s >= 1 and dip < C1 - REARM:
                if s == 2:
                    rec = next(r for r in reversed(self.log) if r[0] == n and r[3] is None)
                    rec[3] = self._cross(prev, dip, C1 - REARM)
                    print(f"[keys] {n} up at {rec[3]:.3f} s", flush=True)
                self.state[n] = 0
            self.last[n] = dip

    def _cross(self, a: float, b: float, level: float) -> float:
        """The time the dip passed `level` in the step just made, from `a` to `b`, linearly between the two."""
        f = 1.0 if abs(b - a) < 1e-9 else float(np.clip((level - a) / (b - a), 0.0, 1.0))
        return self.t - self.dt * (1.0 - f)

    def down(self) -> list[str]:
        return [n for n, s in self.state.items() if s >= 1]


KEYBOARD = Keys()
m.on_reset(KEYBOARD.reset)


def judge(log: list, now_t: float) -> dict:
    """The scoring text, from the note log alone: each onset is the next expected note of the melody or of the bass, or it is wrong."""
    mel, bass, wrong = [], [], 0
    for rec in log:
        n = rec[0]
        if len(mel) < len(MELODY) and n == MELODY[len(mel)]:
            mel.append(rec)
        elif len(bass) < len(BASS) and n == BASS[len(bass)]:
            bass.append(rec)
        else:
            wrong += 1
    out = {"melody": len(mel), "bass": len(bass), "wrong": wrong, "rung": round(MELODY_EACH * len(mel), 5), "why": ""}
    if len(mel) < 8:
        out["why"] = "not all eight melody notes"
        return out
    on = [r[1] for r in mel]
    gaps = np.diff(on)
    beat = float(gaps.mean())
    out["beat"] = round(beat, 3)
    held = lambda r: (r[3] if r[3] is not None else now_t) - r[1]  # noqa: E731
    bass_ok = []
    for k, b in enumerate(bass):  # each bass note: with its melody note, held 3 beats; G3 only after C4 is let up
        ok = abs(b[1] - mel[BASS_WITH[k]][1]) <= SYNC_S and held(b) >= HELD_BEATS * beat
        if k == 1:
            ok = ok and bass[0][3] is not None and bass[0][3] < b[1]
        bass_ok.append(ok)
    out["rung"] = 0.25 + BASS_EACH * sum(bass_ok)
    if not (len(bass) == 2 and all(bass_ok)):
        out["why"] = "the bass: not with melody notes 1 and 5, not held 3 beats, or C4 not let up before G3"
        return out
    mean_v = float(np.mean([r[2] for r in mel]))
    out["velocities"] = {"melody": [r[2] for r in mel], "bass": [r[2] for r in bass]}
    if not all(b[2] <= mean_v - SOFTER_BY for b in bass):
        out["why"] = f"the bass not {SOFTER_BY} softer than the melody's mean ({mean_v:.0f})"
        return out
    out["rung"] = 0.75
    bpm = 60.0 / beat
    even = bool(np.all(np.abs(gaps - beat) <= EVEN_WITHIN * beat))
    detached = all(mel[i][3] is not None and mel[i][3] < mel[i + 1][1] for i in range(len(mel) - 1))
    if not (even and TEMPO_BPM[0] <= bpm <= TEMPO_BPM[1] and detached):
        out["why"] = f"tempo {bpm:.0f} bpm, even {even}, each note let up before the next {detached}"
        return out
    out["rung"] = 1.0
    return out


class Trial:
    """Each control step: the ladder from the note log, less 0.1 a wrong note (up to two); 0 from a third wrong note on, and 0 while any key is down.
    1.0 once the full result has held HOLD_STEPS steps with every key up and every fingertip 5 cm above the white keys."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.rep = {}
        self.n = 0

    def step(self, env):
        self.n += 1
        j = judge(KEYBOARD.log, KEYBOARD.t)
        tips = m.fingertips(env)[:, 2] - TOP
        clear = bool(np.all(tips >= Z_WHITE + CLEAR_ABOVE))
        down = KEYBOARD.down()
        if j["wrong"] > WRONG_MAX:
            s = 0.0
        else:
            rung = j["rung"]
            if rung == 1.0 and not clear:
                rung = 0.75
            s = max(0.0, round(rung - WRONG_COST * j["wrong"], 5))
            if rung == 1.0 and j["wrong"] == 0:
                s = 1.0
        if down:
            s = 0.0
        self.hold = self.hold + 1 if s == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else min(s, 0.95)
        self.rep = {**j, "down": down, "clear": clear, "score": self.score, "physics_steps": KEYBOARD.steps}
        if self.n % 20 == 0:
            print(f"[keyboard] t {KEYBOARD.t:.2f} {self.rep}", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo --------------------------------------------------------------------------------

BEAT = 0.75  # 80 per minute
X_PRESS = X_FRONT + 0.025  # where a finger presses a white key: 25 mm from its front
UNDER = 0.005  # the finger's real underside sits this far above what fingertips() reports (kit README, 22)
BED_AT_PRESS = DIP * (X_PIVOT - X_PRESS) / WHITE_L
PAST = 0.004  # a press is commanded this far below the key's bed
HOVER_FAST, HOVER_SLOW = 0.015, 0.003  # the underside above the key's top, before a strike and before a slow press
STRIKE_S, SLOW_S = 0.10, 0.45
STRIKE_LEAD = 0.10  # a strike starts this long before its beat: the key reaches the second contact near its end
SLOW_LEAD = 0.25  # a slow press starts this long before its beat


def _low(env, side: str) -> np.ndarray:
    tips = m.fingertips(env)[(0 if side == "left" else 12):(12 if side == "left" else 24)]
    return tips[int(np.argmin(tips[:, 2]))]


def demo_script(ctx):
    """Both hands closed, fingertips straight down, closing across the keys. The right strikes the melody in time, rebounding at once; the left
    presses each bass key slowly and holds it. The beat is the demo's own clock (80 per minute); it never reads the scorer."""
    import demokit as d

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onespeed", "melody", "heldE", "shallow", "hold"):
        raise ValueError(mode)
    for _ in range(10):
        yield
    if mode == "none":
        for _ in range(60):
            yield
        return
    grip = d.down(0.0)
    over = lambda n, h: np.array([X_PRESS, KEY_Y[n], TOP + Z_WHITE - UNDER + h])  # noqa: E731  the hand's lowest fingertip point for the underside h above key n
    yield from ctx.together(l.move(over("C4", 0.05), grip=grip, finger=0.0, t=2.5, label="over C4, fingers closed"),
                            r.move(over("E5", 0.05), grip=grip, finger=0.0, t=2.5, label="over E5, fingers closed"))
    yield from ctx.together(l.servo(over("C4", HOVER_SLOW), tol=0.001, timeout=2.0, gain=0.3, read=lambda: _low(env, "left"), label="just over C4"),
                            r.servo(over("E5", HOVER_FAST), tol=0.001, timeout=2.0, gain=0.3, read=lambda: _low(env, "right"), label="1.5 cm over E5"))
    yield from l.wait(0.5)
    t0 = ctx.t + 1.0  # melody note 1's beat
    fast_dz = HOVER_FAST + BED_AT_PRESS + PAST
    slow_dz = HOVER_SLOW + BED_AT_PRESS + PAST
    if mode == "shallow":  # every press stops about 3 mm into the key at the finger (the real finger sits about 5 mm under the estimate here): past the
        fast_dz, slow_dz = HOVER_FAST - 0.002, HOVER_SLOW - 0.002  # first contact, short of the second

    def wait_until(t):
        while ctx.t < t:
            yield

    def melody():
        prev = "E5"
        for i, n in enumerate(MELODY):
            if n != prev:
                yield from r.move(r.at() + np.array([0.0, KEY_Y[n] - KEY_Y[prev], 0.0]), t=0.3, label=f"over {n}")
            prev = n
            yield from wait_until(t0 + i * BEAT - STRIKE_LEAD)
            if mode == "heldE" and i == 1:
                continue  # the repeated E: the key is held down from note 1, not let up and struck again
            yield from r.move(r.at() - np.array([0.0, 0.0, fast_dz]), t=STRIKE_S, label=f"strike {n}")
            yield  # one step at the bottom: the arm lags its command
            print(f"[keyboard] strike {n}: finger underside {1000 * (_low(env, 'right')[2] - TOP + UNDER - Z_WHITE):.1f} mm from the key's top, t {ctx.t:.2f}", flush=True)
            if mode == "heldE" and i == 0:
                yield from wait_until(t0 + 1.5 * BEAT)
            yield from r.move(r.at() + np.array([0.0, 0.0, fast_dz]), t=0.15, label=f"up off {n}")
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.08]), t=0.6, label="the right hand up")

    def bass():
        if mode == "melody":
            yield from l.move(l.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="the left hand up, out of it")
            return
        dz, t_press = (fast_dz, STRIKE_S) if mode == "onespeed" else (slow_dz, SLOW_S)
        lead = STRIKE_LEAD if mode == "onespeed" else SLOW_LEAD
        if mode == "onespeed":
            yield from l.move(l.at() + np.array([0.0, 0.0, HOVER_FAST - HOVER_SLOW]), t=0.4, label="1.5 cm over C4")
        for k, n in enumerate(BASS):
            yield from wait_until(t0 + BASS_WITH[k] * BEAT - lead)
            yield from l.move(l.at() - np.array([0.0, 0.0, dz]), t=t_press, label=f"press {n}")
            yield
            print(f"[keyboard] press {n}: finger underside {1000 * (_low(env, 'left')[2] - TOP + UNDER - Z_WHITE):.1f} mm from the key's top, t {ctx.t:.2f}", flush=True)
            if k == 0:  # C4 held 3 beats, up off it, a pause for the arm to rise, then across to G3 at the same height
                yield from wait_until(t0 + 3.0 * BEAT)
                yield from l.move(l.at() + np.array([0.0, 0.0, dz]), t=0.08, label=f"let {n} up")
                yield from l.wait(0.03)
                yield from l.move(l.at() + np.array([0.0, KEY_Y["G3"] - KEY_Y["C4"], 0.0]), t=0.20, label="over G3")
                z_now = float(_low(env, "left")[2])  # sideways only: the hand's height stays as it is
                yield from l.servo(np.array([X_PRESS, KEY_Y["G3"], z_now]), tol=0.002, timeout=0.12, gain=0.6,
                                   read=lambda: _low(env, "left"), label="settled over G3")
        yield from wait_until(t0 + 7.4 * BEAT)
        if mode == "hold":  # the last bass note left down at the end
            return
        yield from l.move(l.at() + np.array([0.0, 0.0, dz]), t=0.15, label="let G3 up")
        yield from l.move(l.at() + np.array([0.0, 0.0, 0.08]), t=0.6, label="the left hand up")

    yield from ctx.together(melody(), bass())
    for _ in range(50):
        yield
    print(f"[keyboard] end: {TRIAL.rep}", flush=True)
