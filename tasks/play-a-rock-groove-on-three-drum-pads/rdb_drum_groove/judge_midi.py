"""Score one trial of the drum groove from its MIDI file and the trial's events: the same code for a real trial (the
drum module's USB MIDI recording, events filled in from the trial video) and for a simulated one (the 240 Hz hit log
written as MIDI, events written by the simulation). Pure Python 3.9+, standard library only (the plot needs
matplotlib).

    python rdb_drum_groove/judge_midi.py <file.mid> --events events.json [--plot] [--out <folder>] [--name <run>]

1. Note-ons become pads (``layout.PAD_OF_NOTE``: 42, 46 and 23 hi-hat, 38 snare, 40 snare rim, 43 tom; 44, the
   hi-hat pedal, is ignored; anything else is "other"). Note-ons on the same pad less than 40 ms apart are one hit.
   A real trial's events can list note-ons sent by a gripper, an arm or a dropped stick (``removed``): they play no
   note.
2. One tempo (60 to 90 bpm, every 0.25 bpm, then every 0.01 bpm round the best) and one start are fitted: the most
   written notes with a hit on their pad within 50 ms of their place, then the least squared error; then a least
   squares fit of start and beat length to the matched notes, and the notes matched again.
3. Each hit is a played note (on time within 25 ms, off the grid from 25 to 50 ms), a count-in (up to 4 hi-hat hits in
   the bar before the first note), a wrong-pad hit (a rim, or an unmatched hit within 50 ms of another pad's note) or
   an extra hit (any other).
4. H notes played; G pairs together (both notes played, by the two different sticks, within 40 ms); E extra or wrong
   hits plus gripper or arm strikes. The accent rule: the hi-hat's numbers average at least 25 velocity above its
   ands, each number is louder than the ands either side of it, and the snare averages at least 25 above the hi-hat
   ands. The clean end: no hit after the last pair, and both sticks held clear of every pad (50 mm) for 1 s starting
   within 3 s of it.
5. 1.0: H 41, G 8, RMS at most 25 ms, fitted tempo 71 to 79 bpm, the accent rule, E at most 1, a clean end; 0.9: all
   that but the accent rule; 0.75: H 37, G 7, RMS 35 ms, E 3; 0.5: H 31, G 5, E 3; 0.25: H 16, E 3; else 0. A gripper
   or arm striking a pad caps the score at 0.5, a stick resting on a pad for 0.5 s or more at 0.75; in a real trial a
   pad knocked 10 mm off its mark scores 0.

events.json (every key optional):
  "hits": [{"t": s, "note": n, "stick": "left" | "right"}]   which stick sent each note (simulation: every hit)
  "pairs": {"<beat>": true | false}                          real trial: did both sticks come down at this pair
  "removed": [{"t": s, "note": n, "why": "gripper" | "arm" | "dropped stick"}]
  "arm_contacts": [{"t": s, "part": ..., "pad": ...}]       a gripper or arm striking a pad
  "rests": [{"t": s, "stick": ..., "pad": ..., "duration_s": d}]
  "end": {"clean_from": s or null} (simulation) or {"clean": true | false} (real trial)
  "pad_moved": false                                          real trial: a pad knocked 10 mm off its mark
"""

import bisect
import json
import math
import os
import sys
from pathlib import Path

if __package__ in (None, ""):                   # run as a script: python rdb_drum_groove/judge_midi.py
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from rdb_drum_groove import layout as L     # noqa: E402
    from rdb_drum_groove.midi import read_midi  # noqa: E402
else:
    from . import layout as L
    from .midi import read_midi

PADS = ("hihat", "snare", "tom")


# -- reading -----------------------------------------------------------------------------------------------------
def load_hits(note_ons, note_map=None, removed=()):
    """[(t, note, velocity)] -> hits [{t, note, velocity, pad}], sorted, with removed note-ons dropped and the same
    pad's note-ons less than 40 ms apart merged into the first."""
    note_map = dict(L.PAD_OF_NOTE if note_map is None else note_map)
    gone = [(float(r["t"]), int(r["note"])) for r in removed]
    hits = []
    for t, note, vel in sorted(note_ons):
        if note in note_map and note_map[note] is None:
            continue                                            # the hi-hat pedal
        if any(abs(t - rt) <= .002 and note == rn for rt, rn in gone):
            continue
        hits.append({"t": float(t), "note": int(note), "velocity": int(vel), "pad": note_map.get(note, "other")})
    merged, last = [], {}
    for h in hits:
        prev = last.get(h["pad"])
        if prev is not None and h["t"] - prev < L.MASK_S - 1e-9:
            continue
        last[h["pad"]] = h["t"]
        merged.append(h)
    return merged


# -- fitting -----------------------------------------------------------------------------------------------------
def _by_pad(hits):
    out = {}
    for j, h in enumerate(hits):
        out.setdefault(h["pad"], []).append(j)
    return out


def match(hits, notes, bpm, t0, window=L.WINDOW_S):
    """Greedy, in time order: each note takes the nearest unused hit on its pad within ``window`` of its time.
    Returns (matches, errors): matches[i] the hit index or None, errors[i] in seconds or None."""
    p = 60. / bpm
    by_pad = _by_pad(hits)
    times = {pad: [hits[j]["t"] for j in idx] for pad, idx in by_pad.items()}
    used, matches, errors = set(), [], []
    for b, pad, _ in notes:
        want = t0 + b * p
        idx, ts = by_pad.get(pad, []), times.get(pad, [])
        k = bisect.bisect_left(ts, want - window)
        best = None
        while k < len(ts) and ts[k] <= want + window:
            j = idx[k]
            if j not in used and (best is None or abs(ts[k] - want) < abs(hits[best]["t"] - want)):
                best = j
            k += 1
        matches.append(best)
        errors.append(None if best is None else hits[best]["t"] - want)
        if best is not None:
            used.add(best)
    return matches, errors


def _quality(errors):
    got = [e for e in errors if e is not None]
    return len(got), sum(e * e for e in got)


def _best_start(hits, notes, bpm, window):
    """For one tempo: the start that matches the most notes (a sweep over every hit-note pairing's interval), then
    the least squared error among the starts that do."""
    p = 60. / bpm
    by_pad = _by_pad(hits)
    events = []
    for i, (b, pad, _) in enumerate(notes):
        for j in by_pad.get(pad, []):
            c = hits[j]["t"] - b * p
            events.append((c - window, 0, i))
            events.append((c + window, 1, i))
    if not events:
        return None
    events.sort()
    count, covered, best, starts = {}, 0, 0, []
    for k, (x, kind, i) in enumerate(events):
        if kind == 0:
            count[i] = count.get(i, 0) + 1
            if count[i] == 1:
                covered += 1
            nxt = events[k + 1][0] if k + 1 < len(events) else x
            if covered > best:
                best, starts = covered, [(x + nxt) / 2]
            elif covered == best:
                starts.append((x + nxt) / 2)
        else:
            count[i] -= 1
            if count[i] == 0:
                covered -= 1
    result = None
    if len(starts) > 64:                         # many equal covers (scattered hits): sample them evenly
        starts = [starts[int(k * len(starts) / 64)] for k in range(64)]
    for t0 in starts:
        m, e = match(hits, notes, bpm, t0, window)
        # the start minimising the squared error with this set of matches
        pairs = [(hits[j]["t"] - b * p) for j, (b, _, _) in zip(m, notes) if j is not None]
        if pairs:
            t0 = sum(pairs) / len(pairs)
            m, e = match(hits, notes, bpm, t0, window)
        q = _quality(e)
        key = (q[0], -q[1])
        if result is None or key > result[0]:
            result = (key, t0)
    return result


def fit(hits, notes=L.NOTES, bpm_range=L.BPM_RANGE, window=L.WINDOW_S):
    """(bpm, t0, matches, errors) of the best fitted grid; (None, None, [None...], [None...]) without any match."""
    lo, hi = bpm_range
    best = None

    def consider(bpm):
        nonlocal best
        r = _best_start(hits, notes, bpm, window)
        if r is not None and (best is None or r[0] > best[0]):
            best = (r[0], bpm, r[1])

    for k in range(int(round((hi - lo) / .25)) + 1):
        consider(lo + .25 * k)
    if best is None or best[0][0] == 0:
        return None, None, [None] * len(notes), [None] * len(notes)
    centre = best[1]
    for k in range(-25, 26):
        bpm = round(centre + .01 * k, 2)
        if lo <= bpm <= hi:
            consider(bpm)
    _, bpm, t0 = best
    m, e = match(hits, notes, bpm, t0, window)
    # least squares on the matched notes (beat length kept within the range), then match again
    pts = [(notes[i][0], hits[j]["t"]) for i, j in enumerate(m) if j is not None]
    if len(pts) >= 2:
        n = len(pts)
        mb = sum(b for b, _ in pts) / n
        mt = sum(t for _, t in pts) / n
        sbb = sum((b - mb) ** 2 for b, _ in pts)
        if sbb > 0:
            p = sum((b - mb) * (t - mt) for b, t in pts) / sbb
            p = min(max(p, 60. / hi), 60. / lo)
            bpm2, t02 = 60. / p, mt - p * mb
            m2, e2 = match(hits, notes, bpm2, t02, window)
            if _quality(e2)[0] >= _quality(e)[0]:
                bpm, t0, m, e = bpm2, t02, m2, e2
    return bpm, t0, m, e


# -- scoring -----------------------------------------------------------------------------------------------------
def _sticks(hits, events):
    """Which stick sent each hit, from events["hits"] (same note, within 3 ms), or None."""
    rows = sorted((float(r["t"]), int(r["note"]), r.get("stick")) for r in events.get("hits", []))
    times = [r[0] for r in rows]
    out = []
    for h in hits:
        k = bisect.bisect_left(times, h["t"] - .003)
        stick = None
        while k < len(rows) and rows[k][0] <= h["t"] + .003:
            if rows[k][1] == h["note"]:
                stick = rows[k][2]
                break
            k += 1
        out.append(stick)
    return out


def accent_rule(notes, hits, matches):
    """(holds, details): the hi-hat's numbers at least 25 velocity above its ands on average, each number louder than
    the played ands either side of it, and the snare at least 25 above the hi-hat ands on average."""
    hh = [(notes[i][0], hits[j]["velocity"]) for i, j in enumerate(matches)
          if j is not None and notes[i][1] == "hihat"]
    nums = [(b, v) for b, v in hh if float(b).is_integer()]
    ands = [(b, v) for b, v in hh if not float(b).is_integer()]
    snare = [hits[j]["velocity"] for i, j in enumerate(matches) if j is not None and notes[i][1] == "snare"]
    if not nums or not ands or not snare:
        return False, {"why": "not enough played hi-hat numbers, ands or snare notes"}
    mean_num = sum(v for _, v in nums) / len(nums)
    mean_and = sum(v for _, v in ands) / len(ands)
    mean_snare = sum(snare) / len(snare)
    each = True
    and_beats = [b for b, _ in ands]
    for b, v in nums:
        k = bisect.bisect_left(and_beats, b)
        neighbours = [ands[k - 1][1]] if k > 0 else []
        if k < len(ands):
            neighbours.append(ands[k][1])
        if any(v <= w for w in neighbours):
            each = False
    holds = (mean_num - mean_and >= L.ACCENT_MARGIN and each and mean_snare - mean_and >= L.ACCENT_MARGIN)
    return holds, {"numbers_mean": round(mean_num, 1), "ands_mean": round(mean_and, 1),
                   "snare_mean": round(mean_snare, 1), "each_number_louder": each}


def judge(note_ons, events=None, note_map=None, notes=L.NOTES):
    """The score and everything it rests on, from note-ons [(t, note, velocity)] and the trial's events."""
    events = events or {}
    hits = load_hits(note_ons, note_map, events.get("removed", ()))
    sticks = _sticks(hits, events)
    for h, s in zip(hits, sticks):
        h["stick"] = s
    bpm, t0, matches, errors = fit(hits, notes)
    played = sum(m is not None for m in matches)
    out = {"notes": len(notes), "hits": len(hits)}
    if bpm is None:
        arm = len(events.get("arm_contacts", []))
        rests = [r for r in events.get("rests", []) if r.get("duration_s", L.REST_S) >= L.REST_S - 1e-9]
        out.update(score=0., why="no tempo from 60 to 90 bpm puts any hit on a written note", bpm=None, t0=None, H=0,
                   G=0, E=len(hits) + arm, wrong=0, extra=len(hits), arm_contacts=arm, rests=len(rests), count_in=0,
                   rms_ms=None, max_error_ms=None, tempo_in_full_range=False, accent=False, accent_detail={},
                   clean_end=False, end_detail="no notes played", pairs=[], pair_spread_max_ms=None)
        for h in hits:
            h.update(cls="extra", note_index=None, error_ms=None)
        out["classified"] = hits
        out["note_rows"] = [{"beat": b, "pad": pad, "dynamic": dyn, "hit": None, "error_ms": None}
                            for b, pad, dyn in notes]
        return out
    p = 60. / bpm
    errs = [e for e in errors if e is not None]
    rms = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else None
    # classify every hit
    matched = {j: i for i, j in enumerate(matches) if j is not None}
    count_in = 0
    first_note, last_note = t0, t0 + notes[-1][0] * p
    for j, h in enumerate(hits):
        if j in matched:
            e = errors[matched[j]]
            h.update(cls="on time" if abs(e) <= L.ON_TIME_S + 1e-9 else "off grid", note_index=matched[j],
                     error_ms=round(1000 * e, 1))
            continue
        h.update(note_index=None, error_ms=None)
        if (h["pad"] == "hihat" and count_in < L.COUNT_IN
                and first_note - 4 * p - L.WINDOW_S <= h["t"] <= first_note - L.WINDOW_S):
            count_in += 1
            h["cls"] = "count-in"
            continue
        near_other = any(pad != h["pad"] and abs(h["t"] - (t0 + b * p)) <= L.WINDOW_S for b, pad, _ in notes)
        late = h["t"] > last_note + L.AFTER_LAST_S
        h["cls"] = "wrong pad" if (h["pad"] in ("snare_rim", "other") or near_other) and not late else "extra"
    wrong = sum(h["cls"] == "wrong pad" for h in hits)
    extra = sum(h["cls"] == "extra" for h in hits)
    arm = len(events.get("arm_contacts", []))
    rests = [r for r in events.get("rests", []) if r.get("duration_s", L.REST_S) >= L.REST_S - 1e-9]
    E = wrong + extra + arm
    # pairs
    pairs, G = [], 0
    annotated = {float(k): bool(v) for k, v in events.get("pairs", {}).items()}
    for b in L.PAIRS:
        i_h = next(i for i, n in enumerate(notes) if n[0] == b and n[1] == "hihat")
        i_s = next(i for i, n in enumerate(notes) if n[0] == b and n[1] == "snare")
        jh, js = matches[i_h], matches[i_s]
        row = {"beat": b, "spread_ms": None, "two_sticks": None, "together": False}
        if jh is not None and js is not None:
            spread = abs(hits[jh]["t"] - hits[js]["t"])
            sh, ss = hits[jh]["stick"], hits[js]["stick"]
            two = (sh != ss) if (sh is not None and ss is not None) else annotated.get(b)
            row.update(spread_ms=round(1000 * spread, 1), two_sticks=two,
                       together=bool(two) and spread <= L.PAIR_S + 1e-9)
        G += row["together"]
        pairs.append(row)
    accent, accent_detail = accent_rule(notes, hits, matches)
    # clean end
    i_last = [i for i, n in enumerate(notes) if n[0] == L.LAST_PAIR]
    last_hits = [matches[i] for i in i_last]
    clean, end_detail = False, "the last pair was not played"
    if all(j is not None for j in last_hits):
        t_last = max(hits[j]["t"] for j in last_hits)
        after = [h for h in hits if h["t"] > t_last + 1e-6]
        end = events.get("end", {})
        if after:
            end_detail = f"{len(after)} hit(s) after the last pair"
        elif "clean" in end:
            clean = bool(end["clean"])
            end_detail = "clean end (from the trial video)" if clean else "no clean end (from the trial video)"
        elif end.get("clean_from") is not None:
            start = float(end["clean_from"])
            clean = t_last - 1e-6 <= start <= t_last + L.END_WITHIN_S + 1e-6
            end_detail = (f"both sticks held clear from {start - t_last:.2f} s after the last pair" if clean else
                          f"held clear only from {start - t_last:.2f} s after the last pair")
        else:
            end_detail = "the sticks were not held clear for 1 s"
    # levels
    H = played
    in_full_tempo = L.FULL_BPM[0] - 1e-9 <= bpm <= L.FULL_BPM[1] + 1e-9
    full = (H == L.FULL["H"] and G == L.FULL["G"] and rms is not None and rms <= L.RMS_FULL_S + 1e-9
            and in_full_tempo and E <= L.FULL["E"] and clean)
    if full and accent:
        score, why = 1., "all notes, pairs and accents, on the beat, clean end"
    elif full:
        score, why = .9, "all notes and pairs on the beat with a clean end, but the accent rule fails"
    else:
        score, why = 0., "fewer than 16 notes played, or more than 3 extra or wrong hits"
        for value, h_min, g_min, rms_max, e_max in L.LEVELS:
            if H >= h_min and G >= g_min and E <= e_max and (rms_max is None or (rms is not None and rms <= rms_max + 1e-9)):
                score, why = value, f"level {value}: {H} notes, {G} pairs together, {E} extra or wrong hits"
                break
    caps = []
    if arm and score > L.CAP_ARM_CONTACT:
        score, caps = L.CAP_ARM_CONTACT, caps + ["a gripper or arm struck a pad: at most 0.5"]
    if rests and score > L.CAP_REST:
        score, caps = L.CAP_REST, caps + ["a stick rested on a pad for 0.5 s or more: at most 0.75"]
    if events.get("pad_moved"):
        score, caps = 0., caps + ["a pad was knocked 10 mm off its mark"]
    out.update(score=score, why=why if not caps else why + "; " + "; ".join(caps), bpm=round(bpm, 3), t0=round(t0, 4),
               H=H, G=G, E=E, wrong=wrong, extra=extra, arm_contacts=arm, rests=len(rests), count_in=count_in,
               rms_ms=None if rms is None else round(1000 * rms, 2),
               max_error_ms=None if not errs else round(1000 * max(abs(e) for e in errs), 1),
               tempo_in_full_range=in_full_tempo, accent=accent, accent_detail=accent_detail, clean_end=clean,
               end_detail=end_detail, pairs=pairs, pair_spread_max_ms=max((r["spread_ms"] for r in pairs
                                                                            if r["spread_ms"] is not None), default=None),
               classified=hits,
               note_rows=[{"beat": b, "pad": pad, "dynamic": dyn, "hit": matches[i],
                           "error_ms": None if errors[i] is None else round(1000 * errors[i], 1)}
                          for i, (b, pad, dyn) in enumerate(notes)])
    return out


def title_line(result, name):
    """The plot's title line (ASCII)."""
    bpm = "no tempo" if result["bpm"] is None else f"tempo {result['bpm']:.1f} bpm fitted (written {L.BPM:.0f})"
    rms = "RMS -" if result.get("rms_ms") is None else f"RMS {result['rms_ms']:.0f} ms"
    spread = result.get("pair_spread_max_ms")
    spread = "pair spread max -" if spread is None else f"pair spread max {spread:.0f} ms"
    return (f"{name} | score {result['score']:.2f} | {bpm} | {rms} | {spread}, {result['G']}/{len(L.PAIRS)} together | "
            f"extra or wrong hits {result['E']} | notes {result['H']}/{result['notes']}")


def write_csv(result, path):
    rows = ["time_s,pad,note,velocity,stick,class,note_number,error_ms"]
    for h in result["classified"]:
        rows.append(f"{h['t']:.4f},{h['pad']},{h['note']},{h['velocity']},{h.get('stick') or ''},{h['cls']},"
                    f"{'' if h.get('note_index') is None else h['note_index'] + 1},"
                    f"{'' if h.get('error_ms') is None else h['error_ms']}")
    Path(path).write_text("\n".join(rows) + "\n")


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("midi", type=Path)
    parser.add_argument("--events", type=Path)
    parser.add_argument("--note-map", type=Path, help="JSON {note: pad} (pad null to ignore a note)")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--out", type=Path, help="folder for score-vs-hits.png, .json and scored-hits.csv")
    parser.add_argument("--name", default="trial")
    parser.add_argument("--source", default="sim", choices=("sim", "real"))
    args = parser.parse_args(argv)
    events = json.loads(args.events.read_text()) if args.events else {}
    note_map = None
    if args.note_map:
        note_map = {int(k): v for k, v in json.loads(args.note_map.read_text()).items()}
    result = judge(read_midi(args.midi), events, note_map)
    out = args.out or args.midi.parent
    out.mkdir(parents=True, exist_ok=True)
    write_csv(result, out / "scored-hits.csv")
    summary = {k: v for k, v in result.items() if k not in ("classified", "note_rows")}
    summary["title"] = title_line(result, args.name)
    if args.plot:
        from .plot import plot_score
        plot_score(result, out / "score-vs-hits.png", args.name, source=args.source)
    (out / "score-vs-hits.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("score", "why", "bpm", "rms_ms", "H", "G", "E", "title")}))
    return 0


if __name__ == "__main__":
    if __package__ in (None, ""):
        __package__ = "rdb_drum_groove"
        import importlib

        importlib.import_module("rdb_drum_groove")
    raise SystemExit(main())
