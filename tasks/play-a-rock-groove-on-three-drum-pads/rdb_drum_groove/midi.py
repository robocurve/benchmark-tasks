"""A minimal Standard MIDI File writer and reader (format 0 and 1), standard library only.

The simulated trial writes its hits as the drum module would send them: one note-on per hit on channel 10 (General
MIDI percussion), velocity from the hit's speed, and a note-off 50 ms later; 960 ticks a beat with a tempo of
500,000 us a beat (the file's own clock, not the groove's), so one tick is about 0.52 ms.
"""

import struct

PPQ = 960
TEMPO_US = 500000
CHANNEL = 9                      # channel 10
NOTE_LEN_S = .050


def _varlen(n):
    out = [n & 0x7F]
    n >>= 7
    while n:
        out.append((n & 0x7F) | 0x80)
        n >>= 7
    return bytes(reversed(out))


def write_midi(path, note_ons):
    """``note_ons``: [(time s, note, velocity)]. Returns the bytes written."""
    ticks_per_s = PPQ * 1e6 / TEMPO_US
    events = []
    for t, note, vel in note_ons:
        if t < 0:
            raise ValueError(f"a note-on before the file's start: {t} s")
        on = int(round(t * ticks_per_s))
        events.append((on, 1, 0x90 | CHANNEL, int(note), int(vel)))
        events.append((on + int(round(NOTE_LEN_S * ticks_per_s)), 0, 0x80 | CHANNEL, int(note), 0))
    events.sort()
    track = bytearray(b"\x00\xFF\x51\x03" + TEMPO_US.to_bytes(3, "big"))
    now = 0
    for tick, _, status, note, vel in events:
        track += _varlen(tick - now) + bytes((status, note, vel))
        now = tick
    track += b"\x00\xFF\x2F\x00"
    data = b"MThd" + struct.pack(">IHHH", 6, 0, 1, PPQ) + b"MTrk" + struct.pack(">I", len(track)) + bytes(track)
    with open(path, "wb") as f:
        f.write(data)
    return data


def read_midi(path):
    """Note-ons [(time s, note, velocity)] of a format 0 or 1 file (tempo changes honoured), sorted by time."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"MThd":
        raise ValueError("not a Standard MIDI File")
    _, _fmt, ntrk, division = struct.unpack(">IHHH", data[4:14])
    if division & 0x8000:
        raise ValueError("SMPTE time division is not supported")
    raw, pos = [], 14
    for _ in range(ntrk):
        if data[pos:pos + 4] != b"MTrk":
            raise ValueError("bad track header")
        length = struct.unpack(">I", data[pos + 4:pos + 8])[0]
        p, end, tick, status = pos + 8, pos + 8 + length, 0, 0
        while p < end:
            delta = 0
            while True:
                b = data[p]
                p += 1
                delta = (delta << 7) | (b & 0x7F)
                if not b & 0x80:
                    break
            tick += delta
            b = data[p]
            if b == 0xFF:                                   # meta event
                kind = data[p + 1]
                p += 2
                ln = 0
                while True:
                    c = data[p]
                    p += 1
                    ln = (ln << 7) | (c & 0x7F)
                    if not c & 0x80:
                        break
                if kind == 0x51:
                    raw.append((tick, 0, int.from_bytes(data[p:p + 3], "big"), None))
                p += ln
                continue
            if b in (0xF0, 0xF7):                           # sysex
                p += 1
                ln = 0
                while True:
                    c = data[p]
                    p += 1
                    ln = (ln << 7) | (c & 0x7F)
                    if not c & 0x80:
                        break
                p += ln
                continue
            if b & 0x80:
                status = b
                p += 1
            kind = status & 0xF0
            if kind in (0xC0, 0xD0):
                p += 1
                continue
            a, c = data[p], data[p + 1]
            p += 2
            if kind == 0x90 and c > 0:
                raw.append((tick, 1, a, c))
        pos = end
    raw.sort(key=lambda r: (r[0], r[1]))
    out, tempo, last_tick, seconds = [], TEMPO_US, 0, 0.
    for tick, kind, a, c in raw:
        seconds += (tick - last_tick) * tempo / 1e6 / division
        last_tick = tick
        if kind == 0:
            tempo = a
        else:
            out.append((seconds, a, c))
    return out
