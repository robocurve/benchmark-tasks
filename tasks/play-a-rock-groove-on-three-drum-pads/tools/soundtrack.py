"""Add a drum soundtrack, synthesized from a run's logged hits, to its combined video, and check that sound and
picture line up. Nothing here is simulated sound: the simulation has none, and the robot hears nothing.

    python tools/soundtrack.py out/<run>            # writes out/<run>/media/drums.wav and out/<run>/demo.mp4
    python tools/soundtrack.py out/control-air combined.mp4      # a control's combined video

- The combined video (``media/top_cam.mp4`` and ``media/side_cam.mp4`` side by side, 20 fps, as the templates' README
  gives it) gets a caption strip along its bottom: "Sound added offline from the logged hits; not simulated; the
  robot heard nothing" (drawn with Pillow; the runtime's ffmpeg has no text filter).
- The sound is every note-on in ``media/hits.mid`` (every logged hit: wrong pads, extra hits and all), at its exact
  time (not quantized), its loudness and brightness set by its MIDI velocity. Each drum is synthesized here (no
  samples, no third-party sounds; MIT like the rest of this package): the closed hi-hat from six detuned square waves
  (the classic analog-drum-machine recipe) with a fast high-passed decay, the snare from two decaying sines and a
  noise burst, the tom from a falling sine with a soft attack, the snare's rim from a short click.
- Sync check: the muxed file's audio and video streams both start at 0, their lengths differ by less than a frame,
  and cross-correlating the decoded soundtrack with the rendered one finds them within 2 ms of each other; each hit's
  onset is then found in the decoded audio (``sync.json``).

Needs numpy and ffmpeg (the runtime's). The caption font is DejaVu Sans, bundled with matplotlib (DejaVu Fonts
licence).
"""

import json
import math
import shutil
import subprocess
import sys
import wave
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_drum_groove.midi import read_midi  # noqa: E402

SR = 48000
CAPTION = "Sound added offline from the logged hits; not simulated; the robot heard nothing"
FPS = 20


def _env(n, tau):
    return np.exp(-np.arange(n) / (tau * SR))


def _highpass(x, k=.95):
    y = np.empty_like(x)
    prev_x = prev_y = 0.
    for i, v in enumerate(x):
        prev_y = k * (prev_y + v - prev_x)
        prev_x = v
        y[i] = prev_y
    return y


def hihat(vel, rng):
    n = int(.09 * SR)
    t = np.arange(n) / SR
    tone = sum(np.sign(np.sin(2 * math.pi * f * t)) for f in (205.3, 304.4, 369.6, 522.7, 540., 800.)) / 6
    noise = rng.uniform(-1, 1, n)
    x = .6 * tone + .4 * noise
    x = _highpass(_highpass(x, .9), .9)
    return x * _env(n, .010 + .006 * vel / 127)


def snare(vel, rng):
    n = int(.26 * SR)
    t = np.arange(n) / SR
    body = (np.sin(2 * math.pi * 185 * t) + .6 * np.sin(2 * math.pi * 330 * t)) * _env(n, .045)
    noise = _highpass(rng.uniform(-1, 1, n), .8) * _env(n, .07)
    bright = .5 + .5 * vel / 127
    return .7 * body + bright * 1.1 * noise


def tom(vel, rng):
    n = int(.5 * SR)
    t = np.arange(n) / SR
    f = 88 + 30 * np.exp(-t / .05)
    phase = 2 * math.pi * np.cumsum(f) / SR
    body = np.sin(phase) * _env(n, .16)
    click = _highpass(rng.uniform(-1, 1, n), .7) * _env(n, .006)
    return body + .4 * click


def rim(vel, rng):
    n = int(.08 * SR)
    t = np.arange(n) / SR
    return (np.sin(2 * math.pi * 1700 * t) * _env(n, .012) + .5 * rng.uniform(-1, 1, n) * _env(n, .003))


VOICES = {42: hihat, 46: hihat, 23: hihat, 38: snare, 40: rim, 43: tom}


def render(note_ons, duration):
    rng = np.random.default_rng(0)
    out = np.zeros(int(math.ceil(duration * SR)) + SR)
    for t, note, vel in note_ons:
        voice = VOICES.get(note)
        if voice is None:
            continue
        x = voice(vel, rng) * (vel / 127.) ** 1.6
        i = int(round(t * SR))
        j = min(len(out), i + len(x))
        out[i:j] += x[:j - i]
    out = out[:int(round(duration * SR))]
    return .45 * out


def write_wav(path, x):
    pcm = np.clip(x, -1, 1)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes((pcm * 32767).astype("<i2").tobytes())


def read_wav(path):
    with wave.open(str(path), "rb") as w:
        data = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float) / 32767
        return data.reshape(-1, w.getnchannels()).mean(axis=1), w.getframerate()


def ffmpeg():
    exe = shutil.which("ffmpeg")
    if exe is None:
        import imageio_ffmpeg

        exe = imageio_ffmpeg.get_ffmpeg_exe()
    return exe


def probe(path):
    out = subprocess.run([shutil.which("ffprobe") or "ffprobe", "-v", "error", "-show_entries",
                          "stream=codec_type,start_time,duration,nb_frames,r_frame_rate,sample_rate,width,height",
                          "-of", "json", str(path)], capture_output=True, text=True, check=True)
    return json.loads(out.stdout)["streams"]


def font():
    import matplotlib

    return str(Path(matplotlib.get_data_path()) / "fonts" / "ttf" / "DejaVuSans.ttf")


def caption_png(path, width):
    """The caption as a strip to lay over the bottom of the video (the runtime's ffmpeg has no text filter)."""
    from PIL import Image, ImageDraw, ImageFont

    height = 52
    img = Image.new("RGBA", (width, height), (0, 0, 0, 140))
    draw = ImageDraw.Draw(img)
    face = ImageFont.truetype(font(), 26)
    w = draw.textlength(CAPTION, font=face)
    draw.text(((width - w) / 2, 11), CAPTION, font=face, fill=(255, 255, 255, 255))
    img.save(path)


def make(run, name="demo.mp4"):
    run = Path(run)
    media = run / "media"
    note_ons = read_midi(media / "hits.mid")
    silent = run / "demo-silent.mp4"
    ff = ffmpeg()
    subprocess.run([ff, "-y", "-loglevel", "error", "-i", str(media / "top_cam.mp4"), "-i", str(media / "side_cam.mp4"),
                    "-filter_complex", "[0:v]scale=1280:704:flags=neighbor[t];[t][1:v]hstack=inputs=2",
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(silent)], check=True)
    if name == "demo.mp4":                  # the task card's screenshot: the first frame, overhead and side, no caption
        subprocess.run([ff, "-y", "-loglevel", "error", "-i", str(silent), "-frames:v", "1", str(run / "setup.png")],
                       check=True)
    video = next(s for s in probe(silent) if s["codec_type"] == "video")
    frames = int(video["nb_frames"])
    duration = frames / FPS
    track = render(note_ons, duration)
    write_wav(media / "drums.wav", track)
    caption = media / "caption.png"
    caption_png(caption, int(video["width"]))
    final = run / name
    subprocess.run([ff, "-y", "-loglevel", "error", "-i", str(silent), "-i", str(media / "drums.wav"),
                    "-loop", "1", "-i", str(caption),
                    "-filter_complex", "[0:v][2:v]overlay=0:H-h:shortest=1[v]", "-map", "[v]", "-map", "1:a",
                    "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-r", str(FPS), "-c:a", "aac", "-b:a", "192k", "-ar", str(SR),
                    "-t", f"{duration:.3f}", "-movflags", "+faststart", str(final)], check=True)
    silent.unlink()
    caption.unlink()
    return check_sync(run, track, note_ons, duration, name)


def check_sync(run, track, note_ons, duration, name="demo.mp4"):
    run = Path(run)
    final = run / name
    streams = probe(final)
    v = next(s for s in streams if s["codec_type"] == "video")
    a = next(s for s in streams if s["codec_type"] == "audio")
    decoded = run / "media" / "decoded.wav"
    subprocess.run([ffmpeg(), "-y", "-loglevel", "error", "-i", str(final), "-map", "0:a", "-ac", "1", "-ar", str(SR),
                    str(decoded)], check=True)
    x, sr = read_wav(decoded)
    decoded.unlink()
    n = min(len(x), len(track))
    lag_max = int(.1 * SR)
    ref = track[:n]
    best = None
    # cross-correlation by FFT
    size = 1 << int(math.ceil(math.log2(2 * n)))
    corr = np.fft.irfft(np.fft.rfft(x[:n], size) * np.conj(np.fft.rfft(ref, size)), size)
    lags = np.concatenate([np.arange(0, lag_max + 1), np.arange(-lag_max, 0)])
    vals = np.concatenate([corr[:lag_max + 1], corr[-lag_max:]])
    best = int(lags[int(np.argmax(vals))])
    # each hit's onset in the decoded audio: the first 1 ms frame within +-15 ms whose energy jumps
    env = np.sqrt(np.convolve(x * x, np.ones(48) / 48, mode="same"))
    onsets = []
    for t, note, vel in note_ons:
        i = int(round(t * SR))
        lo, hi = max(0, i - int(.015 * SR)), min(len(env), i + int(.015 * SR))
        if hi - lo < 10:
            continue
        seg = env[lo:hi]
        base = float(np.median(env[max(0, lo - 2400):lo])) if lo > 0 else 0.
        thresh = base + .3 * (seg.max() - base)
        k = int(np.argmax(seg > thresh))
        onsets.append(round(1000 * ((lo + k) / SR - t), 2))
    report = {"video_start_s": float(v.get("start_time", 0.)), "audio_start_s": float(a.get("start_time", 0.)),
              "video_duration_s": float(v["duration"]), "audio_duration_s": float(a["duration"]),
              "frames": int(v["nb_frames"]), "fps": v["r_frame_rate"], "sample_rate": a.get("sample_rate"),
              "xcorr_offset_ms": round(1000 * best / SR, 2), "hits": len(note_ons),
              "onset_offsets_ms": {"median": float(np.median(onsets)) if onsets else None,
                                   "min": min(onsets) if onsets else None, "max": max(onsets) if onsets else None},
              "caption": CAPTION}
    report["in_sync"] = bool(abs(report["xcorr_offset_ms"]) <= 2. and abs(report["video_start_s"]) < 1e-3
                             and abs(report["audio_start_s"]) < 1e-3
                             and abs(report["audio_duration_s"] - report["video_duration_s"]) <= 1. / FPS + 1e-3)
    (run / "media" / "sync.json").write_text(json.dumps(report, indent=1) + "\n")
    return report


if __name__ == "__main__":
    print(json.dumps(make(sys.argv[1], *sys.argv[2:3])))
