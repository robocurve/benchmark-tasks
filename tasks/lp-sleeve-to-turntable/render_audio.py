"""Render the modeled audio output from recorded native playback state.

This is original synthesized music, not acoustic groove simulation. Silence is
mandatory unless the trace reports actual stylus contact and measured rotation.
The source camera video stays unchanged; the result is a separate audio version.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import wave
import numpy as np

RATE = 48000
NOTES = (60, 64, 67, 72, 69, 67, 64, 62, 60, 64, 67, 71, 69, 65, 62, 60)


def audio_frames(rows):
    phase = 0.
    for row in rows:
        samples = np.zeros(RATE // 20)
        if row['metrics']['music_playing']:
            t = phase + np.arange(len(samples)) / RATE
            for i, time in enumerate(t):
                beat = time / .25
                note = NOTES[int(beat) % len(NOTES)]
                hz = 440 * 2 ** ((note - 69) / 12)
                within = time % .25
                envelope = min(1., within / .008) * np.exp(-within * 10)
                samples[i] = .22 * envelope * (np.sin(2 * np.pi * hz * time) + .25 * np.sin(4 * np.pi * hz * time))
            phase += len(samples) / RATE
        yield (samples * 32767).astype('<i2').tobytes()


def render(directory, output):
    rows = [json.loads(line) for line in (directory / 'trace.jsonl').read_text().splitlines()]
    wav = output.with_suffix('.wav')
    with wave.open(str(wav), 'wb') as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(RATE)
        for frame in audio_frames(rows):
            stream.writeframesraw(frame)
    subprocess.run(['ffmpeg', '-y', '-v', 'error', '-i', str(directory / 'top_cam.mp4'), '-i', str(wav),
                    '-c:v', 'copy', '-c:a', 'aac', '-b:a', '128k', '-shortest', '-movflags', '+faststart', str(output)], check=True)
    output.with_suffix('.audio.json').write_text(json.dumps(dict(
        scope='Modeled electronics audio rendered from native trace; original synthesized phrase',
        first_audible_step=next((r['step'] for r in rows if r['metrics']['music_playing']), None),
        native_frames=len(rows), sample_rate=RATE,
        source_video_sha256=hashlib.sha256((directory/'top_cam.mp4').read_bytes()).hexdigest(),
        source_trace_sha256=hashlib.sha256((directory/'trace.jsonl').read_bytes()).hexdigest(),
        output_sha256=hashlib.sha256(output.read_bytes()).hexdigest()), indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    render(args.directory, args.output)
