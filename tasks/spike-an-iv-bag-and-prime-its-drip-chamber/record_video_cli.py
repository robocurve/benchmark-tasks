"""Run Inspect Robots unchanged, recording top-camera observations directly to H.264.

Set IV_VIDEO_DIR to an empty output directory. Simulation and policy observations
are unchanged. Native camera dimensions and every emitted frame are checked.
"""
import json
import os
from pathlib import Path
import subprocess
from importlib.metadata import entry_points
import numpy as np
from PIL import Image
from yam_arena.inspect_embodiment import YamArenaEmbodiment


class Recorder:
    def __init__(self, output):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        if (self.output / 'run.mp4').exists():
            raise FileExistsError('Refusing to overwrite a recording')
        self.process = None
        self.times = []

    def record(self, embodiment, observation):
        t = float(observation.state_time)
        if self.times and t == self.times[-1]:
            return
        if self.times and t < self.times[-1]:
            raise RuntimeError('A recording must contain exactly one trial')
        frame = np.ascontiguousarray(observation.images['top_cam'])
        native = embodiment._base.scene['top_cam'].data.output['rgb']
        if tuple(native.shape[1:3]) != (720,1280) or frame.shape != (720,1280,3):
            raise ValueError(f'Expected native 1280x720; got sensor {native.shape}, observation {frame.shape}')
        if self.process is None:
            Image.fromarray(frame).save(self.output / 'setup.png')
            self.process = subprocess.Popen([
                'ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
                '-s','1280x720','-r',str(embodiment.info.control_hz),'-i','-',
                '-an','-c:v','libx264','-preset','veryfast','-crf','20',
                '-pix_fmt','yuv420p','-movflags','+faststart',str(self.output/'run.mp4')],stdin=subprocess.PIPE)
        self.process.stdin.write(frame.tobytes())
        self.times.append(t)

    def close(self):
        if self.process is not None:
            self.process.stdin.close()
            if self.process.wait() != 0:
                raise RuntimeError('Video encoding failed')
            (self.output/'recording.json').write_text(json.dumps({
                'camera':'top_cam','native_resolution':[1280,720],
                'frames':len(self.times),'first_time':self.times[0],
                'last_time':self.times[-1],'simulation_times':self.times},indent=2))


if __name__ == '__main__':
    recorder = Recorder(os.environ['IV_VIDEO_DIR'])
    original = YamArenaEmbodiment._observation
    def observation(self):
        result = original(self)
        recorder.record(self,result)
        return result
    YamArenaEmbodiment._observation = observation
    try:
        next(iter(entry_points(group='console_scripts').select(name='inspect-robots'))).load()()
    finally:
        recorder.close()
