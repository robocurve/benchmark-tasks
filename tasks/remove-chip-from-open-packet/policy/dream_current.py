"""Official policy playback with an explicit per-trial SDK session summary."""
from dataclasses import replace
from pathlib import Path
import json,atexit,time
from inspect_robots_dream.policy import DreamPolicy
from inspect_robots_dream.bridge import BridgeSession

class ClosingBridge(BridgeSession):
    def start(self,episode):
        super().start(episode)
        (Path.cwd()/'dream_episode.json').write_text(json.dumps({'episode':self._episode,'server_url':self.url,'started_at':time.time()},indent=2)+'\n')
    def close(self):
        if self._episode:
            try:
                summary=self._call('summary',{'episode':self._episode},30.)
                (Path.cwd()/'dream_session.json').write_text(json.dumps(summary,indent=2)+'\n')
            finally:
                try:self._call('close',{'episode':self._episode},30.)
                finally:self._episode=''

def dream_current(server_url='http://127.0.0.1:8241',**kwargs):
    session=ClosingBridge(server_url)
    atexit.register(session.close)
    policy=DreamPolicy(session=session,**kwargs)
    policy.server_url=server_url
    policy.info=replace(policy.info,name='dream_current')
    return policy
