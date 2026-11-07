"""Official Dream policy playback through the current SDK bridge."""
from dataclasses import replace
from inspect_robots_dream.policy import DreamPolicy
from inspect_robots_dream.bridge import BridgeSession

class ClosingBridge(BridgeSession):
    def close(self):
        if self._episode:
            try:self._call('close',{'episode':self._episode},30.)
            finally:self._episode=''

def dream_current(server_url='http://127.0.0.1:8240',**kwargs):
    policy=DreamPolicy(session=ClosingBridge(server_url),**kwargs)
    policy.server_url=server_url
    policy.info=replace(policy.info,name='dream_current')
    return policy
