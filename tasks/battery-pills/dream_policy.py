"""Dream-1 adapter for ring-on-finger (copied from build/pillowcase/dream_policy.py = peg-out-a-tshirt's recoverable variant).

Model: dreamlabs/dream-1-beta (inspect_robots_dream DEFAULT_MODEL, the launchable README's `--policy dream`).

Credentials are resolved by dreamlabs-client itself (DREAM_API_KEY or its own credential file).
Nothing here reads, copies or logs them. Every server-session recovery is recorded.
"""
from __future__ import annotations

import time
import warnings

MAX_EXPIRIES = 12


def recoverable_client_class(base, expired_error, request_audit, session_audit):
    """Keep the official SDK retry path, recording server-session recoveries."""
    class AuditedDreamClient(base):
        def get_action(self, *, commanded=None):
            started, before = time.monotonic(), self.session_id
            event = {'request': len(request_audit) + 1}
            request_audit.append(event)
            try:
                reply = super().get_action(commanded=commanded)
                after = self.session_id
                seen = getattr(self, '_audit_seen_sessions', set())
                if after is not None:
                    seen.add(after)
                self._audit_seen_sessions = seen
                session_audit['session_count'] = len(seen)
                event.update(status='success', session_changed=before is not None and after != before)
                return reply
            except expired_error:
                event['status'] = 'session_expired_retryable'
                session_audit['expiry_count'] = session_audit.get('expiry_count', 0) + 1
                if session_audit['expiry_count'] > MAX_EXPIRIES:
                    raise RuntimeError('Repeated Dream service expiry exceeded the bounded recovery allowance') from None
                raise
            except Exception as error:
                event.update(status='error', error_type=type(error).__name__)
                raise
            finally:
                event['wall_seconds'] = time.monotonic() - started
    return AuditedDreamClient


MODEL = 'dreamlabs/dream-1-beta'


def make_policy(request_audit, session_audit, *, rows_per_request=2):
    import dream
    from inspect_robots_dream import DreamPolicy
    from inspect_robots_dream.session import DreamSession
    warnings.filterwarnings('ignore', category=dream.DreamSlowNetworkWarning)
    client = recoverable_client_class(dream.DreamClient, dream.DreamSessionExpired, request_audit, session_audit)
    return DreamPolicy(model=MODEL, rows_per_request=rows_per_request, session=DreamSession(make_client=client))


def public_transcript(policy):
    """Remove private server identifiers without hiding failures or timings."""
    transcript = policy.transcript()
    transcript.pop('server_url', None)
    transcript['requests'] = [{k: v for k, v in row.items() if k != 'session_id'} for row in transcript.get('requests', [])]
    return transcript
