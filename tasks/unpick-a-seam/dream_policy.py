"""Normal Dream adapter with an auditable, uninterrupted server episode.

Credentials are resolved by dreamlabs-client itself from DREAM_API_KEY or its
existing credential file (DREAM_CREDENTIALS / ~/.dreamlabs/credentials). They
are never copied into run artifacts. A replaced episode is a failed execution.
"""
from __future__ import annotations

import time
import warnings


def continuous_client_class(base, expired_error, request_audit, session_audit):
    """Injectable for offline tests; retain session IDs only inside the client."""
    class ContinuousDreamClient(base):
        def get_action(self, *, commanded=None):
            started = time.monotonic()
            previous = getattr(self, "_last_success_at", None)
            before = self.session_id
            event = {"request": len(request_audit)+1,
                     "idle_wall_seconds": None if previous is None else started-previous}
            request_audit.append(event)
            try:
                reply = super().get_action(commanded=commanded)
                after = self.session_id
                if after is not None:
                    seen = getattr(self, "_audit_seen_sessions", set())
                    seen.add(after)
                    self._audit_seen_sessions = seen
                    session_audit["session_count"] = len(seen)
                if before is not None and after != before:
                    event["status"] = "session_replaced"
                    raise RuntimeError("Dream session changed; reply will not execute")
            except expired_error:
                event["status"] = "session_replaced"
                # DreamSession retries its own expiry type. Stop before that retry.
                raise RuntimeError("Dream session expired or was replaced; trial stopped") from None
            except Exception as error:
                event.setdefault("status", "error")
                event["error_type"] = type(error).__name__
                raise
            else:
                self._last_success_at = time.monotonic()
                event.update(status="success", session_changed=False)
                return reply
            finally:
                event["wall_seconds"] = time.monotonic()-started
    return ContinuousDreamClient


def make_policy(request_audit, session_audit, *, rows_per_request=8):
    import dream
    from inspect_robots_dream import DreamPolicy
    from inspect_robots_dream.session import DreamSession
    warnings.filterwarnings("ignore", category=dream.DreamSlowNetworkWarning)
    client = continuous_client_class(dream.DreamClient, dream.DreamSessionExpired,
                                     request_audit, session_audit)
    return DreamPolicy(rows_per_request=rows_per_request,
                       session=DreamSession(make_client=client))


def public_transcript(policy):
    """Remove private server identifiers without hiding failures or timings."""
    transcript = policy.transcript()
    transcript.pop("server_url", None)
    transcript["requests"] = [{k: v for k, v in row.items() if k != "session_id"}
                               for row in transcript.get("requests", [])]
    return transcript
