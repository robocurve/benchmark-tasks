"""A process-safe ceiling on committed API tariff costs, in exact decimal USD."""

from contextlib import contextmanager
from decimal import Decimal
from pathlib import Path
import fcntl
import json
import os
import time


class BudgetExceeded(RuntimeError):
    pass


class BudgetLedger:
    def __init__(self, path, limit_usd="10.00"):
        self.path = Path(path)
        self.limit = Decimal(str(limit_usd))
        if not self.limit.is_finite() or self.limit < 0:
            raise ValueError("budget must be finite and nonnegative")

    @contextmanager
    def _locked(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o600)
        with os.fdopen(descriptor, "r+") as file:
            os.chmod(self.path, 0o600)
            fcntl.flock(file.fileno(), fcntl.LOCK_EX)
            try:
                file.seek(0)
                raw = file.read()
                state = json.loads(raw) if raw else {"limit_usd": str(self.limit), "committed_usd": "0", "requests": []}
                if Decimal(state["limit_usd"]) != self.limit:
                    raise ValueError("ledger budget differs from the configured ceiling")
                yield state
                file.seek(0)
                json.dump(state, file, indent=2)
                file.truncate()
                file.flush()
                os.fsync(file.fileno())
            finally:
                fcntl.flock(file.fileno(), fcntl.LOCK_UN)

    def commit_request(self, *, price_millicents, model):
        """Reserve the published tariff before issuing one model request.

        Reservations are deliberately not refunded on errors: a failed connection
        does not establish that the provider performed no billable work. This is
        a spending guard, not a claim about the provider's actual invoice.
        """
        value = Decimal(str(price_millicents))
        if not value.is_finite() or value < 0 or value != value.to_integral():
            raise ValueError("price_millicents must be a nonnegative integer")
        cost = value / Decimal(100000)
        with self._locked() as state:
            total = Decimal(state["committed_usd"]) + cost
            if total > self.limit:
                raise BudgetExceeded(f"request exceeds the ${self.limit} project budget")
            state["committed_usd"] = str(total)
            state["requests"].append({"time_unix": time.time(), "model": model,
                                      "tariff_usd": str(cost)})
        return cost

    def status(self):
        with self._locked() as state:
            return {"limit_usd": state["limit_usd"], "committed_usd": state["committed_usd"],
                    "request_count": len(state["requests"])}


def make_guarded_dream_policy(*, ledger_path, model="dreamlabs/dream-1-beta", **kwargs):
    """Use the actual Dream client, reserving each request before sending it."""
    import urllib.request
    from dream import DreamClient, credentials
    from inspect_robots_dream.policy import DreamPolicy
    from inspect_robots_dream.session import DreamSession

    ledger = BudgetLedger(ledger_path)
    resolved = credentials.load()
    req = urllib.request.Request(resolved.platform_url+"/v1/models",
                                 headers={"Authorization": "Bearer "+resolved.api_key})
    with urllib.request.urlopen(req, timeout=30) as response:
        listing = json.load(response)
    rows = listing if isinstance(listing, list) else listing.get("data", listing.get("models", []))
    metadata = next((row for row in rows if row.get("id") == model), None)
    if metadata is None or "price_millicents" not in metadata:
        raise RuntimeError("cannot run paid inference without current model tariff metadata")

    class GuardedClient:
        def __init__(self):
            self.inner = DreamClient()

        def __getattr__(self, name):
            return getattr(self.inner, name)

        def get_action(self, **options):
            ledger.commit_request(price_millicents=metadata["price_millicents"], model=model)
            return self.inner.get_action(**options)

    session = DreamSession(make_client=GuardedClient)
    return DreamPolicy(model=model, session=session, **kwargs)

