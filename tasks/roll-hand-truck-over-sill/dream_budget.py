"""Persistent, conservative request reservations around the official Dream client.

Credentials come from DREAM_API_KEY in the process environment. The ledger holds
no credentials, frames, or robot/object state. A reservation is never refunded,
including on failed requests, and covers the SDK's one transport retry.
"""
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import urllib.request

MODEL = 'dreamlabs/dream-1-beta'
MAX_ATTEMPTS = 178
RESERVE_USD = .02  # Two wire attempts at a conservative $0.01 each.
CAP_USD = 3.56


class BudgetExceeded(RuntimeError):
    pass


def reserve(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a+') as ledger:
        fcntl.flock(ledger, fcntl.LOCK_EX)
        ledger.seek(0)
        rows = [json.loads(line) for line in ledger if line.strip()]
        if len(rows) >= MAX_ATTEMPTS or round(sum(r['reserved_usd'] for r in rows)+RESERVE_USD, 8)>CAP_USD:
            raise BudgetExceeded('Persistent Dream request budget exhausted')
        row = {'attempt':len(rows)+1, 'reserved_usd':RESERVE_USD,
               'max_wire_requests':2, 'model':MODEL,
               'utc':datetime.now(timezone.utc).isoformat()}
        ledger.seek(0,2)
        ledger.write(json.dumps(row)+'\n')
        ledger.flush()
        os.fsync(ledger.fileno())
        return row


def verified_price():
    # The official platform's model listing is read-only. Fail closed if it no
    # longer publishes a finite, nonnegative price within the reservation.
    key = os.environ['DREAM_API_KEY']
    request = urllib.request.Request('https://platform.dreamlabs.ai/v1/models',
                                    headers={'Authorization':'Bearer '+key})
    with urllib.request.urlopen(request, timeout=30) as reply:
        models = json.load(reply)
    model = next(m for m in models if m['id']==MODEL)
    price = model['price_millicents']
    if not isinstance(price,(int,float)) or not 0<=price<=1000 or not model['servable']:
        raise BudgetExceeded('Model unavailable or current price exceeds the reserved unit price')
    return price/100000


class GuardedClient:
    def __init__(self, ledger):
        from dream import DreamClient
        self.price_usd = verified_price()
        self.ledger = Path(ledger)
        self.client = DreamClient()

    def __getattr__(self, name):
        return getattr(self.client,name)

    def get_action(self, *, commanded=None):
        reserve(self.ledger)
        return self.client.get_action(commanded=commanded)


def make_policy(ledger):
    from inspect_robots_dream.policy import DreamPolicy
    from inspect_robots_dream.session import DreamSession
    session = DreamSession(make_client=lambda:GuardedClient(ledger))
    return DreamPolicy(model=MODEL,session=session,record=False,timeout_s=90.)
