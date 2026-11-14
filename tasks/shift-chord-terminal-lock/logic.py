"""Debounced key events with a real future-inclusive modifier sampling window."""
import numpy as np
import geometry as g

def new_ledger():
    return dict(step=0,buffer='',accepted=False,locked=False,locked_after=False,
                rejected=False,wipe=False,fixture_fault=False,submitted=False,
                down={k:False for k in g.KEYS},pending=[],recent=[],hist=[],events=[])

def _commit(ledger,key,event):
    if ledger['locked'] or event['void']:
        return
    shift,ctrl=event['shift'],event['ctrl']
    if ctrl:
        if key=='erase':
            ledger['wipe']=True
        elif key=='l':
            ledger['locked']=True
            ledger['locked_after']=ledger['accepted']
        return
    if key=='enter':
        ledger['submitted']=True
        if ledger['buffer']==g.CODE:
            ledger['accepted']=True
        else:
            ledger['rejected']=True
    elif key=='erase':
        ledger['buffer']=ledger['buffer'][:-1]
    elif len(ledger['buffer'])<3:
        char='&' if key=='7' and shift else key.upper() if shift else key
        ledger['buffer']+=char

def advance(ledger,positions,fixture_fault=False):
    ledger['step']+=1
    step=ledger['step']
    values={k:max(0.,-float(positions.get(k,positions.get('digit7',0.) if k=='7' else 0.))) for k in g.KEYS}
    ledger['fixture_fault'] |= fixture_fault
    ledger['recent'].append((step,{k:values[k]>=g.DOWN for k in g.MODIFIERS}))
    del ledger['recent'][:max(0,len(ledger['recent'])-(2*g.WINDOW+1))]
    for key in g.KEYS:
        if not ledger['down'][key] and values[key]>=g.DOWN:
            ledger['down'][key]=True
            if key in g.NONMODS:
                ledger['pending'].append(dict(key=key,t=step,up=False,void=False,shift=False,ctrl=False))
        elif ledger['down'][key] and values[key]<g.UP:
            ledger['down'][key]=False
            for event in reversed(ledger['pending']):
                if event['key']==key and not event['up']:
                    event['up']=True
                    break
    active=[k for k in g.NONMODS if ledger['down'][k]]
    if len(active)>1:
        for event in ledger['pending']:
            if event['key'] in active and not event['up']:
                event['void']=True
    for event in list(ledger['pending']):
        for t,mods in ledger['recent']:
            if abs(t-event['t'])<=g.WINDOW:
                for mod in g.MODIFIERS:
                    event[mod] |= mods[mod]
        if event['up'] and step>=event['t']+g.WINDOW:
            _commit(ledger,event['key'],event)
            ledger['events'].append(event.copy())
            ledger['pending'].remove(event)
    ledger['hist'].append(np.array([values[k] for k in g.KEYS]))
    del ledger['hist'][:-g.STILL_STEPS]

def still(ledger):
    return bool(len(ledger['hist'])==g.STILL_STEPS and np.ptp(ledger['hist'],axis=0).max()<=g.STILL_M)

def all_up(positions):
    return all(max(0.,-float(positions.get(k,positions.get('digit7',0.) if k=='7' else 0.)))<g.UP for k in g.KEYS)

def evaluate_state(ledger,keys_up,released,is_still):
    if ledger['rejected'] or ledger['wipe'] or ledger['fixture_fault']:
        return 0.,False,'permanent rejection, erase chord or moved fixture'
    if ledger['accepted']:
        if ledger['locked_after']:
            if keys_up and released and is_still:
                return 1.,True,'exact code accepted, secured afterwards, released and still'
            return .75,False,'accepted and secured; finishing conditions missing'
        return .5,False,'accepted but unsecured'
    if ledger['locked'] or (not ledger['submitted'] and ledger['buffer'].startswith(g.CODE[:2])):
        return .25,False,'correct prefix or secured before entry'
    return 0.,False,'no accepted entry'
