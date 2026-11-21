"""Read-only backing-contact event history. No forces or object states written."""
import re


class BackingContacts:
    def __init__(self):
        self.active={};self.last_seen={};self.first_seen={};self.last_loaded={};self.load_peaks={};self.transitions=[]

    def record(self,paths,event,interval_start,points,impulse_norm_sum=None):
        pen=next((re.search(r'pen_(\d+)/Barrel$',p) for p in paths if re.search(r'pen_(\d+)/Barrel$',p)),None)
        cap=next((re.search(r'cap_(\d+)/LinerBacking\d+$',p) for p in paths if re.search(r'cap_(\d+)/LinerBacking\d+$',p)),None)
        if pen is None or cap is None:return
        if impulse_norm_sum is None:
            impulse_norm_sum=sum(sum(float(v)**2 for v in p.get('impulse',[]))**.5 for p in points)
        loaded=impulse_norm_sum>1e-8
        pair=f'{pen[1]}_{cap[1]}';key=tuple(sorted(paths))
        row=dict(pair=pair,colliders=list(key),event=event,control_interval_start_s=float(interval_start),points=points,impulse_norm_sum_Ns=float(impulse_norm_sum),loaded=loaded,
                 timestamp_scope='Event delivered within following control interval; not an exact substep timestamp.')
        if event=='lost':self.active.pop(key,None)
        else:
            self.active[key]=row;self.last_seen[pair]=float(interval_start)
            self.first_seen.setdefault(pair,float(interval_start))
            if loaded:
                self.last_loaded[pair]=float(interval_start)
                self.load_peaks[pair]=max(self.load_peaks.get(pair,0.),float(impulse_norm_sum))
        if event in ('found','lost'):self.transitions.append(row)

    def snapshot(self,now):
        result={}
        for pair,last in self.last_seen.items():
            active=[row for row in self.active.values() if row['pair']==pair]
            result[pair]=dict(active=bool(active),recent=now-last<=.1000001,loaded=any(r['loaded'] for r in active),recent_loaded=now-self.last_loaded.get(pair,float('-inf'))<=.1000001,last_report_interval_start_s=last,last_loaded_interval_start_s=self.last_loaded.get(pair),peak_impulse_norm_sum_Ns=self.load_peaks.get(pair,0.),
                              first_report_interval_start_s=self.first_seen[pair],
                              active_colliders=[r['colliders'] for r in active],latest_reports=active)
        return result

    def drain(self):
        rows=self.transitions;self.transitions=[];return rows
