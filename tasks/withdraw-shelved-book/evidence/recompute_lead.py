"""Recompute time-aligned evidence from an extracted local task archive, using only Python."""
import argparse
import hashlib
import json
from pathlib import Path


def ok(values,conditions):
    return all(c['lo']<=values[c['metric']]<=c['hi'] for c in conditions)


def excursion(spec,rows,extra):
    signal=spec['metric']
    sign=1 if spec['direction']=='increase' else -1
    origin=at_origin=crossing=None
    for i,row in enumerate(rows):
        pos=sign*row['m'][signal]
        if origin is None or pos<=origin+spec['noise']:
            origin=pos if origin is None else min(origin,pos)
            at_origin=i
        elif pos-origin>=spec['travel']-1e-12:
            crossing=i
            break
    item={'metric':signal,'direction':spec['direction'],'travel':spec['travel'],'noise':spec['noise'],
          'onsetAlternatives':spec['onset_any'],'continuousAlternatives':spec['during_any'],
          'confirmed':crossing is not None}
    if crossing is None:
        return item
    channels=sorted({signal,*extra,*[rule['metric'] for group in spec['onset_any']+spec['during_any'] for rule in group]})
    origin_row=rows[at_origin]
    onset=[j+1 for j,conditions in enumerate(spec['onset_any']) if ok(origin_row['m'],conditions)]
    seq=[]
    for i in range(max(0,at_origin-2),min(len(rows),crossing+3)):
        row=rows[i];v=row['m']
        eligible=[j+1 for j,conditions in enumerate(spec['during_any']) if ok(v,conditions)]
        seq.append({'step':row['step'],'directedTravelFromExtreme':sign*v[signal]-origin,
                    'values':{k:v[k] for k in channels},'satisfiedContinuousAlternatives':eligible})
    held=all(not spec['during_any'] or any(ok(row['m'],a) for a in spec['during_any'])
             for row in rows[at_origin+1:crossing+1])
    ready=set(spec['needs'])<=set(rows[max(0,at_origin-1)]['milestones'])
    item.update(lastOriginSample=origin_row['step'],firstBeyondNoise=rows[at_origin+1]['step'],
                firstSignificantTravelSample=rows[crossing]['step'],satisfiedOriginAlternatives=onset,
                everyMovingSampleSatisfied=held,eligibleAtOrigin=ready,passed=bool(onset and ready and held),
                contiguousSamples=seq)
    return item


def summarize(fixture,stage,path):
    report=json.loads((path/'result.json').read_bytes())
    raw=(path/'trace.jsonl').read_bytes()
    trace=[json.loads(x) for x in raw.splitlines()]
    rows=[{'m':r['grade']['measurements'],'milestones':r['grade']['milestones'],
           'invalid':r['grade']['invalid'],'step':r['step']} for r in trace]
    full=[t for t in fixture['tiers'] if t['score']==1]
    windows=[]
    for tier in full:
        end=rows[-tier['ticks']:]
        ranges={}
        for cond in tier['all']:
            k=cond['metric'];values=[r['m'][k] for r in end]
            ranges[k]={'min':min(values),'max':max(values),'lo':cond['lo'],'hi':cond['hi'],
                       'passedEverySample':all(cond['lo']<=v<=cond['hi'] for v in values)}
        prereq=all(set(tier['needs'])<=set(r['milestones']) for r in end)
        valid=not any(r['invalid'] for r in end)
        windows.append({'first':end[0]['step'],'last':end[-1]['step'],'samples':len(end),
                        'requiredSamples':tier['ticks'],'requiredMilestones':tier['needs'],
                        'prerequisitesEverySample':prereq,'invalidationFree':valid,'observedRanges':ranges,
                        'passed':len(end)==tier['ticks'] and prereq and valid and
                                 all(v['passedEverySample'] for v in ranges.values())})
    if report['score']==1 and not any(w['passed'] for w in windows):
        raise ValueError(f'{stage}: cannot reproduce final full-score dwell from measurements')
    initial=report.get('initialMeasurements',{})
    history=([{'m':initial,'milestones':[],'invalid':False,'step':0}]+rows) if initial else rows
    contact={m['name']:m for m in fixture.get('metrics',[]) if m['kind']=='contact_n'}
    headcap=[k for k,m in contact.items() if m['b']=='crown' and m['a'] in ('left_tcp','right_tcp')]
    additional=[k for k in ('in_l','in_r','gap_l','gap_r','ll','lr','ll_rev','lr_rev','near_l',*headcap)
                if k in rows[0]['m']]
    events={spec['name']:excursion(spec,history,additional) for spec in fixture.get('excursions',[])}
    for name,event in events.items():
        if event.get('confirmed') and event.get('passed')!=trace[-1]['grade']['excursions'][name]['outcome']:
            raise ValueError(f'{stage}: independent first-motion calculation disagrees for {name}')
    return {'score':report['score'],'steps':report['steps'],'elapsedSeconds':report['elapsed_s'],
            'traceSha256':hashlib.sha256(raw).hexdigest(),'headcapAllEpisodePeakN':{
               k:max(r['m'][k] for r in history) for k in headcap},
            'events':events,'endOfEpisodeFullScoreRoutes':windows}


def lead_evidence(fixture,stage_dir,stages):
    return {'schemaVersion':1,'units':'Positions in m, contact normal force magnitudes in N, angles in degrees.',
            'coordinateSignals':fixture.get('metrics',[]),'channelSums':fixture.get('contact_totals',{}),
            'wholeEpisodeInvalidatingConditions':fixture['invalidate'],
            'stages':{s:summarize(fixture,s,stage_dir/s) for s in stages}}


def published_book_notes(original,fixture,lead):
    """Current public measurement guide for the source with physically separate floor and wall sensors."""
    sensors={m['name']:m for m in fixture['metrics']}
    if not all(sensors[k]['b']=='shelf_floor' for k in ('bearing_l','bearing_r')):
        raise ValueError('The measured book description requires separate shelf-floor reporters')
    first=lead['stages']['scripted-first']
    event=first['events']['opposed_pull'];window=next(w for w in first['endOfEpisodeFullScoreRoutes'] if w['passed'])
    origin=next(i['values'] for i in event['contiguousSamples'] if i['step']==event['lastOriginSample'])
    last=next(i['values'] for i in event['contiguousSamples'] if i['step']==event['firstSignificantTravelSample'])
    out=original.split('## Measurement record')[0].split('\n',1)[-1].strip()
    out+='\n\nThe fixed plywood plinth has its own rigid-body contact reporter named shelf_floor. Its mass and the wall assembly mass sum to the shelf mass listed above. The side and back walls belong to the separate bay body.\n'
    out+='\n## Scoring and measured contacts\n\n'
    out+=(f"At control sample {event['lastOriginSample']}, the tan books had moved inward by {origin['in_l']*1000:.2f} mm and {origin['in_r']*1000:.2f} mm. "
          f"The left gripper's opposing pads already loaded the two cover regions at {origin['ll']:.3f} N and {origin['lr']:.3f} N. "
          f"Outward travel first exceeded the numerical noise band at sample {event['firstBeyondNoise']} and reached 3 mm at sample {event['firstSignificantTravelSample']}, "
          f"where the two measured pad forces were {last['ll']:.3f} N and {last['lr']:.3f} N. Opposing contact above 0.10 N per pad and the mid-spine grasp-distance condition held at every intervening sample. ")
    out+=('The scorer accepts either gripper and either pad ordering. It checks both neighbouring setbacks and gaps before the first significant outward motion and monitors headcap loading throughout the episode. '
          f"The left and right arm-to-headcap peaks in this demonstration were {first['headcapAllEpisodePeakN']['lh']:.3f} N and {first['headcapAllEpisodePeakN']['rh']:.3f} N. A reading of 0.05 N or more invalidates a trial.\n")
    out+=('\nDuring the final two seconds, bearing_l measures tan_left against shelf_floor and bearing_r measures tan_right against shelf_floor. '
          'Wall reactions are excluded from these two force measurements. Both neighbours also have to stand upright with their bottom edges at the floor and their corners in the bay. '
          'The four reactions from the blue core, each cover and the crown against the mat are summed at each sample. The required total is at least 0.10 N. '
          'The blue book must lie flat, with all eight physical corners inside the mat, all three books quiet and both grippers disengaged and clear.\n')
    ranges=window['observedRanges']
    out+=f"\nAll {len(ranges)} final measurement bounds and the two onset prerequisites held throughout samples {window['first']}–{window['last']}. "
    out+='The ranges below cover all forty consecutive control samples.\n\n| Measurement | Minimum | Maximum |\n|---|---:|---:|\n'
    for name,label,scale in [('book_on_mat','Total book-to-mat reaction (N)',1),('bearing_l','Left neighbour on floor (N)',1),
               ('bearing_r','Right neighbour on floor (N)',1),('up','Book up-axis angle (degrees)',1),
               ('depth_angle','Book depth-axis angle (degrees)',1),('height','Book centre height (mm)',1000),
               ('left_book','Left hand to book centre (mm)',1000),('right_book','Right hand to book centre (mm)',1000)]:
        out+=f"| {label} | {ranges[name]['min']*scale:.4f} | {ranges[name]['max']*scale:.4f} |\n"
    out+=('\nThe full condition ranges, continuous onset samples, per-episode headcap peaks, sensor definitions and trace hashes are in evidence/lead-evidence.json. '
          'From the extracted archive, run `python evidence/recompute_lead.py .` to reproduce them. '
          'Contact-force thresholds and the noise floor are engineering settings for the rigid simulator. A physical teaching fixture would require force-sensor calibration and a separate handling check.\n')
    return out


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('archive_root',nargs='?',default='.')
    root=Path(parser.parse_args().archive_root);manifest=json.loads((root/'evidence/manifest.json').read_bytes())
    fixture=json.loads((root/'task_impl/fixture.json').read_bytes())
    calculated=lead_evidence(fixture,root/'evidence',manifest['runs'])
    saved=json.loads((root/'evidence/lead-evidence.json').read_bytes())
    if calculated!=saved:
        raise SystemExit('Archived evidence does not reproduce')
    print('All stage evidence reproduced.')
