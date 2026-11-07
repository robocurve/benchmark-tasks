"""Loopback bridge using the documented Dream observation names and client IK."""
import json,threading,time,uuid
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import numpy as np
import dream

sessions={};sessions_lock=threading.Lock()
def start(body):
    client=dream.DreamClient()
    client.policy({'checkpoint':body['model'],'ik':{'arm':'yam','head':'joint'},
                   'action_hz':.001,'obs_window':9,'obs_fps':5,'timeout_ms':45000})
    key=uuid.uuid4().hex
    with sessions_lock:sessions[key]={'client':client,'lock':threading.Lock(),'last_seen':time.monotonic(),'timeout':body.get('timeout_s',300.)}
    return {'episode':key}

def route(path,body):
    if path=='/check':
        with dream.DreamClient() as client:
            available={model.id for model in client.get_models() if model.servable}
        if body['model'] not in available:raise ValueError('Checkpoint is not available')
        return {}
    if path=='/start':return start(body)
    key=body.get('episode')
    with sessions_lock:session=sessions.get(key)
    if session is None:raise ValueError('Unknown bridge episode')
    with session['lock']:
        session['last_seen']=time.monotonic();client=session['client']
        if path=='/close':
            client.close()
            with sessions_lock:sessions.pop(key,None)
            return {}
        if path=='/observe':
            q=np.asarray(body['state'],dtype=float)
            if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('Invalid joint state')
            frames={new:body['frames'][old] for new,old in [('video.overhead','top_cam'),('video.wrist_l','left_cam'),('video.wrist_r','right_cam')]}
            state={'prompt':body['prompt'],'state.left_arm':q[:6].tolist(),'state.left_gripper':q[6:7].tolist(),
                   'state.right_arm':q[7:13].tolist(),'state.right_gripper':q[13:14].tolist()}
            client.set_observation(frames,state,timestamp=float(body['timestamp']))
            return {}
        if path=='/act':
            last=body.get('commanded');commanded=None
            if last is not None:
                last=np.asarray(last,dtype=float)
                if last.shape!=(14,) or not np.isfinite(last).all():raise ValueError('Invalid commanded state')
                commanded={'q_l_chunk':last[:7],'q_r_chunk':last[7:]}
            began=time.monotonic();deadline=began+session['timeout'];refused=0
            for attempt in range(1,25):
                try:reply=client.get_action(commanded=commanded);break
                except (dream.DreamSessionExpired,dream.DreamTimeout) as error:
                    if time.monotonic()>deadline:raise
                    print('Retrying',type(error).__name__,flush=True)
                    continue
                except dream.DreamProtocolError:
                    if time.monotonic()+1.>deadline:raise
                    time.sleep(1.)
                    continue
                except dream.DreamServerError as error:
                    refused=refused+1 if error.reason=='error' else 0
                    wait=error.retry_after_s or 1.
                    if not error.retryable or refused>=3 or time.monotonic()+wait>deadline:raise
                    time.sleep(wait)
            else:raise RuntimeError('API retry budget exhausted')
            left=np.array(reply['q_l_chunk'],dtype=float);right=np.array(reply['q_r_chunk'],dtype=float)
            if left.ndim!=2 or left.shape!=right.shape or left.shape[1]!=7 or not 1<=len(left)<=48:raise ValueError('Invalid action chunk shape')
            rows=np.hstack([left,right])
            if not np.isfinite(rows).all():raise ValueError('Nonfinite action')
            if np.any(rows[:,[6,13]]<-.05) or np.any(rows[:,[6,13]]>1.05):raise ValueError('Invalid gripper convention')
            rows[:,[6,13]]=np.clip(rows[:,[6,13]],0.,1.)
            return {'rows':rows.tolist(),'rtt_s':time.monotonic()-began,'attempts':attempt,
                    'infer_ms':(client.last_timing or {}).get('serve_infer_ms'),'session_id':None}
        raise ValueError('Unknown route')

class Handler(BaseHTTPRequestHandler):
    def reply(self,status,body):
        data=json.dumps(body).encode();self.send_response(status)
        self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)))
        self.end_headers();self.wfile.write(data)
    def do_GET(self):self.reply(200 if self.path=='/health' else 404,{'ok':self.path=='/health'})
    def do_POST(self):
        try:
            length=int(self.headers.get('Content-Length',0))
            if not 0<length<8_000_000:raise ValueError('Invalid request size')
            body=json.loads(self.rfile.read(length))
            self.reply(200,route(self.path,body))
        except Exception as error:
            self.reply(500,{'error':type(error).__name__,'message':getattr(error,'reason',None) or 'Bridge request failed'})
    def log_message(self,*args):pass

if __name__=='__main__':
    print('Current SDK bridge listening on loopback port 8240',flush=True)
    ThreadingHTTPServer(('127.0.0.1',8240),Handler).serve_forever()
