#!/usr/bin/env python3
"""Interactive analytics app for report evidence + live demo.

A self-contained web app (Python standard library + Chart.js from CDN - no
extra installs) that visualises everything the other tools produce:

  - incident outcomes (pie) and per-room counts (bar)
  - detection class counts + confidence histogram (from annotate_detections.py)
  - inference latency / FPS summary
  - FSM escalation timeline (from capture_sim_evidence.py)
  - a gallery of the annotated detection + escalation frames
  - the raw incident table

Reads, if present:
  ~/compliance_robot_logs/incidents.jsonl   (live/sim incidents)
  evidence/output/summary.json + detections.csv
  evidence/output/sim/run.json + incidents.json
  evidence/output/annotated/*  and  evidence/output/sim/*.jpg

Run ON THE LAPTOP, then open http://localhost:8090 :

  python3 evidence/analytics_app.py
  python3 evidence/analytics_app.py --port 8090
"""

import argparse
import csv
import json
import os
from collections import Counter
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'output')
SIM = os.path.join(OUT, 'sim')
INCIDENTS = os.path.expanduser('~/compliance_robot_logs/incidents.jsonl')

# Image galleries: label -> directory
GALLERIES = {'annotated': os.path.join(OUT, 'annotated'), 'sim': SIM,
             'eval': os.path.join(OUT, 'eval')}


def read_jsonl(path):
    rows = []
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            for line in f:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    pass
    return rows


def read_json(path, default):
    if os.path.isfile(path):
        try:
            with open(path, encoding='utf-8') as f:
                return json.load(f)
        except ValueError:
            pass
    return default


def gather():
    incidents = read_jsonl(INCIDENTS) + read_json(os.path.join(SIM, 'incidents.json'), [])
    outcomes = Counter(i.get('outcome', 'unknown') for i in incidents)
    rooms = Counter((i.get('room') or i.get('room_id') or 'unknown') for i in incidents)
    events = Counter(i.get('event_class', 'unknown') for i in incidents)

    summary = read_json(os.path.join(OUT, 'summary.json'), {})
    class_counts = summary.get('class_counts', {})

    confs = []
    csv_path = os.path.join(OUT, 'detections.csv')
    if os.path.isfile(csv_path):
        with open(csv_path) as f:
            for r in csv.DictReader(f):
                try:
                    confs.append(float(r['confidence']))
                except (KeyError, ValueError):
                    pass
    bins = [0] * 10
    for c in confs:
        bins[min(int(c * 10), 9)] += 1

    run = read_json(os.path.join(SIM, 'run.json'), {})

    images = {}
    for label, d in GALLERIES.items():
        if os.path.isdir(d):
            images[label] = sorted(n for n in os.listdir(d)
                                   if n.lower().endswith(('.jpg', '.jpeg', '.png')))
    return {
        'incidents': incidents[-500:],
        'outcomes': outcomes, 'rooms': rooms, 'events': events,
        'class_counts': class_counts,
        'conf_bins': bins,
        'detection_summary': {
            'images': summary.get('images'),
            'total_detections': summary.get('total_detections'),
            'mean_latency_ms': summary.get('mean_latency_ms'),
            'mean_fps': summary.get('mean_fps'),
            'primary_model': summary.get('primary_model'),
            'smoking_model': summary.get('smoking_model'),
        },
        'fsm_timeline': run.get('fsm_timeline', []),
        'images': images,
    }


PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Compliance Robot - Evidence Analytics</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
<style>
 :root{--bg:#0f1420;--card:#19202f;--ink:#e8edf6;--mut:#9aa7bd;--ac:#3d7bff}
 *{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);
   font:14px/1.5 system-ui,Segoe UI,Roboto,sans-serif}
 header{padding:18px 26px;background:linear-gradient(90deg,#16203a,#0f1420);
   border-bottom:1px solid #232c40}
 h1{margin:0;font-size:18px}.sub{color:var(--mut);font-size:12px;margin-top:3px}
 .wrap{padding:22px;max-width:1200px;margin:0 auto}
 .grid{display:grid;gap:18px;grid-template-columns:repeat(auto-fit,minmax(330px,1fr))}
 .card{background:var(--card);border:1px solid #232c40;border-radius:14px;padding:16px}
 .card h2{margin:0 0 10px;font-size:13px;text-transform:uppercase;letter-spacing:.4px;
   color:var(--mut)}
 .kpis{display:flex;gap:14px;flex-wrap:wrap;margin-bottom:18px}
 .kpi{background:var(--card);border:1px solid #232c40;border-radius:12px;padding:14px 18px;
   min-width:130px}.kpi b{display:block;font-size:22px}.kpi span{color:var(--mut);font-size:12px}
 table{width:100%;border-collapse:collapse;font-size:12.5px}
 th,td{text-align:left;padding:6px 8px;border-bottom:1px solid #232c40}
 th{color:var(--mut);font-weight:600}
 .gal{display:grid;gap:8px;grid-template-columns:repeat(auto-fill,minmax(150px,1fr))}
 .gal figure{margin:0}.gal img{width:100%;border-radius:8px;border:1px solid #232c40;cursor:zoom-in}
 .gal figcaption{color:var(--mut);font-size:11px;margin-top:3px;word-break:break-all}
 .full{grid-column:1/-1}
 #lb{position:fixed;inset:0;background:rgba(0,0,0,.85);display:none;align-items:center;
   justify-content:center;z-index:9}#lb img{max-width:92%;max-height:92%;border-radius:8px}
</style></head><body>
<header><h1>Compliance Robot &mdash; Evidence Analytics</h1>
<div class=sub id=meta>loading&hellip;</div></header>
<div class=wrap>
 <div class=kpis id=kpis></div>
 <div class=grid>
  <div class=card><h2>Incident outcomes</h2><canvas id=cOut></canvas></div>
  <div class=card><h2>Incidents per room</h2><canvas id=cRoom></canvas></div>
  <div class=card><h2>Detections per class</h2><canvas id=cClass></canvas></div>
  <div class=card><h2>Detection confidence</h2><canvas id=cConf></canvas></div>
  <div class="card full"><h2>Escalation FSM timeline (simulation)</h2><canvas id=cFsm height=90></canvas></div>
  <div class="card full"><h2>Annotated detections</h2><div class=gal id=galA></div></div>
  <div class="card full"><h2>Escalation frames (simulation)</h2><div class=gal id=galS></div></div>
  <div class="card full"><h2>Incident log</h2><div style=overflow:auto><table id=tbl></table></div></div>
 </div>
</div>
<div id=lb onclick="this.style.display='none'"><img id=lbi></div>
<script>
const C={ac:'#3d7bff',gr:'#15a36a',pu:'#7b2dd6',rd:'#d83b3b',or:'#e6a23c',gy:'#6b778c'};
Chart.defaults.color='#9aa7bd';Chart.defaults.borderColor='#232c40';
function pie(id,obj){const k=Object.keys(obj),v=k.map(x=>obj[x]);
 new Chart(document.getElementById(id),{type:'doughnut',
  data:{labels:k,datasets:[{data:v,backgroundColor:[C.gr,C.rd,C.or,C.pu,C.ac,C.gy]}]},
  options:{plugins:{legend:{position:'bottom'}}}});}
function bar(id,obj,col){const k=Object.keys(obj),v=k.map(x=>obj[x]);
 new Chart(document.getElementById(id),{type:'bar',
  data:{labels:k,datasets:[{data:v,backgroundColor:col}]},
  options:{plugins:{legend:{display:false}},scales:{y:{beginAtZero:true}}}});}
function gallery(el,label,names){el.innerHTML=names.map(n=>
 `<figure><img loading=lazy src="/img/${label}/${n}" onclick="lb(this.src)">`+
 `<figcaption>${n}</figcaption></figure>`).join('')||'<span style=color:#6b778c>none yet</span>';}
function lb(s){document.getElementById('lbi').src=s;document.getElementById('lb').style.display='flex';}
fetch('/api/data').then(r=>r.json()).then(d=>{
 const ds=d.detection_summary,inc=d.incidents;
 document.getElementById('meta').textContent=
  `${inc.length} incidents | ${ds.total_detections||0} detections | `+
  `${ds.mean_fps?ds.mean_fps+' FPS':'n/a'} | model ${ds.primary_model||'-'}`;
 const kpi=[['Incidents',inc.length],['Detections',ds.total_detections||0],
  ['Images',ds.images||0],['Mean FPS',ds.mean_fps||'-'],
  ['Latency ms',ds.mean_latency_ms||'-']];
 document.getElementById('kpis').innerHTML=kpi.map(k=>
  `<div class=kpi><b>${k[1]}</b><span>${k[0]}</span></div>`).join('');
 pie('cOut',d.outcomes);bar('cRoom',d.rooms,C.ac);bar('cClass',d.class_counts,C.gr);
 const cb={};d.conf_bins.forEach((v,i)=>cb[(i/10).toFixed(1)]=v);bar('cConf',cb,C.pu);
 // FSM timeline as a stepped line
 const tl=d.fsm_timeline;if(tl.length){const order=[];tl.forEach(p=>{if(!order.includes(p[1]))order.push(p[1]);});
  const pts=tl.map(p=>({x:p[0],y:order.indexOf(p[1])}));
  new Chart(document.getElementById('cFsm'),{type:'line',
   data:{datasets:[{data:pts,stepped:true,borderColor:C.ac,pointBackgroundColor:C.rd,pointRadius:4}]},
   options:{plugins:{legend:{display:false}},scales:{
    x:{type:'linear',title:{display:true,text:'time (s)'}},
    y:{ticks:{callback:v=>order[v]||'',stepSize:1},min:0,max:order.length-0.5}}}});}
 gallery(document.getElementById('galA'),'annotated',(d.images.annotated||[]));
 gallery(document.getElementById('galS'),'sim',(d.images.sim||[]).filter(n=>n.endsWith('.jpg')));
 const cols=['timestamp','room','event_class','stage_reached','outcome','confidence'];
 document.getElementById('tbl').innerHTML='<tr>'+cols.map(c=>`<th>${c}</th>`).join('')+'</tr>'+
  inc.slice().reverse().map(r=>'<tr>'+cols.map(c=>`<td>${r[c]??r[c==='room'?'room_id':'']??''}</td>`).join('')+'</tr>').join('');
});
</script></body></html>"""


def make_handler(data_fn):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == '/' or self.path.startswith('/index'):
                body = PAGE.encode()
                self._send(body, 'text/html; charset=utf-8')
            elif self.path == '/api/data':
                d = data_fn()
                payload = dict(d)
                for k in ('outcomes', 'rooms', 'events'):
                    payload[k] = dict(d[k])
                self._send(json.dumps(payload).encode(), 'application/json')
            elif self.path.startswith('/img/'):
                self._serve_image(self.path[len('/img/'):])
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def _serve_image(self, rel):
            label, _, name = rel.partition('/')
            base = GALLERIES.get(label)
            if not base or '..' in name or name.startswith('/'):
                self.send_error(HTTPStatus.NOT_FOUND); return
            path = os.path.join(base, name)
            if not os.path.isfile(path):
                self.send_error(HTTPStatus.NOT_FOUND); return
            with open(path, 'rb') as f:
                self._send(f.read(), 'image/jpeg')

        def _send(self, body, ctype):
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8090)
    args = ap.parse_args()
    httpd = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(gather))
    print(f'Evidence analytics app: http://localhost:{args.port}')
    print('Reading:')
    print(f'  incidents : {INCIDENTS}')
    print(f'  detection : {os.path.join(OUT, "summary.json")}')
    print(f'  sim run   : {os.path.join(SIM, "run.json")}')
    print('Ctrl+C to stop.')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
