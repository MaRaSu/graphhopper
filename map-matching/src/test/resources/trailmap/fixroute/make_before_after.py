#!/usr/bin/env python3
"""Before/after review page for a /fix_route change: the OLD fix result (left) vs the NEW one (right).

Inputs: two EngineCompareExportTest output folders with the same leg files (old build switches vs new),
and an optional questions.json {"route:leg": {"why": ..., "question": ..., "focus": [lat, lng]}} in the
new folder.

Both maps: the saved route as a wide pale-blue band; the fix result as a red line with direction
arrows; added waypoints as black dots numbered in route order; unroutable stretches as orange bands;
the leg's own waypoints white. Flagged by my own checks, so the reviewer need not look for them: U-turns
at added waypoints (orange "U" with the metres ridden back) and edges ridden against their direction
(red "!"). Under each map: the check results and both quality measures (saved route left unfollowed;
route off the saved route when walked in order), for the fix and for today's route.

Cases are numbered in page order; the text names each case by route:leg. Local file only (real users'
routes).

Usage: python3 make_before_after.py <old_dir> <new_dir> <out_html>
"""
import glob
import json
import math
import os
import sys

old_dir, new_dir, out = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2]), sys.argv[3]
qfile = os.path.join(new_dir, 'questions.json')
questions = json.load(open(qfile)) if os.path.exists(qfile) else {}


def dist(a, b):
    return math.hypot((b[0] - a[0]) * 111320 * math.cos(math.radians(a[1])), (b[1] - a[1]) * 111320)


def length(c):
    return sum(dist(c[i - 1], c[i]) for i in range(1, len(c)))


def arrows(coords, max_count=200):
    """Points along a [lng, lat] line with their bearing [deg], for direction arrows."""
    total = length(coords)
    if total <= 0:
        return []
    step = max(60.0, total / max_count)
    out, d, nxt = [], 0.0, step / 2
    for i in range(1, len(coords)):
        a, b = coords[i - 1], coords[i]
        s = dist(a, b)
        while s > 0 and d + s >= nxt:
            t = (nxt - d) / s
            lng, lat = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            brg = math.degrees(math.atan2((b[0] - a[0]) * math.cos(math.radians(a[1])), b[1] - a[1])) % 360
            out.append([lat, lng, round(brg)])
            nxt += step
        d += s
    return out


def side(fc):
    feats = fc['features']
    final = next((x for x in feats if x['properties']['name'] == 'match_final'), None)
    pts = lambda n: [x for x in feats if x['properties']['name'] == n]
    return {
        'final': final,
        'arrows': arrows(final['geometry']['coordinates']) if final else [],
        'wps': [x['geometry']['coordinates'] for x in pts('match_new_waypoint')],
        'vias': [x['geometry']['coordinates'] for x in pts('via_point')],
        'today': next((x for x in feats if x['properties']['name'] == 'before'), None),
        'context': pts('context'),
        'orange': [x for x in feats if x['properties']['name'].startswith('match_unroutable')],
        'orange_m': [round(length(x['geometry']['coordinates'])) for x in feats if x['properties']['name'].startswith('match_unroutable')],
        'uturns': [[x['geometry']['coordinates'], x['properties'].get('offset_m')] for x in pts('uturn')],
        'wrongway': [x['geometry']['coordinates'] for x in pts('wrongway')],
        'checks': fc['properties'].get('checks', {}),
    }


items = []
for f in sorted(glob.glob(os.path.join(new_dir, '*.geojson'))):
    name = os.path.basename(f)
    of = os.path.join(old_dir, name)
    if not os.path.exists(of):
        continue
    new, old = json.load(open(f)), json.load(open(of))
    p = new['properties']
    q = questions.get(f"{p['route']}:{p['leg']}")
    if q is None and questions:
        continue  # a questions file lists exactly the cases to show
    q = q or {}
    saved = next(x for x in new['features'] if x['properties']['name'] == 'saved')
    fixed = [x['geometry']['coordinates'] for x in new['features'] if x['properties']['name'] == 'fixed_waypoint']
    o, n = side(old), side(new)
    focus = q.get('focus')
    if not focus and not q.get('fit'):  # where the sides differ: a flagged U-turn, else the first orange or waypoint
        for s in (o, n):
            for c, _ in s['uturns']:
                focus = focus or [c[1], c[0]]
        for s in (o, n):
            for x in s['orange']:
                c = x['geometry']['coordinates']
                focus = focus or [c[len(c) // 2][1], c[len(c) // 2][0]]
            for c in s['wps']:
                focus = focus or [c[1], c[0]]
    items.append({'route': p['route'], 'leg': p['leg'], 'saved': saved, 'fixed': fixed, 'old': o, 'new': n,
                  'focus': focus, 'why': q.get('why', ''), 'question': q.get('question', 'Is the RIGHT result better or equal? (Yes / No)'),
                  'order': q.get('order', 99)})
items.sort(key=lambda it: (it['order'], it['route'], it['leg']))

html = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>Fix before and after</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>
 body{margin:0;font:14px system-ui,sans-serif;height:100vh;display:flex;flex-direction:column}
 header{padding:8px 12px;background:#f6f8fa;border-bottom:1px solid #ddd}
 main{flex:1;display:flex;min-height:0}
 #list{width:190px;overflow:auto;border-right:1px solid #ccc}
 .it{padding:8px 10px;border-bottom:1px solid #eee;cursor:pointer}
 .it:hover,.it.sel{background:#eef3ff}
 .maps{flex:1;display:flex;min-width:0}
 .pane{flex:1;display:flex;flex-direction:column;min-width:0;border-left:1px solid #ccc}
 .pane h3{margin:0;padding:6px 10px;font-size:14px;background:#fff;border-bottom:1px solid #eee}
 .checks{padding:4px 10px;font-size:12px;background:#fafafa;border-bottom:1px solid #eee}
 .map{flex:1}
 .badge{padding:1px 6px;border-radius:3px;margin-left:6px;font-weight:normal}
 .key{display:inline-block;width:22px;vertical-align:middle;margin:0 4px 0 12px}
 .arr{color:#d1242f;font-size:14px;line-height:14px;text-shadow:0 0 2px #fff}
 .wpn{background:#000;color:#fff;border-radius:9px;width:18px;height:18px;line-height:18px;text-align:center;font:bold 11px system-ui}
 .ut{background:#f08c00;color:#000;border:2px solid #000;border-radius:3px;padding:0 3px;font:bold 11px system-ui;white-space:nowrap}
 .ww{background:#d1242f;color:#fff;border-radius:3px;padding:0 4px;font:bold 12px system-ui}
 .via{background:#1f6feb;color:#fff;border:2px solid #fff;border-radius:2px;width:16px;height:16px;line-height:16px;text-align:center;font:bold 11px system-ui}
</style></head><body>
<header><div id="q" style="font-size:15px;margin-bottom:6px"></div>
 <span class="key" style="background:#7fb0ff;height:12px"></span>saved route
 <span class="key" style="background:#d1242f;height:4px"></span>route after the fix (&#10148; direction)
 <span class="key" style="background:#f08c00;height:12px"></span>reported unroutable
 &nbsp; <span class="wpn" style="display:inline-block">1</span> added waypoints in route order
 &nbsp; &#9675; the leg's own waypoints
 &nbsp; <span class="ut">U 30 m</span> U-turn at an added waypoint
 &nbsp; <span class="ww">!</span> ridden against its direction
 &nbsp; <span class="via" style="display:inline-block">1</span> via points (no new waypoint)
 <span class="key" style="background:#a000e0;height:4px"></span>today's route, no fix (purple)
 <span class="key" style="background:#00a3a3;height:3px"></span>neighbouring legs (teal)</header>
<main><div id="list"></div><div class="maps">
 <div class="pane"><h3>LEFT: before this change <span id="b1" class="badge"></span></h3><div id="c1" class="checks"></div><div id="m1" class="map"></div></div>
 <div class="pane"><h3>RIGHT: after this change <span id="b2" class="badge"></span></h3><div id="c2" class="checks"></div><div id="m2" class="map"></div></div>
</div></main>
<script>
const ITEMS = __DATA__;
const tiles = 'https://tiles.trailmap.fi/styles/mtb-trailmap-global-v2/512/{z}/{x}/{y}.png';
function mk(id){const m=L.map(id);L.tileLayer(tiles,{tileSize:512,zoomOffset:-1,maxZoom:20}).addTo(m);return m;}
const m1=mk('m1'), m2=mk('m2');
let syncing=false;
function sync(a,b){a.on('move',()=>{if(syncing)return;syncing=true;b.setView(a.getCenter(),a.getZoom(),{animate:false});syncing=false;});}
sync(m1,m2); sync(m2,m1);
let l1=null,l2=null;
function badge(el,s){const n=s.wps.length,v=(s.vias||[]).length;el.textContent=(n?n+' new waypoint'+(n>1?'s':''):'no new waypoints')
  +(v?' · '+v+' via point'+(v>1?'s':''):'')
  +(s.orange_m.length?' · unroutable '+s.orange_m.join(' m, ')+' m':' · nothing unroutable');
  el.style.background=n?'#000':'#e6e6e6'; el.style.color=n?'#fff':'#333';}
function km(m){return m<0?'-':(m>=1000?(m/1000).toFixed(1)+' km':Math.round(m)+' m');}
function checks(el,s){const c=s.checks||{};
  el.innerHTML=`U-turns at added waypoints: <b>${c.uturns??'-'}</b>${c.uturns?' ('+c.uturn_m+' m back)':''} · against direction: <b>${c.wrongway??'-'}</b>`
   +`<br>saved route left unfollowed: <b>${km(c.saved_left_m)}</b> (today ${km(c.saved_left_today_m)})`
   +` · route off the saved route, in order: <b>${km(c.walk_off_m)}</b> (today ${km(c.walk_off_today_m)})`;}
function draw(g,it,s){
  (s.context||[]).forEach(c=>{L.geoJSON(c,{style:{color:'#fff',weight:6,opacity:0.9}}).addTo(g);
    L.geoJSON(c,{style:{color:'#00a3a3',weight:3,opacity:1}}).addTo(g);});
  L.geoJSON(it.saved,{style:{color:'#7fb0ff',weight:14,opacity:0.55}}).addTo(g);
  if(s.today){L.geoJSON(s.today,{style:{color:'#fff',weight:7,opacity:0.9}}).addTo(g);
    L.geoJSON(s.today,{style:{color:'#a000e0',weight:4,opacity:1}}).addTo(g);}
  s.orange.forEach(u=>L.geoJSON(u,{style:{color:'#f08c00',weight:14,opacity:0.8}}).addTo(g));
  if(s.final) L.geoJSON(s.final,{style:{color:'#d1242f',weight:4,opacity:1}}).addTo(g);
  s.arrows.forEach(a=>L.marker([a[0],a[1]],{interactive:false,icon:L.divIcon({className:'',iconSize:[14,14],iconAnchor:[7,7],
    html:`<div class="arr" style="transform:rotate(${a[2]-90}deg)">&#10148;</div>`})}).addTo(g));
  it.fixed.forEach(c=>L.circleMarker([c[1],c[0]],{radius:6,color:'#000',weight:2,fillColor:'#fff',fillOpacity:1}).addTo(g));
  s.wps.forEach((c,k)=>L.marker([c[1],c[0]],{icon:L.divIcon({className:'',iconSize:[18,18],iconAnchor:[9,9],html:`<div class="wpn">${k+1}</div>`})}).addTo(g));
  (s.vias||[]).forEach((c,k)=>L.marker([c[1],c[0]],{icon:L.divIcon({className:'',iconSize:[20,20],iconAnchor:[10,10],html:`<div class="via">${k+1}</div>`})}).addTo(g));
  s.uturns.forEach(u=>L.marker([u[0][1],u[0][0]],{icon:L.divIcon({className:'',iconSize:[60,18],iconAnchor:[-12,9],html:`<span class="ut">U ${u[1]} m</span>`})}).addTo(g));
  s.wrongway.forEach(c=>L.marker([c[1],c[0]],{icon:L.divIcon({className:'',iconSize:[14,18],iconAnchor:[7,9],html:'<span class="ww">!</span>'})}).addTo(g));
}
function show(i){
  document.querySelectorAll('.it').forEach((e,k)=>e.classList.toggle('sel',k===i));
  const it=ITEMS[i];
  document.getElementById('q').innerHTML=`<b>${i+1}. ${it.route}:${it.leg} — ${it.question}</b><br><i>Why shown:</i> ${it.why}`;
  badge(document.getElementById('b1'),it.old); badge(document.getElementById('b2'),it.new);
  checks(document.getElementById('c1'),it.old); checks(document.getElementById('c2'),it.new);
  if(l1){m1.removeLayer(l1);m2.removeLayer(l2);}
  syncing=true;
  if(it.focus) m1.setView(it.focus,16); else m1.fitBounds(L.geoJSON(it.saved).getBounds(),{padding:[30,30]});
  m2.setView(m1.getCenter(),m1.getZoom());
  syncing=false;
  l1=L.layerGroup().addTo(m1); l2=L.layerGroup().addTo(m2);
  draw(l1,it,it.old); draw(l2,it,it.new);
}
ITEMS.forEach((it,i)=>{const d=document.createElement('div');d.className='it';
  d.innerHTML=`<b>${i+1}. ${it.route}:${it.leg}</b><br><i>wp ${it.old.wps.length} → ${it.new.wps.length}`
    +` · U-turns ${(it.old.checks||{}).uturns??'-'} → ${(it.new.checks||{}).uturns??'-'}</i>`;
  d.onclick=()=>show(i);document.getElementById('list').appendChild(d);});
if(ITEMS.length) show(0);
</script></body></html>"""
open(out, 'w').write(html.replace('__DATA__', json.dumps(items)))
print(f'{len(items)} legs -> {out}')
