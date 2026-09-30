#!/usr/bin/env python3
"""Side-by-side review page for /fix_route (EngineCompareExportTest output).

Two maps that move together. LEFT: the client's route today (before any fix). RIGHT: the route after
the new engine's fix. On both: the saved route as a wide pale-blue band, the route as a solid red
line on top — compare red against blue. Right only: added waypoints (black dots) and stretches the
server reports as unroutable (orange band). Leg start / end: white dots. One yes/no question per
example. Local file only (real users' routes).

Usage: python3 make_side_by_side.py <compare_dir>
"""
import glob
import json
import os
import sys

d = os.path.abspath(sys.argv[1])
# Boundary-review items (EngineCompareExportTest label): what the server decided, in words.
BOUNDARY = {
    'fixed_near': 'Today\'s route is up to {peak:.0f} m from the saved route (threshold 30 m): '
                  'the server FIXES it ({n} added waypoint(s), right map).',
    'unroutable_near': 'Today\'s route is up to {peak:.0f} m from the saved route (threshold 30 m), '
                       'and no waypoint makes it follow: the server reports the orange stretch as unroutable.',
    'left_near': 'Today\'s route is up to {peak:.0f} m from the saved route, under the 30 m threshold: '
                 'the server LEAVES it as it is (right map = left map).',
    'ok_edges': 'The saved line is up to {peak:.0f} m from today\'s route, but the map matching says it takes '
                'the same roads (the saved line is drawn off them): the server LEAVES it as it is.',
}
stats = {}
legs_file = os.path.join(d, '..', 'fix-matching', 'legs.jsonl')
if os.path.exists(legs_file):
    for line in open(legs_file):
        j = json.loads(line)
        stats[(j['route'], j['leg'])] = j
items = []
for f in sorted(glob.glob(os.path.join(d, '*.geojson'))):
    fc = json.load(open(f))
    feats = {x['properties']['name']: x for x in fc['features'] if x['geometry']['type'] == 'LineString'}
    saved = feats.get('saved')
    before = feats.get('before')
    after = feats.get('match_final') or before
    un = [x for x in fc['features'] if x['properties']['name'].startswith('match_unroutable')]
    wps = [x for x in fc['features'] if x['properties']['name'] == 'match_new_waypoint']
    ends = [saved['geometry']['coordinates'][0], saved['geometry']['coordinates'][-1]] if saved else []
    pts = lambda name: [x['geometry']['coordinates'] for x in fc['features'] if x['properties']['name'] == name]
    fixed_wps, snaps = pts('fixed_waypoint'), pts('waypoint_snap')
    suggested = [{'c': x['geometry']['coordinates'], 'offset': x['properties'].get('offset_m')}
                 for x in fc['features'] if x['properties']['name'] == 'snap_suggested']
    if fixed_wps:
        ends = fixed_wps
    focus = None
    for x in [{'geometry': {'type': 'Point', 'coordinates': s['c']}} for s in suggested] + un + wps:
        c = x['geometry']['coordinates']
        c = c[len(c) // 2] if x['geometry']['type'] == 'LineString' else c
        focus = [c[1], c[0]]
        break
    st = stats.get((fc['properties']['route'], fc['properties']['leg']), {})
    n = len(wps)
    facts = f'Server added {n} waypoint{"s" if n != 1 else ""} (black dots).' if n else 'Server added no waypoints.'
    if suggested:
        facts += (' It reports that a waypoint snaps onto a different way than the saved route: the grey tick'
                  ' runs from the waypoint (white) to where the app snaps it; the green dot is the suggested'
                  ' position on the saved route\'s road.')
        question = 'Is this what the fix should produce here, and is the green suggested waypoint position right?'
    elif un:
        parts = [f'{u["len"]:.0f} m (the way round today: {u["reroute"]:.0f} m)' for u in st.get('unroutable', [])]
        facts += ' It says today\'s map cannot follow the saved route on the orange stretch: ' + ', '.join(parts) + '.'
        question = ('Is the orange stretch really impossible to ride as saved on today\'s map '
                    '(way missing, closed, or one-way against you)?')
    else:
        question = ('Does the red line on the RIGHT now follow the blue band, and is every black dot '
                    'needed? (If a dot is not needed, say which.)')
    label = fc['properties'].get('label') or ''
    if label in BOUNDARY and before:
        # The engine's own measurement: its largest stretch (at least 25 m long), and where it is.
        import re
        import make_threshold_review as tr  # densify helper
        best = None
        for s in fc['properties'].get('deviations', []):
            m = re.match(r'(route|saved) (\d+)\.\.(\d+) peak (\d+)', s)
            if m and int(m.group(3)) - int(m.group(2)) >= 25 and (best is None or int(m.group(4)) > best[3]):
                best = (m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4)))
        if best:
            line = tr.densify((before if best[0] == 'route' else saved)['geometry']['coordinates'])
            c = line[min(len(line) - 1, (best[1] + best[2]) // 4)]  # 2 m samples
            focus = [c[1], c[0]]
        facts = BOUNDARY[label].format(peak=best[3] if best else 0, n=len(wps))
        question = 'Is the server\'s decision right? (Yes / No)'
    # Optional per-item question and "why shown" (questions.json in the export folder, key "route:leg").
    qfile = os.path.join(d, 'questions.json')
    custom = json.load(open(qfile)).get(f"{fc['properties']['route']}:{fc['properties']['leg']}") if os.path.exists(qfile) else None
    if custom:
        question = custom['question']
        facts = f'<i>Why shown:</i> {custom["why"]}<br>' + facts
    items.append({'route': fc['properties']['route'], 'leg': fc['properties']['leg'],
                  'saved': saved, 'before': before, 'after': after, 'unroutable': un, 'wps': wps,
                  'ends': ends, 'snaps': snaps, 'suggested': suggested, 'focus': focus, 'facts': facts,
                  'question': question})
# Stretch questions first, then waypoint questions; the page numbers them in this order.
items.sort(key=lambda it: (0 if it['unroutable'] else 1, it['route'], it['leg']))

html = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>Fix review</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>
 body{margin:0;font:14px system-ui,sans-serif;height:100vh;display:flex;flex-direction:column}
 header{padding:8px 12px;background:#f6f8fa;border-bottom:1px solid #ddd}
 header b{font-size:15px}
 main{flex:1;display:flex;min-height:0}
 #list{width:170px;overflow:auto;border-right:1px solid #ccc}
 .it{padding:8px 10px;border-bottom:1px solid #eee;cursor:pointer}
 .it:hover,.it.sel{background:#eef3ff}
 .maps{flex:1;display:flex;min-width:0}
 .pane{flex:1;display:flex;flex-direction:column;min-width:0;border-left:1px solid #ccc}
 .pane h3{margin:0;padding:6px 10px;font-size:14px;background:#fff;border-bottom:1px solid #eee}
 .map{flex:1}
 .key{display:inline-block;width:22px;height:8px;vertical-align:middle;margin:0 4px 0 12px}
</style></head><body>
<header><div id="q" style="font-size:15px;margin-bottom:6px"></div>
 Answer each number with <b>Yes</b> or <b>No</b> (+ a word why). Click a number on the left; the two maps move together.<br>
 <span class="key" style="background:#7fb0ff;height:10px"></span>saved route
 <span class="key" style="background:#d1242f;height:4px"></span>route (left: today, right: after fix)
 <span class="key" style="background:#f08c00;height:10px"></span>server says: today's roads cannot follow the saved route here
 &nbsp; &#9679; added waypoint &nbsp; &#9675; leg's own waypoint (grey tick: to where the app snaps it)
 &nbsp; <span style="color:#2da44e">&#9679;</span> suggested waypoint position</header>
<main><div id="list"></div><div class="maps">
 <div class="pane"><h3>LEFT: today, before any fix</h3><div id="m1" class="map"></div></div>
 <div class="pane"><h3>RIGHT: after the fix</h3><div id="m2" class="map"></div></div>
</div></main>
<script>
const ITEMS = __DATA__;
const tiles = 'https://tiles.trailmap.fi/styles/mtb-trailmap-global-v2/512/{z}/{x}/{y}.png';
function mk(id){const m=L.map(id,{zoomControl:true});L.tileLayer(tiles,{tileSize:512,zoomOffset:-1,maxZoom:20}).addTo(m);return m;}
const m1=mk('m1'), m2=mk('m2');
let syncing=false;
function sync(a,b){a.on('move',()=>{if(syncing)return;syncing=true;b.setView(a.getCenter(),a.getZoom(),{animate:false});syncing=false;});}
sync(m1,m2); sync(m2,m1);
let l1=null,l2=null;
const band={color:'#7fb0ff',weight:14,opacity:0.55}, route={color:'#d1242f',weight:4,opacity:1};
function ends(g,it){
  // Grey tick: from each waypoint as placed to where the app snaps it (nearest way).
  it.ends.forEach((c,k)=>{const s=it.snaps[k]; if(s) L.polyline([[c[1],c[0]],[s[1],s[0]]],{color:'#555',weight:3}).addTo(g);});
  it.ends.forEach(c=>L.circleMarker([c[1],c[0]],{radius:6,color:'#000',weight:2,fillColor:'#fff',fillOpacity:1}).addTo(g));
}
let curItem=null;
function toWps(){ const it=curItem; if(!it||!it.wps.length) return;
  m1.fitBounds(L.latLngBounds(it.wps.map(w=>[w.geometry.coordinates[1],w.geometry.coordinates[0]])).pad(0.8),{maxZoom:17});
  m2.setView(m1.getCenter(),m1.getZoom()); }
function show(i){
  document.querySelectorAll('.it').forEach((e,k)=>e.classList.toggle('sel',k===i));
  const it=ITEMS[i]; curItem=it;
  const nw=it.wps.length;
  document.getElementById('q').innerHTML=`<b>${i+1}. ${it.question}</b><br>`
    +`<span style="background:${nw?'#000':'#e6e6e6'};color:${nw?'#fff':'#333'};padding:1px 6px;border-radius:3px">`
    +`${nw?nw+' new waypoint'+(nw>1?'s':'')+' added (black dots, right map)':'No new waypoints'}</span>`
    +(nw?' <button onclick="toWps()">show new waypoints</button>':'')+`<br>${it.facts}`;
  if(l1){m1.removeLayer(l1);m2.removeLayer(l2);}
  const b=L.geoJSON(it.saved).getBounds();
  syncing=true;
  if(it.suggested.length) m1.setView([it.suggested[0].c[1],it.suggested[0].c[0]],18);
  else if(it.unroutable.length) m1.setView(it.focus,17);
  else if(it.wps.length) m1.fitBounds(L.latLngBounds(it.wps.map(w=>[w.geometry.coordinates[1],w.geometry.coordinates[0]])).pad(0.6),{maxZoom:17});
  else m1.fitBounds(b,{padding:[30,30]});
  m2.setView(m1.getCenter(),m1.getZoom());
  syncing=false;
  l1=L.layerGroup().addTo(m1); l2=L.layerGroup().addTo(m2);
  if(it.saved){L.geoJSON(it.saved,{style:band}).addTo(l1);L.geoJSON(it.saved,{style:band}).addTo(l2);}
  it.unroutable.forEach(u=>L.geoJSON(u,{style:{color:'#f08c00',weight:14,opacity:0.8}}).addTo(l2));
  if(it.before) L.geoJSON(it.before,{style:route}).addTo(l1);
  if(it.after) L.geoJSON(it.after,{style:route}).addTo(l2);
  it.wps.forEach(w=>{const c=w.geometry.coordinates;L.circleMarker([c[1],c[0]],{radius:6,color:'#000',fillColor:'#000',fillOpacity:1}).addTo(l2);});
  ends(l1,it); ends(l2,it);
  it.suggested.forEach(s=>L.circleMarker([s.c[1],s.c[0]],{radius:7,color:'#000',weight:1,fillColor:'#2da44e',fillOpacity:1})
    .bindTooltip(`suggested waypoint position (${s.offset} m from the waypoint)`,{permanent:true,direction:'right'}).addTo(l2));
}
ITEMS.forEach((it,i)=>{const d=document.createElement('div');d.className='it';
  d.innerHTML=`<b>${i+1}</b> &nbsp;route ${it.route} leg ${it.leg}<br><i>${it.wps.length?'+'+it.wps.length+' new waypoint'+(it.wps.length>1?'s':''):'no new waypoints'}`
    +`${it.unroutable.length?', orange':''}${it.suggested.length?', snap':''}</i>`;
  d.onclick=()=>show(i);document.getElementById('list').appendChild(d);});
if(ITEMS.length) show(0);
</script></body></html>"""
path = os.path.join(d, 'review.html')
open(path, 'w').write(html.replace('__DATA__', json.dumps(items)))
print(f'{len(items)} legs -> {path}')
