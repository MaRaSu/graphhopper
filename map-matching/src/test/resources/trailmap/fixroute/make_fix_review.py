#!/usr/bin/env python3
"""Local review page for /fix_route corpus results (FixRouteCorpusTest output).

Shows chosen fixed / unroutable legs: saved track (blue), the route the client draws after applying
the fix (red), added waypoints (black), non-reproducible stretches (orange). Local file only —
the fixtures are real users' routes.

Usage: PICK="route:leg:Label,…" python3 make_fix_review.py <fix_out_dir>
"""
import glob
import json
import os
import sys

out_dir = os.path.abspath(sys.argv[1])
legs_dir = os.path.join(out_dir, 'legs')
pick = [t.split(':', 2) for t in os.environ['PICK'].split(',')]
items = []
for r, l, label in pick:
    f = os.path.join(legs_dir, f'route-{r}_leg-{l}.geojson')
    if not os.path.exists(f):
        continue
    fc = json.load(open(f))
    p = fc['properties']
    un = [x for x in fc['features'] if x['properties']['name'].startswith('unroutable')]
    focus = None
    if un:
        c = un[0]['geometry']['coordinates']
        focus = [c[len(c) // 2][1], c[len(c) // 2][0]]
    items.append({'label': label, 'props': p, 'fc': fc, 'focus': focus})

html = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>Fix-route review</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>
 body{margin:0;font:13px system-ui,sans-serif;display:flex;height:100vh}
 #list{width:360px;overflow:auto;border-right:1px solid #ccc}
 #map{flex:1}
 .it{padding:6px 10px;border-bottom:1px solid #eee;cursor:pointer}
 .leg:hover,.leg.sel{background:#eef3ff}
 .legend{position:absolute;z-index:1000;right:10px;top:10px;background:#fff;padding:6px 10px;border-radius:4px;box-shadow:0 1px 4px #0003}
</style></head><body>
<div id="list"></div><div id="map"></div>
<div class="legend"><button onclick="layer&&map.fitBounds(layer.getBounds(),{padding:[30,30]})">whole leg</button><br>
<span style="color:#1f6feb">&#9644;</span> saved track &nbsp; <span style="color:#d1242f">&#9644;</span> route after fix
&nbsp; <span style="color:#f08c00">&#9644;</span> reported unroutable &nbsp; &#9679; added waypoint</div>
<script>
const ITEMS = __DATA__;
const map = L.map('map');
L.tileLayer('https://tiles.trailmap.fi/styles/mtb-trailmap-global-v2/512/{z}/{x}/{y}.png',{tileSize:512,zoomOffset:-1,maxZoom:20,attribution:'&copy; Trailmap &copy; OpenStreetMap contributors'}).addTo(map);
let layer = null;
const list = document.getElementById('list');
function style(f){
  const n=f.properties.name;
  if(n==='saved') return {color:'#1f6feb',weight:8,opacity:0.45};
  if(n==='final') return {color:'#d1242f',weight:3,opacity:0.9};
  return {color:'#f08c00',weight:6,opacity:0.8};
}
function show(i){
  document.querySelectorAll('.leg').forEach((e,k)=>e.classList.toggle('sel',k===i));
  if(layer) map.removeLayer(layer);
  const it=ITEMS[i];
  // Set the view before adding vector layers (Leaflet needs a view to add them).
  const tmp=L.geoJSON(it.fc);
  if(it.focus) map.setView(it.focus,17); else map.fitBounds(tmp.getBounds(),{padding:[30,30]});
  layer=L.geoJSON(it.fc,{style:style,pointToLayer:(f,ll)=>L.circleMarker(ll,{radius:5,color:'#000',fillColor:'#000',fillOpacity:1})}).addTo(map);
}
ITEMS.forEach((it,i)=>{
  const p=it.props, d=document.createElement('div'); d.className='it leg';
  const un=(p.unroutable||[]).map(u=>`${Math.round(u.len)} m ${u.cause} (reroute ${u.reroute==null?'-':Math.round(u.reroute)} m)`).join('; ');
  d.innerHTML=`<b>${it.label}</b><br>route ${p.route} leg ${p.leg} · ${p.status} · +${p.added} waypoints<br>`+
    `saved ${p.saved_m} m → after fix ${p.final_m} m${un?'<br>unroutable: '+un:''}`;
  d.onclick=()=>show(i); list.appendChild(d);
});
if(ITEMS.length) show(0);
</script></body></html>"""
path = os.path.join(out_dir, os.environ.get('OUT', 'review.html'))
open(path, 'w').write(html.replace('__DATA__', json.dumps(items)))
print(f'{len(items)} legs -> {path}')
