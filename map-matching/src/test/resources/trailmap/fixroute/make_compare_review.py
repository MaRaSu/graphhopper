#!/usr/bin/env python3
"""Local review page comparing the two /fix_route engines (EngineCompareExportTest output).

Per leg: saved track (blue), the client's current route before any fix (grey dashed), the current
geometry engine's result (green), the new matching engine's result (red), the matching engine's added
waypoints (black) and reported unroutable stretches (orange). Local file only (real users' routes).

Usage: python3 make_compare_review.py <compare_dir>
"""
import glob
import json
import os
import sys

d = os.path.abspath(sys.argv[1])
items = []
for f in sorted(glob.glob(os.path.join(d, '*.geojson'))):
    fc = json.load(open(f))
    p = fc['properties']
    focus = None
    for x in fc['features']:
        if x['properties']['name'].startswith('match_unroutable') or x['properties']['name'] == 'match_new_waypoint':
            c = x['geometry']['coordinates']
            c = c[len(c) // 2] if x['geometry']['type'] == 'LineString' else c
            focus = [c[1], c[0]]
            break
    items.append({'label': p['label'], 'route': p['route'], 'leg': p['leg'], 'fc': fc, 'focus': focus})
items.sort(key=lambda it: it['label'])

html = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>Engine comparison</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>
 body{margin:0;font:13px system-ui,sans-serif;display:flex;height:100vh}
 #list{width:340px;overflow:auto;border-right:1px solid #ccc}
 #map{flex:1}
 .it{padding:6px 10px;border-bottom:1px solid #eee;cursor:pointer}
 .leg:hover,.leg.sel{background:#eef3ff}
 .legend{position:absolute;z-index:1000;right:10px;top:10px;background:#fff;padding:6px 10px;border-radius:4px;box-shadow:0 1px 4px #0003;line-height:1.6}
</style></head><body>
<div id="list"></div><div id="map"></div>
<div class="legend"><button onclick="layer&&map.fitBounds(layer.getBounds(),{padding:[30,30]})">whole leg</button><br>
<span style="color:#1f6feb">&#9644;&#9644;</span> saved track<br>
<span style="color:#666">- - -</span> current route (before fix)<br>
<span style="color:#1a7f37">&#9644;</span> current engine result<br>
<span style="color:#d1242f">&#9644;</span> new engine result<br>
<span style="color:#f08c00">&#9644;&#9644;</span> new engine: unroutable &nbsp; &#9679; new engine: added waypoint</div>
<script>
const ITEMS = __DATA__;
const map = L.map('map');
L.tileLayer('https://tiles.trailmap.fi/styles/mtb-trailmap-global-v2/512/{z}/{x}/{y}.png',{tileSize:512,zoomOffset:-1,maxZoom:20,attribution:'&copy; Trailmap &copy; OpenStreetMap contributors'}).addTo(map);
let layer = null;
const list = document.getElementById('list');
function style(f){
  const n=f.properties.name;
  if(n==='saved') return {color:'#1f6feb',weight:9,opacity:0.4};
  if(n==='before') return {color:'#666',weight:2,dashArray:'6 6',opacity:0.9};
  if(n==='geo_final') return {color:'#1a7f37',weight:4,opacity:0.8};
  if(n==='match_final') return {color:'#d1242f',weight:2.5,opacity:0.95};
  if(n.startsWith('match_unroutable')) return {color:'#f08c00',weight:8,opacity:0.7};
  return {opacity:0};
}
function show(i){
  document.querySelectorAll('.leg').forEach((e,k)=>e.classList.toggle('sel',k===i));
  if(layer) map.removeLayer(layer);
  const it=ITEMS[i];
  const tmp=L.geoJSON(it.fc);
  if(it.focus) map.setView(it.focus,17); else map.fitBounds(tmp.getBounds(),{padding:[30,30]});
  layer=L.geoJSON(it.fc,{style:style,filter:f=>!f.properties.name.startsWith('geo_new')&&!f.properties.name.startsWith('geo_unroutable'),
    pointToLayer:(f,ll)=>L.circleMarker(ll,{radius:5,color:'#000',fillColor:'#000',fillOpacity:1})}).addTo(map);
}
ITEMS.forEach((it,i)=>{
  const d=document.createElement('div'); d.className='it leg';
  d.innerHTML=`<b>${it.label}</b><br>route ${it.route} leg ${it.leg}`;
  d.onclick=()=>show(i); list.appendChild(d);
});
if(ITEMS.length) show(0);
</script></body></html>"""
path = os.path.join(d, 'review.html')
open(path, 'w').write(html.replace('__DATA__', json.dumps(items)))
print(f'{len(items)} legs -> {path}')
