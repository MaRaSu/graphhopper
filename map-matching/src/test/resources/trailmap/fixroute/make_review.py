#!/usr/bin/env python3
"""Build a local review page (review.html) for the /fix_route quick-check disagreements.

Reads the GeoJSON files QuickCheckCorpusTest writes to data/fix-route-out/quickcheck/ and
embeds them in one self-contained HTML page: a list of legs on the left, the map on the right
(saved track blue, new route red). Local file only — the fixtures are real users' routes, so the
page is never published.

Usage: python3 make_review.py [out_dir]
"""
import glob
import json
import os
import sys

out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(__file__), '../../../../../../../data/fix-route-out/quickcheck')
out_dir = os.path.abspath(out_dir)
import csv
diff = {}
for row in csv.DictReader(open(os.path.join(out_dir, 'legs.csv'))):
    diff[(int(row['route']), int(row['leg']))] = row.get('b_diff', '')
items = []
skipped_ends = 0
for f in sorted(glob.glob(os.path.join(out_dir, '*.geojson'))):
    fc = json.load(open(f))
    p = fc['features'][0]['properties']
    cat = os.path.basename(f).split('_')[0]
    d = diff.get((p['route'], p['leg']), '')
    # A accepts, B rejects only because one extra edge sits at a leg end: B's own boundary
    # artefact (where the slice/route begins), not a path difference — not worth reviewing.
    if cat == 'A-only' and d.startswith('ends'):
        skipped_ends += 1
        continue
    p['b_diff'] = d
    p['maxdev'] = max(p['route_to_ref_max_m'], p['ref_to_route_max_m'])
    items.append({'file': os.path.basename(f), 'cat': cat, 'props': p, 'fc': fc})
import math

def _proj(p, a, b):
    m = 111320.0; ml = m * math.cos(math.radians((a[1] + b[1]) / 2))
    ax, ay = a[0] * ml, a[1] * m; bx, by = b[0] * ml, b[1] * m; px, py = p[0] * ml, p[1] * m
    dx, dy = bx - ax, by - ay; L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)

def _worst(line, other):
    best, pt = -1, line[0]
    for q in line:
        d = min(_proj(q, other[j], other[j + 1]) for j in range(len(other) - 1)) if len(other) > 1 else 0
        if d > best: best, pt = d, q
    return best, pt

for it in items:
    s_line = it['fc']['features'][0]['geometry']['coordinates']
    r_line = it['fc']['features'][1]['geometry']['coordinates']
    d1, p1 = _worst(r_line, s_line)
    d2, p2 = _worst(s_line, r_line)
    it['focus'] = [p1[1], p1[0]] if d1 >= d2 else [p2[1], p2[0]]

pick = os.environ.get('PICK')  # "route:leg:group,…" → shortlist page
if pick:
    want = []
    for tok in pick.split(','):
        r, l, g = tok.split(':')
        want.append((int(r), int(l), g))
    by = {(it['props']['route'], it['props']['leg']): it for it in items}
    items = [dict(by[(r, l)], group=g) for (r, l, g) in want if (r, l) in by]
order = {'A-only': 0, 'bump': 1, 'B-only': 2}
if not pick:
    items.sort(key=lambda it: (order.get(it['cat'], 9), -it['props']['maxdev']))

html = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>Quick-check review</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>
 body{margin:0;font:13px system-ui,sans-serif;display:flex;height:100vh}
 #list{width:340px;overflow:auto;border-right:1px solid #ccc}
 #map{flex:1}
 .it{padding:6px 10px;border-bottom:1px solid #eee;cursor:pointer}
 .it:hover,.it.sel{background:#eef3ff}
 .cat{font-weight:600} .A-only{color:#b35900} .B-only{color:#0a6} .bump{color:#7a3db8}
 .legend{position:absolute;z-index:1000;right:10px;top:10px;background:#fff;padding:6px 10px;border-radius:4px;box-shadow:0 1px 4px #0003}
</style></head><body>
<div id="list"><div class="it"><b>A-only</b>: geometry check says same, road-edge check says different.<br><b>bump</b>: only the tolerant variant accepts (short local bump).<br><b>B-only</b>: road-edge check says same, geometry check says different.</div></div><div id="map"></div>
<div class="legend"><button onclick="layer&&map.fitBounds(layer.getBounds(),{padding:[30,30]})">whole leg</button><br><span style="color:#1f6feb">&#9644;</span> saved track &nbsp; <span style="color:#d1242f">&#9644;</span> new route (client request)</div>
<script>
const ITEMS = __DATA__;
const map = L.map('map');
L.tileLayer('https://tiles.trailmap.fi/styles/mtb-trailmap-global-v2/512/{z}/{x}/{y}.png',{tileSize:512,zoomOffset:-1,maxZoom:20,attribution:'&copy; Trailmap &copy; OpenStreetMap contributors'}).addTo(map);
let layer = null;
const list = document.getElementById('list');
function show(i){
  document.querySelectorAll('.leg').forEach((e,k)=>e.classList.toggle('sel',k===i));
  if(layer) map.removeLayer(layer);
  if(window._mk) map.removeLayer(window._mk);
  // Set the view BEFORE adding vector layers: on the very first call the map has no view yet,
  // and Leaflet then adds the layers inside setView before its SVG renderer has bounds.
  map.setView(ITEMS[i].focus, 17);
  layer = L.geoJSON(ITEMS[i].fc,{style:f=>({color:f.properties.stroke,weight:f.properties.name==='saved'?7:3,opacity:f.properties.name==='saved'?0.5:0.9})}).addTo(map);
  window._mk = L.circleMarker(ITEMS[i].focus,{radius:14,color:'#000',weight:2,fill:false}).addTo(map);
}
ITEMS.forEach((it,i)=>{
  const p=it.props, d=document.createElement('div'); d.className='it leg';
  d.innerHTML=(it.group?`<b>${it.group}</b> · `:'')+`<span class="cat ${it.cat}">${it.cat}</span> route ${p.route} leg ${p.leg} (${p.profile})<br>`+
   `new→saved max ${p.route_to_ref_max_m} m, saved→new max ${p.ref_to_route_max_m} m, `+
   `length ${Math.round(p.route_m)} vs ${Math.round(p.slice_m)} m${p.b_gap?' · matcher gap':''}`;
  d.onclick=()=>show(i); list.appendChild(d);
});
if(ITEMS.length) show(0); else list.insertAdjacentHTML('beforeend','<div class="it">No disagreements.</div>');
</script></body></html>"""
path = os.path.join(out_dir, os.environ.get('OUT', 'review.html'))
open(path, 'w').write(html.replace('__DATA__', json.dumps(items)))
print(f'{len(items)} legs -> {path} (skipped {skipped_ends} A-only end-edge artefacts)')
