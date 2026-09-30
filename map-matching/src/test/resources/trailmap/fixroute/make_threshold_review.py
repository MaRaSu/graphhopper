#!/usr/bin/env python3
"""Threshold calibration page for /fix_route: which deviations are worth fixing?

Per leg (from EngineCompareExportTest output): the saved route as a wide pale-blue band and the
client's route TODAY (before any fix) as a red line, plus the leg's own waypoints. One question per
leg: fix it, or leave it? The deviation's size and length are measured here (route vs saved line,
ignoring 30 m at each end, where waypoint snapping dominates). Local file only (real users' routes).

Usage: python3 make_threshold_review.py <compare_dir> <out_html> route:leg[,route:leg...]
"""
import json
import math
import os
import sys

def xy(p, q):
    k = math.cos(math.radians(p[1])) * 111320
    return (q[0] - p[0]) * k, (q[1] - p[1]) * 111320


def seg_dist(p, a, b):
    ax, ay = xy(p, a)
    bx, by = xy(p, b)
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0 if l2 == 0 else max(0, min(1, -(ax * dx + ay * dy) / l2))
    return math.hypot(ax + t * dx, ay + t * dy)


def dist(p, line):
    return min(seg_dist(p, line[i], line[i + 1]) for i in range(len(line) - 1))


def densify(line, step=2):
    o = []
    for i in range(len(line) - 1):
        n = max(1, int(math.hypot(*xy(line[i], line[i + 1])) / step))
        for k in range(n):
            o.append([line[i][0] + (line[i + 1][0] - line[i][0]) * k / n, line[i][1] + (line[i + 1][1] - line[i][1]) * k / n])
    o.append(line[-1])
    return o


def main():
    d, out, legs = os.path.abspath(sys.argv[1]), sys.argv[2], sys.argv[3].split(',')
    items = []
    for i, rl in enumerate(legs):
        r, l = rl.split(':')
        fc = json.load(open(os.path.join(d, f'route-{r}_leg-{l}.geojson')))
        lines = {x['properties']['name']: x for x in fc['features'] if x['geometry']['type'] == 'LineString'}
        saved, before = lines['saved'], lines['before']
        pts = densify(before['geometry']['coordinates'])
        ds = [dist(p, saved['geometry']['coordinates']) for p in pts]
        inner = range(15, len(pts) - 15)
        k = max(inner, key=lambda j: ds[j])
        far_m = 2 * sum(1 for j in inner if ds[j] > 8)
        wps = [x['geometry']['coordinates'] for x in fc['features'] if x['properties']['name'] == 'fixed_waypoint']
        items.append({'label': chr(ord('A') + i), 'route': r, 'leg': l, 'saved': saved, 'before': before,
                      'wps': wps, 'focus': [pts[k][1], pts[k][0]],
                      'facts': f'Today\'s route is up to {ds[k]:.0f} m from the saved route; it is more than 8 m away over {far_m} m.'})

    html = """<!doctype html><html lang="en"><head><meta charset="utf-8">
    <title>Deviation threshold</title>
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
    <style>
     body{margin:0;font:14px system-ui,sans-serif;height:100vh;display:flex;flex-direction:column}
     header{padding:8px 12px;background:#f6f8fa;border-bottom:1px solid #ddd}
     main{flex:1;display:flex;min-height:0}
     #list{width:150px;overflow:auto;border-right:1px solid #ccc}
     .it{padding:8px 10px;border-bottom:1px solid #eee;cursor:pointer}
     .it:hover,.it.sel{background:#eef3ff}
     #map{flex:1}
     .key{display:inline-block;width:22px;vertical-align:middle;margin:0 4px 0 12px}
    </style></head><body>
    <header><div id="q" style="font-size:15px;margin-bottom:6px"></div>
     Answer each letter with <b>Fix</b> or <b>Leave</b>. Click a letter on the left; "whole leg" zooms out.
     <button onclick="whole()">whole leg</button><br>
     <span class="key" style="background:#7fb0ff;height:12px"></span>saved route
     <span class="key" style="background:#d1242f;height:4px"></span>the app's route today (no fix)
     &nbsp; &#9675; the leg's own waypoints</header>
    <main><div id="list"></div><div id="map"></div></main>
    <script>
    const ITEMS = __DATA__;
    const map = L.map('map');
    L.tileLayer('https://tiles.trailmap.fi/styles/mtb-trailmap-global-v2/512/{z}/{x}/{y}.png',{tileSize:512,zoomOffset:-1,maxZoom:20}).addTo(map);
    let layer=null, cur=0;
    function whole(){ if(layer) map.fitBounds(L.geoJSON(ITEMS[cur].saved).getBounds(),{padding:[30,30]}); }
    function show(i){
      cur=i;
      document.querySelectorAll('.it').forEach((e,k)=>e.classList.toggle('sel',k===i));
      const it=ITEMS[i];
      document.getElementById('q').innerHTML=`<b>${it.label}. Should the fixer correct this deviation, or leave the route as it is?</b><br>${it.facts}`;
      map.setView(it.focus,17);
      if(layer) map.removeLayer(layer);
      layer=L.layerGroup().addTo(map);
      L.geoJSON(it.saved,{style:{color:'#7fb0ff',weight:14,opacity:0.55}}).addTo(layer);
      L.geoJSON(it.before,{style:{color:'#d1242f',weight:4,opacity:1}}).addTo(layer);
      it.wps.forEach(c=>L.circleMarker([c[1],c[0]],{radius:6,color:'#000',weight:2,fillColor:'#fff',fillOpacity:1}).addTo(layer));
    }
    ITEMS.forEach((it,i)=>{const d=document.createElement('div');d.className='it';
      d.innerHTML=`<b>${it.label}</b> &nbsp;route ${it.route}<br>leg ${it.leg}`;d.onclick=()=>show(i);document.getElementById('list').appendChild(d);});
    if(ITEMS.length) show(0);
    </script></body></html>"""
    open(out, 'w').write(html.replace('__DATA__', json.dumps(items)))
    for it in items:
        print(it['label'], it['route'], it['leg'], it['facts'])
    print('->', out)


if __name__ == '__main__':
    main()
