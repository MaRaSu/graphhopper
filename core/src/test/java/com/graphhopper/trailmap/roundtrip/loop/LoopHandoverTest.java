package com.graphhopper.trailmap.roundtrip.loop;

import com.graphhopper.util.PointList;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** The hand-over's geometry: where a loop leaves the final route, and where the added waypoint goes. */
class LoopHandoverTest {

    // about 1 km east per 0.0186 deg of longitude at 61.5 N, 1 km north per 0.009 deg of latitude
    static final double LAT = 61.5, LON = 23.7, DLON_KM = 1 / (111.32 * Math.cos(Math.toRadians(LAT))), DLAT_KM = 1 / 111.32;

    /** A 4 km x 4 km square loop from the start, east, north, west, south, as [lat, lon] every 100 m. */
    static List<double[]> square() {
        List<double[]> out = new ArrayList<>();
        double[][] corners = {{0, 0}, {4, 0}, {4, 4}, {0, 4}, {0, 0}};
        for (int c = 1; c < corners.length; c++)
            for (int k = c == 1 ? 0 : 1; k <= 40; k++) {
                double t = k / 40.0;
                double x = corners[c - 1][0] + t * (corners[c][0] - corners[c - 1][0]);
                double y = corners[c - 1][1] + t * (corners[c][1] - corners[c - 1][1]);
                out.add(new double[]{LAT + y * DLAT_KM, LON + x * DLON_KM});
            }
        return out;
    }

    static PointList line(List<double[]> coords) {
        PointList pl = new PointList();
        for (double[] c : coords) pl.add(c[0], c[1]);
        return pl;
    }

    @Test
    void finalOnTheLoopIsFaithful() {
        LoopHandover.Divergence d = LoopHandover.divergence(square(), line(square()));
        assertNull(d.longest);
        assertEquals(0, d.offM);
    }

    @Test
    void shortcutAcrossTheNorthSideIsFound() {
        // the final route leaves the loop's north side (x 4 -> 0 at y 4) for a parallel road 500 m south of it
        List<double[]> fin = new ArrayList<>();
        for (double[] p : square()) {
            double y = (p[0] - LAT) / DLAT_KM, x = (p[1] - LON) / DLON_KM;
            if (y > 3.99 && x > 0.01 && x < 3.99) fin.add(new double[]{LAT + 3.5 * DLAT_KM, p[1]});
            else fin.add(p);
        }
        LoopHandover.Divergence d = LoopHandover.divergence(square(), line(fin));
        assertNotNull(d.longest);
        // the north side runs from 8 km to 12 km along the loop
        assertEquals(8000, d.longest.fromM, 150);
        assertEquals(12000, d.longest.toM, 150);
        assertEquals(4000, d.offM, 300);
    }

    @Test
    void shortDivergenceIsLeftAlone() {
        List<double[]> fin = new ArrayList<>();
        for (double[] p : square()) {
            double y = (p[0] - LAT) / DLAT_KM, x = (p[1] - LON) / DLON_KM;
            // 50 m off for one point = under MIN_DIVERGENCE_M
            if (y > 3.99 && Math.abs(x - 2) < 0.01) fin.add(new double[]{LAT + 3.95 * DLAT_KM, p[1]});
            else fin.add(p);
        }
        assertNull(LoopHandover.divergence(square(), line(fin)).longest);
    }

    @Test
    void addedWaypointGoesBetweenItsNeighboursAlongTheLoop() {
        List<double[]> loop = square();
        // waypoints at the three far corners (4 km, 8 km, 12 km along the loop)
        List<double[]> wps = new ArrayList<>(List.of(loop.get(40), loop.get(80), loop.get(120)));
        LoopHandover.insert(wps, loop, new LoopHandover.Stretch(8000, 12000));
        assertEquals(4, wps.size());
        // inserted third: the middle of the north side, x 2 km, y 4 km
        double[] p = wps.get(2);
        assertEquals(4, (p[0] - LAT) / DLAT_KM, 0.01);
        assertEquals(2, (p[1] - LON) / DLON_KM, 0.01);
        assertSame(loop.get(120), wps.get(3));
    }

    @Test
    void waypointPositionsFollowTheLoopOrderOnARepeatedStem() {
        // out and back on the same road: 0 -> 2 km east -> back to 0; a waypoint at 1 km is matched once going out,
        // and a second one at 1 km is matched on the way back
        List<double[]> loop = new ArrayList<>();
        for (int k = 0; k <= 20; k++) loop.add(new double[]{LAT, LON + k * 0.1 * DLON_KM});
        for (int k = 19; k >= 0; k--) loop.add(new double[]{LAT, LON + k * 0.1 * DLON_KM});
        LoopHandover.Proj proj = new LoopHandover.Proj(LAT, LON);
        double[] pos = LoopHandover.positions(proj, loop, List.of(loop.get(10), loop.get(30)));
        assertEquals(1000, pos[0], 1);
        assertEquals(3000, pos[1], 1);
    }
}
