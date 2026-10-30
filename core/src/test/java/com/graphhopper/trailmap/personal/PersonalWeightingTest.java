package com.graphhopper.trailmap.personal;

import com.carrotsearch.hppc.LongIntHashMap;
import com.graphhopper.routing.Dijkstra;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.io.StringReader;

import static org.junit.jupiter.api.Assertions.*;

class PersonalWeightingTest {

    /** Distance as weight, 1 per metre; turn costs on, no turn cost of its own. */
    static class DistanceWeighting implements Weighting {
        public double calcMinWeightPerDistance() { return 1; }
        public double calcEdgeWeight(EdgeIteratorState e, boolean reverse) { return e.getDistance(); }
        public long calcEdgeMillis(EdgeIteratorState e, boolean reverse) { return (long) (e.getDistance() * 200); }
        public double calcTurnWeight(int in, int via, int out) { return 0; }
        public long calcTurnMillis(int in, int via, int out) { return 0; }
        public boolean hasTurnCosts() { return true; }
        public String getName() { return "distance"; }
    }

    // A square ~1 km wide near Tampere. Route 0 → 1 → 3 (north side) is the ridden one,
    // 0 → 2 → 3 (south side) is unridden and 2 % longer.
    BaseGraph graph;
    EdgeIteratorState north01, north13, south02, south23;
    PersonalCells cells;
    PersonalCellGrid grid = new PersonalCellGrid(20);

    @BeforeEach
    void setUp() {
        graph = new BaseGraph.Builder(4).create();
        NodeAccess na = graph.getNodeAccess();
        na.setNode(0, 61.5000, 23.7500);
        na.setNode(1, 61.5050, 23.7600);
        na.setNode(2, 61.4950, 23.7600);
        na.setNode(3, 61.5000, 23.7700);
        north01 = graph.edge(0, 1).setDistance(1000);
        north13 = graph.edge(1, 3).setDistance(1000);
        south02 = graph.edge(0, 2).setDistance(1020);
        south23 = graph.edge(2, 3).setDistance(1020);

        LongIntHashMap m = new LongIntHashMap();
        mark(m, 61.5000, 23.7500, 61.5050, 23.7600, 12);
        mark(m, 61.5050, 23.7600, 61.5000, 23.7700, 2);
        cells = new PersonalCells(grid, m);
    }

    private void mark(LongIntHashMap m, double lat1, double lon1, double lat2, double lon2, int rides) {
        for (int i = 0; i <= 1000; i++) {
            double t = i / 1000.0;
            m.put(grid.cellId(lat1 + (lat2 - lat1) * t, lon1 + (lon2 - lon1) * t), rides);
        }
    }

    private PersonalWeighting weighting(String mode, double strength, double switchPenalty) {
        PMap h = new PMap();
        h.putObject(PersonalParams.MODE, mode);
        h.putObject(PersonalParams.USER, "owner");
        h.putObject(PersonalParams.CELLS, "test");
        h.putObject(PersonalParams.STRENGTH, strength);
        h.putObject(PersonalParams.SWITCH_PENALTY, switchPenalty);
        return new PersonalWeighting(new DistanceWeighting(), new PersonalFamiliarity(cells, graph), PersonalParams.from(h));
    }

    @Test
    void shareOfRiddenAndUnriddenEdges() {
        PersonalFamiliarity f = new PersonalFamiliarity(cells, graph);
        assertEquals(1.0, f.share(north01), 0.01);
        assertEquals(1.0, f.share(north13), 0.01);
        assertEquals(0.0, f.share(south02), 0.03); // only the shared start cell
        assertEquals(12, f.rides(north01));
        assertEquals(2, f.rides(north13));
        assertEquals(3, f.getComputedEdges());
    }

    @Test
    void exploreMakesRiddenEdgesDearer() {
        PersonalWeighting w = weighting("explore", 0.5, 0);
        assertEquals(1500, w.calcEdgeWeight(north01, false), 1);
        assertEquals(1020, w.calcEdgeWeight(south02, false), 20);
    }

    @Test
    void favorMakesUnriddenAndRarelyRiddenEdgesDearer() {
        PersonalWeighting w = weighting("favor", 0.5, 0);
        assertEquals(1000, w.calcEdgeWeight(north01, false), 1);              // 12 rides ≥ saturation 10
        assertEquals(1000 * (1 + 0.5 * 0.8), w.calcEdgeWeight(north13, false), 1); // 2 rides: habit 0.2
        assertEquals(1020 * 1.5, w.calcEdgeWeight(south02, false), 20);
    }

    @Test
    void avoidMakesHabitualEdgesDearer() {
        PersonalWeighting w = weighting("avoid", 0.5, 0);
        assertEquals(1500, w.calcEdgeWeight(north01, false), 1);
        assertEquals(1000 * 1.1, w.calcEdgeWeight(north13, false), 1);
        assertEquals(1020, w.calcEdgeWeight(south02, false), 20);
    }

    @Test
    void factorsNeverGoBelowProfileCost() {
        for (String mode : new String[]{"favor", "avoid", "explore"}) {
            PersonalWeighting w = weighting(mode, 3.0, 0);
            for (EdgeIteratorState e : new EdgeIteratorState[]{north01, north13, south02, south23})
                assertTrue(w.calcEdgeWeight(e, false) >= e.getDistance() - 1e-9, mode);
            assertEquals(1, w.calcMinWeightPerDistance());
        }
    }

    @Test
    void switchPenaltyOnlyWhereRiddenStateChanges() {
        PersonalWeighting w = weighting("explore", 0, 40);
        assertEquals(0, w.calcTurnWeight(north01.getEdge(), 1, north13.getEdge()));   // ridden → ridden
        assertEquals(40, w.calcTurnWeight(north01.getEdge(), 0, south02.getEdge()));  // ridden → unridden
        assertEquals(40, w.calcTurnWeight(south23.getEdge(), 3, north13.getEdge()));  // unridden → ridden
        assertEquals(0, w.calcTurnWeight(south02.getEdge(), 2, south23.getEdge()));   // unridden → unridden
    }

    @Test
    void routeChoiceFollowsMode() {
        assertEquals("north", side(weighting("favor", 0.5, 0)));
        assertEquals("south", side(weighting("explore", 0.5, 0)));
        assertEquals("south", side(weighting("avoid", 0.5, 0)));
        assertEquals("north", side(new DistanceWeighting()));
    }

    private String side(Weighting w) {
        Path p = new Dijkstra(graph, w, TraversalMode.EDGE_BASED).calcPath(0, 3);
        assertTrue(p.isFound());
        return p.calcNodes().contains(1) ? "north" : "south";
    }

    @Test
    void neutralIsInactive() {
        PMap h = new PMap();
        assertFalse(PersonalParams.from(h).isActive());
        h.putObject(PersonalParams.MODE, "explore");
        h.putObject(PersonalParams.USER, "owner");
        h.putObject(PersonalParams.CELLS, "x");
        h.putObject(PersonalParams.STRENGTH, 0);
        h.putObject(PersonalParams.SWITCH_PENALTY, 0);
        assertFalse(PersonalParams.from(h).isActive());
    }

    @Test
    void activeModeWithoutUserOrCellsIsInactive() {
        PMap h = new PMap();
        h.putObject(PersonalParams.MODE, "explore");
        assertFalse(PersonalParams.from(h).isActive());
        h.putObject(PersonalParams.USER, "owner");
        assertFalse(PersonalParams.from(h).isActive());
        h.putObject(PersonalParams.CELLS, "gravel_last50");
        assertTrue(PersonalParams.from(h).isActive());
    }

    @Test
    void namesAreValidated() {
        PMap h = new PMap();
        h.putObject(PersonalParams.MODE, "favor");
        h.putObject(PersonalParams.USER, "../etc");
        h.putObject(PersonalParams.CELLS, "x");
        assertThrows(IllegalArgumentException.class, () -> PersonalParams.from(h));
    }

    @Test
    void readsPipelineFile() throws IOException {
        String csv = "# zoom=20 buffer_m=15 filter=gravel last=50\n"
                + "x,y,rides,last_ride_date\n"
                + "596000,293000,3,2026-10-20\n"
                + "596001,293000,1,2026-09-01\n";
        PersonalCells c = PersonalCells.read(new StringReader(csv));
        assertEquals(2, c.size());
        assertEquals(20, c.getGrid().getZoom());
        assertEquals(3, c.getMaxRides());
        double lat = c.getGrid().latOf(293000) - 1e-6, lon = c.getGrid().lonOf(596000) + 1e-6;
        assertEquals(3, c.ridesAt(lat, lon));
    }

    @Test
    void gridRoundTrip() {
        double lat = 61.4978, lon = 23.7610;
        long x = grid.cellX(lon), y = grid.cellY(lat);
        assertTrue(grid.lonOf(x) <= lon && lon < grid.lonOf(x + 1));
        assertTrue(grid.latOf(y + 1) < lat && lat <= grid.latOf(y));
        long id = grid.cellId(lat, lon);
        assertEquals(x, grid.xOf(id));
        assertEquals(y, grid.yOf(id));
    }
}
