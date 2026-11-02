package com.graphhopper.trailmap.roundtrip.loop;

import com.graphhopper.GHRequest;
import com.graphhopper.GraphHopper;
import com.graphhopper.routing.TestProfiles;
import com.graphhopper.util.Helper;
import com.graphhopper.util.exceptions.ConnectionNotFoundException;
import com.graphhopper.util.exceptions.PointNotFoundException;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * End to end on Monaco: a press whose request the graph can't serve (a path detail it lacks) gets that reason from the
 * hand-over instead of the loop being dropped, which made /autoroute answer "no loop"; a loop whose own waypoints fail
 * is still dropped.
 */
class LoopHandoverRequestErrorTest {

    private static final String GH_LOCATION = "target/loop-handover-request-error-gh";
    private static final String PROFILE = "car";
    private static final GHPoint START = new GHPoint(43.727687, 7.418737);
    private static GraphHopper hopper;

    @BeforeAll
    static void setUp() {
        Helper.removeDir(new File(GH_LOCATION));
        // no surface encoded value: details ["surface"] can't be served
        hopper = new GraphHopper().
                setGraphHopperLocation(GH_LOCATION).
                setOSMFile("../core/files/monaco.osm.gz").
                setEncodedValuesString("car_access, car_average_speed").
                setProfiles(TestProfiles.accessAndSpeed(PROFILE, "car"),
                        TestProfiles.accessAndSpeed(PROFILE + "_explore", "car"));
        hopper.importOrLoad();
    }

    @AfterAll
    static void tearDown() {
        hopper.close();
        Helper.removeDir(new File(GH_LOCATION));
    }

    /** A loop through two waypoints in Monaco (or the given ones), as the engine hands it over. */
    private static LoopAutoRoute.Route route(List<double[]> waypoints) {
        MergeEngine.Route r = new MergeEngine.Route();
        r.source = "test";
        r.label = "";
        r.distance = 3000;
        r.waypoints = waypoints;
        r.coords = List.of(new double[]{START.lat, START.lon}, waypoints.get(0), new double[]{START.lat, START.lon});
        return new LoopAutoRoute.Route(r);
    }

    private static final List<double[]> MONACO = List.of(new double[]{43.74958, 7.436566}, new double[]{43.7389, 7.4248});

    private static GHRequest request(List<String> details) {
        GHRequest req = new GHRequest(List.of(START)).setProfile(PROFILE);
        req.setPathDetails(details);
        return req;
    }

    @Test
    void detailNotInTheGraphIsPassedThrough() {
        LoopHandover handover = new LoopHandover(hopper);
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class,
                () -> handover.handOver(PROFILE, START, route(MONACO), request(List.of("surface"))));
        assertTrue(e.getMessage().contains("Cannot find the path details"), e.getMessage());
        assertTrue(e.getMessage().contains("surface"), e.getMessage());
    }

    @Test
    void servableRequestIsHandedOver() {
        assertNotNull(new LoopHandover(hopper).handOver(PROFILE, START, route(MONACO), request(List.of("distance"))));
    }

    @Test
    void loopWhoseWaypointCannotBeRoutedIsStillDropped() {
        // a waypoint in the sea, far from any road of the graph: this loop fails, the press goes on
        List<double[]> sea = List.of(new double[]{43.60, 7.50}, MONACO.get(1));
        assertNull(new LoopHandover(hopper).handOver(PROFILE, START, route(sea), request(List.of("distance"))));
    }

    @Test
    void perLoopFailuresAreNotRequestErrors() {
        assertNull(LoopHandover.requestError(List.of(new PointNotFoundException("p", 1),
                new ConnectionNotFoundException("c", java.util.Map.of()))));
        IllegalArgumentException plain = new IllegalArgumentException("Cannot find the path details: [surface]");
        assertSame(plain, LoopHandover.requestError(List.of(new PointNotFoundException("p", 1), plain)));
    }
}
