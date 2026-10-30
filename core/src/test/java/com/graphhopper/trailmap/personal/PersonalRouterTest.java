package com.graphhopper.trailmap.personal;

import com.carrotsearch.hppc.LongIntHashMap;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.routing.TestProfiles;
import com.graphhopper.util.Helper;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.TurnCostsConfig;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * End to end on Monaco with a turn-cost profile: the personal router routes exactly as
 * GraphHopper when inactive, and moves the route the expected way when active.
 */
class PersonalRouterTest {

    private static final String GH_LOCATION = "target/personal-router-test-gh";
    private static final String PROFILE = "car_tc";
    private static GraphHopper hopper;
    private static final GHPoint FROM = new GHPoint(43.727687, 7.418737);
    private static final GHPoint TO = new GHPoint(43.74958, 7.436566);

    @BeforeAll
    static void setUp() {
        Helper.removeDir(new File(GH_LOCATION));
        hopper = new GraphHopper().
                setGraphHopperLocation(GH_LOCATION).
                setOSMFile("../core/files/monaco.osm.gz").
                setEncodedValuesString("car_access, car_average_speed").
                setProfiles(TestProfiles.accessAndSpeed(PROFILE, "car").
                        setTurnCostsConfig(new TurnCostsConfig(List.of("motorcar", "motor_vehicle"), 20)));
        hopper.importOrLoad();
    }

    @AfterAll
    static void tearDown() {
        hopper.close();
        Helper.removeDir(new File(GH_LOCATION));
    }

    private static GHRequest request() {
        return new GHRequest(FROM, TO).setProfile(PROFILE);
    }

    private static PersonalParams params(String mode, double strength, double switchPenalty) {
        PMap h = new PMap();
        h.putObject(PersonalParams.MODE, mode);
        h.putObject(PersonalParams.USER, "owner");
        h.putObject(PersonalParams.CELLS, "test");
        h.putObject(PersonalParams.STRENGTH, strength);
        h.putObject(PersonalParams.SWITCH_PENALTY, switchPenalty);
        return PersonalParams.from(h);
    }

    private static ResponsePath route(PersonalFamiliarity f, PersonalParams p) {
        GHResponse rsp = new PersonalRouter(hopper, f, p).route(request());
        assertFalse(rsp.hasErrors(), rsp.getErrors().toString());
        return rsp.getBest();
    }

    /** Cells along a route, as the activity pipeline would mark them from a recorded ride. */
    private static PersonalFamiliarity ridden(PointList pl) {
        PersonalCellGrid grid = new PersonalCellGrid(20);
        LongIntHashMap m = new LongIntHashMap();
        for (int i = 0; i < pl.size() - 1; i++)
            for (int s = 0; s <= 20; s++) {
                double t = s / 20.0;
                m.put(grid.cellId(pl.getLat(i) + (pl.getLat(i + 1) - pl.getLat(i)) * t,
                        pl.getLon(i) + (pl.getLon(i + 1) - pl.getLon(i)) * t), 20);
            }
        return new PersonalFamiliarity(new PersonalCells(grid, m), hopper.getBaseGraph());
    }

    private static double shareInCells(ResponsePath path, PersonalFamiliarity f) {
        return f.measure(path.getPoints())[0];
    }

    @Test
    void inactiveRoutesExactlyAsGraphHopper() {
        ResponsePath plain = hopper.route(request()).getBest();
        ResponsePath neutral = route(null, params("neutral", 0.5, 30));
        assertEquals(plain.getDistance(), neutral.getDistance(), 1e-6);
        assertEquals(plain.getTime(), neutral.getTime());
        assertEquals(plain.getPoints(), neutral.getPoints());
        // an active mode with an empty history changes nothing either: every factor is 1
        PersonalFamiliarity empty = new PersonalFamiliarity(
                new PersonalCells(new PersonalCellGrid(20), new LongIntHashMap()), hopper.getBaseGraph());
        assertEquals(plain.getPoints(), route(empty, params("explore", 1.0, 30)).getPoints());
    }

    @Test
    void modesMoveTheRouteTheExpectedWay() {
        ResponsePath plain = hopper.route(request()).getBest();
        PersonalFamiliarity f = ridden(plain.getPoints());
        assertTrue(shareInCells(plain, f) > 0.95);

        // favor: the ridden route is already the profile's best, so it stays
        assertEquals(plain.getPoints(), route(f, params("favor", 1.0, 0)).getPoints());

        // explore and avoid: the route moves off the ridden cells, within the cost cap
        for (String mode : new String[]{"explore", "avoid"}) {
            double strength = 1.0;
            ResponsePath p = route(f, params(mode, strength, 0));
            assertTrue(shareInCells(p, f) < shareInCells(plain, f) - 0.1, mode + ": share " + shareInCells(p, f));
            assertTrue(p.getRouteWeight() >= plain.getRouteWeight(), mode);
        }
    }

    @Test
    void switchPenaltyStillFindsARoute() {
        ResponsePath plain = hopper.route(request()).getBest();
        PersonalFamiliarity f = ridden(plain.getPoints());
        ResponsePath p = route(f, params("explore", 1.0, 120));
        assertTrue(p.getDistance() > 0);
        assertTrue(f.getComputedEdges() > 0);
    }
}
