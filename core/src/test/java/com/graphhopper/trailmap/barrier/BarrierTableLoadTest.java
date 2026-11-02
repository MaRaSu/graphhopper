package com.graphhopper.trailmap.barrier;

import com.graphhopper.GraphHopper;
import com.graphhopper.routing.TestProfiles;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.util.Helper;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.nio.file.Files;

import static org.junit.jupiter.api.Assertions.*;

/**
 * W26 (lead_routing GO 2026-11-02): trailmap_barriers.json is read once when the graph loads. A missing file is an
 * empty table and the graph serves; a malformed or truncated file fails the load with a clear message, instead of
 * every /route and /personal_route request failing later.
 */
public class BarrierTableLoadTest {

    private static final String DIR = "./target/barrier-table-load-gh";

    @AfterEach
    void clean() {
        Helper.removeDir(new File(DIR));
    }

    private static TrailmapGraphHopper hopper() {
        TrailmapGraphHopper h = new TrailmapGraphHopper();
        h.setGraphHopperLocation(DIR);
        h.setOSMFile("../core/files/monaco.osm.gz");
        h.setEncodedValuesString("car_access, car_average_speed");
        h.setProfiles(TestProfiles.accessAndSpeed("car"));
        return h;
    }

    private static void imported() {
        Helper.removeDir(new File(DIR));
        GraphHopper h = hopper();
        h.importOrLoad();
        h.close();
    }

    @Test
    void missingFileLoadsWithAnEmptyTable() {
        imported();
        new File(DIR, BarrierTable.FILE).delete();
        TrailmapGraphHopper h = hopper();
        assertTrue(h.load(), "the graph loads without a barrier table");
        assertEquals(0, h.barrierTable().size());
        assertSame(h.barrierTable(), BarrierTable.of(h), "every caller uses the table loaded with the graph");
        h.close();
    }

    @Test
    void validFileIsReadAtLoad() throws Exception {
        imported();
        Files.writeString(new File(DIR, BarrierTable.FILE).toPath(),
                "[{\"osm_node\":1,\"lat\":43.73,\"lon\":7.42,\"tags\":{\"barrier\":\"gate\"},\"bike\":\"ASK\",\"foot\":\"ASK\"}]");
        TrailmapGraphHopper h = hopper();
        assertTrue(h.load());
        assertEquals(1, h.barrierTable().size());
        h.close();
    }

    @Test
    void malformedFileFailsTheLoad() throws Exception {
        imported();
        // a truncated table, as a broken copy would leave it
        Files.writeString(new File(DIR, BarrierTable.FILE).toPath(), "[{\"osm_node\":1,\"lat\":43.7");
        TrailmapGraphHopper h = hopper();
        IllegalStateException e = assertThrows(IllegalStateException.class, h::load);
        assertTrue(e.getMessage().contains("malformed or truncated barrier table"), e.getMessage());
        assertTrue(e.getMessage().contains(BarrierTable.FILE), e.getMessage());
        h.close();
    }
}
