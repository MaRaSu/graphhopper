package com.graphhopper.trailmap.personal;

import com.graphhopper.storage.BaseGraph;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The store drops a user's cells after the idle time even when no further request comes, and keeps
 * them while the user keeps routing.
 */
public class PersonalStoreTest {

    @TempDir
    Path cellsDir;

    private PersonalStore store(long ttlMillis) throws IOException {
        Files.createDirectories(cellsDir.resolve("u"));
        Files.writeString(cellsDir.resolve("u").resolve("set.csv"), "x,y,rides\n1,2,3\n");
        BaseGraph graph = new BaseGraph.Builder(4).create();
        graph.edge(0, 1).setDistance(10);
        return new PersonalStore(cellsDir, graph, ttlMillis, 8);
    }

    @Test
    void sweepIntervalIsAFifthOfTheTtlBetweenOneSecondAndOneMinute() {
        assertEquals(60_000, PersonalStore.sweepIntervalMillis(5 * 60_000));
        assertEquals(1_000, PersonalStore.sweepIntervalMillis(200));
        assertEquals(12_000, PersonalStore.sweepIntervalMillis(60_000));
    }

    @Test
    void idleEntryIsDroppedWithoutAnotherRequest() throws Exception {
        try (PersonalStore s = store(200)) {
            assertNotNull(s.get("u", "set"));
            assertEquals(1, s.size());
            long deadline = System.currentTimeMillis() + 5_000;
            while (s.size() > 0 && System.currentTimeMillis() < deadline)
                Thread.onSpinWait();
            assertEquals(0, s.size(), "the background sweep should have dropped the idle entry");
        }
    }

    @Test
    void usedEntryStaysAndIsReused() throws Exception {
        try (PersonalStore s = store(60_000)) {
            assertFalse(s.get("u", "set").cached);
            s.sweep();
            assertEquals(1, s.size());
            assertTrue(s.get("u", "set").cached);
        }
    }

    @Test
    void closeDropsEverything() throws Exception {
        PersonalStore s = store(60_000);
        s.get("u", "set");
        s.close();
        assertEquals(0, s.size());
    }
}
