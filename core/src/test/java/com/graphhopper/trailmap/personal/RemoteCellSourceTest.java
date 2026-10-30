package com.graphhopper.trailmap.personal;

import com.sun.net.httpserver.HttpServer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.GZIPOutputStream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The activity-service fetch with its disk copy, against a local stand-in server: 200 / 304 / 204 / 404, the secret,
 * the service being down, broken disk copies and the age-based cleanup.
 */
public class RemoteCellSourceTest {

    @TempDir
    Path disk;

    private HttpServer server;
    private String base;
    private final List<String> seen = new ArrayList<>();
    /** What the stand-in answers: status, ETag, CSV body; null status = close without answering. */
    private volatile int status = 200;
    private volatile String etag = "\"b1\"";
    private volatile String csv = "x,y,rides\n1,2,3\n4,5,6\n";

    @BeforeEach
    void start() throws IOException {
        server = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        server.createContext("/internal/cells", ex -> {
            String inm = ex.getRequestHeaders().getFirst("If-None-Match");
            seen.add(ex.getRequestURI().getPath() + " auth=" + ex.getRequestHeaders().getFirst("Authorization") + " inm=" + inm);
            if (status == 200 && etag.equals(inm)) {
                ex.sendResponseHeaders(304, -1);
            } else if (status == 200) {
                byte[] body = gzip(csv);
                ex.getResponseHeaders().add("ETag", etag);
                ex.sendResponseHeaders(200, body.length);
                ex.getResponseBody().write(body);
            } else {
                ex.sendResponseHeaders(status, -1);
            }
            ex.close();
        });
        server.setExecutor(java.util.concurrent.Executors.newCachedThreadPool()); // a stalled answer must not block the next
        server.start();
        base = "http://127.0.0.1:" + server.getAddress().getPort() + "/internal/cells";
    }

    @AfterEach
    void stop() {
        server.stop(0);
    }

    private RemoteCellSource source(String url) {
        return new RemoteCellSource(url, "s3cret", disk, 30L * 86_400_000L, Duration.ofSeconds(2), Duration.ofSeconds(5));
    }

    private static byte[] gzip(String s) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        try (GZIPOutputStream z = new GZIPOutputStream(out)) {
            z.write(s.getBytes(StandardCharsets.UTF_8));
        }
        return out.toByteArray();
    }

    @Test
    void fetchesThenRevalidatesWithTheEtag() throws IOException {
        RemoteCellSource s = source(base);
        PersonalCellSource.Loaded l = s.load("uid1", "cycling_lifetime");
        assertEquals(2, l.cells.size());
        assertEquals("\"b1\"", l.version);
        assertEquals("/internal/cells/uid1/cycling_lifetime auth=Bearer s3cret inm=null", seen.get(0));

        // the next load (after the memory entry was dropped) asks with the ETag and gets a 304
        l = s.load("uid1", "cycling_lifetime");
        assertEquals(2, l.cells.size());
        assertEquals("/internal/cells/uid1/cycling_lifetime auth=Bearer s3cret inm=\"b1\"", seen.get(1));

        // a new build replaces the disk copy
        etag = "\"b2\"";
        csv = "x,y,rides\n1,2,3\n";
        l = s.load("uid1", "cycling_lifetime");
        assertEquals(1, l.cells.size());
        assertEquals("\"b2\"", l.version);
        assertEquals(1, source(base).load("uid1", "cycling_lifetime").cells.size());
    }

    @Test
    void noCellsOrUnknownSetRoutesPlainAndDropsTheDiskCopy() throws IOException {
        RemoteCellSource s = source(base);
        assertNotNull(s.load("uid1", "cycling_lifetime"));
        status = 204;
        assertNull(s.load("uid1", "cycling_lifetime"));
        assertTrue(s.diskFiles().isEmpty());
        status = 404;
        assertNull(s.load("uid1", "nope"));
    }

    @Test
    void serviceDownUsesTheDiskCopyElseNone() throws IOException {
        assertNotNull(source(base).load("uid1", "cycling_lifetime"));
        status = 503;
        PersonalCellSource.Loaded l = source(base).load("uid1", "cycling_lifetime");
        assertEquals(2, l.cells.size());
        server.stop(0);
        l = source(base).load("uid1", "cycling_lifetime");
        assertEquals(2, l.cells.size(), "connection refused: the disk copy");
        assertNull(source(base).load("uid2", "cycling_lifetime"), "no disk copy: plain routing");
    }

    @Test
    void serverErrorUsesTheDiskCopyElseNone() throws IOException {
        assertNotNull(source(base).load("uid1", "cycling_lifetime"));
        status = 500;
        assertEquals(2, source(base).load("uid1", "cycling_lifetime").cells.size(), "500: the disk copy");
        assertNull(source(base).load("uid2", "cycling_lifetime"), "500 and no disk copy: plain routing");
        status = 401; // wrong secret on our side
        assertNull(source(base).load("uid2", "cycling_lifetime"));
    }

    @Test
    void garbageBodyIsNotUsedAndNotKept() throws IOException {
        assertNotNull(source(base).load("uid1", "cycling_lifetime"));
        server.removeContext("/internal/cells");
        server.createContext("/internal/cells", ex -> {
            byte[] b = {0x1f, (byte) 0x8b, 1, 2, 3}; // gzip magic, then nonsense
            ex.getResponseHeaders().add("ETag", "\"bad\"");
            ex.sendResponseHeaders(200, b.length);
            ex.getResponseBody().write(b);
            ex.close();
        });
        PersonalCellSource.Loaded l = source(base).load("uid1", "cycling_lifetime");
        assertEquals("\"b1\"", l.version, "the old disk copy, not the broken answer");
        assertNull(source(base).load("uid2", "cycling_lifetime"));
    }

    /** A service that sends its headers and then stalls (or never answers) is cut at the timeout, headers and body together. */
    @Test
    void slowOrStalledAnswerIsCutAtTheTimeout() throws IOException {
        server.removeContext("/internal/cells");
        server.createContext("/internal/cells", ex -> {
            seen.add(ex.getRequestURI().getPath() + " auth=" + ex.getRequestHeaders().getFirst("Authorization"));
            try {
                if (ex.getRequestURI().getPath().endsWith("/stall-body")) {
                    ex.sendResponseHeaders(200, 100);
                    Thread.sleep(15_000);
                } else {
                    Thread.sleep(15_000); // no headers at all
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            ex.close();
        });
        RemoteCellSource s = new RemoteCellSource(base, "s3cret", disk, 30L * 86_400_000L, Duration.ofMillis(500), Duration.ofMillis(1000));
        for (String set : new String[]{"stall-body", "stall-headers"}) {
            long t0 = System.currentTimeMillis();
            assertNull(s.load("uid1", set), set + ": plain routing");
            long ms = System.currentTimeMillis() - t0;
            assertTrue(ms < 2_500, set + " took " + ms + " ms with a 1 s timeout");
        }
        assertTrue(seen.stream().allMatch(x -> x.endsWith("auth=Bearer s3cret")), "the secret header arrives");
    }

    @Test
    void brokenOrHalfWrittenDiskCopyIsIgnored() throws IOException {
        RemoteCellSource s = source(base);
        s.load("uid1", "cycling_lifetime");
        Path gz = disk.resolve("uid1").resolve("cycling_lifetime.csv.gz");
        Files.write(gz, new byte[]{1, 2, 3}); // size no longer matches the meta
        seen.clear();
        assertEquals(2, s.load("uid1", "cycling_lifetime").cells.size());
        assertEquals("/internal/cells/uid1/cycling_lifetime auth=Bearer s3cret inm=null", seen.get(0), "no ETag sent for a broken copy");
        assertEquals(gzip(csv).length, Files.size(gz), "repaired");
    }

    @Test
    void badUserIdIsRefusedBeforeAnyRequest() {
        assertThrows(IllegalArgumentException.class, () -> source(base).load("../etc", "x"));
        assertThrows(IllegalArgumentException.class, () -> source(base).load("a/b", "x"));
        // a set name must not leave the user's disk folder (another user's copy, or any file)
        assertThrows(IllegalArgumentException.class, () -> source(base).load("uid1", "../victim/cycling_lifetime"));
        assertThrows(IllegalArgumentException.class, () -> source(base).load("uid1", "/etc/passwd"));
        assertThrows(IllegalArgumentException.class, () -> source(base).load("uid1", ""));
        assertTrue(seen.isEmpty());
    }

    @Test
    void cleanupDeletesCopiesUnusedFor30Days() throws IOException {
        RemoteCellSource s = source(base);
        s.load("uid1", "a");
        s.load("uid2", "b");
        FileTime old = FileTime.fromMillis(System.currentTimeMillis() - 31L * 86_400_000L);
        for (String f : new String[]{"a.csv.gz", "a.meta"})
            Files.setLastModifiedTime(disk.resolve("uid1").resolve(f), old);
        s.maintain();
        List<Path> left = s.diskFiles();
        assertEquals(2, left.size());
        assertTrue(left.stream().allMatch(p -> p.startsWith(disk.resolve("uid2"))));
    }

    @Test
    void indexListsTheSets() throws IOException {
        server.removeContext("/internal/cells");
        server.createContext("/internal/cells", ex -> {
            // the shape of activity-api.md "GraphHopper's routing sets" (0.2.7)
            byte[] b = ("{\"format\":1,\"asOf\":\"2026-10-31\",\"build\":\"x\",\"sets\":{"
                    + "\"foot_y2025\":{\"file\":\"foot_y2025.csv.gz\",\"buildId\":\"3f9c0a1b2c3d4e5f\",\"cells\":10},"
                    + "\"cycling_lifetime\":{\"file\":\"cycling_lifetime.csv.gz\",\"buildId\":\"0f9c0a1b2c3d4e5f\",\"cells\":20}}}")
                    .getBytes(StandardCharsets.UTF_8);
            ex.sendResponseHeaders(200, b.length);
            ex.getResponseBody().write(b);
            ex.close();
        });
        assertEquals(List.of("cycling_lifetime", "foot_y2025"), source(base).dataSets("uid1"));
    }
}
