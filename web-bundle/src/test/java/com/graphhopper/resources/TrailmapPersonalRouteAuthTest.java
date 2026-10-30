package com.graphhopper.resources;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.GHRequest;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.routing.TestProfiles;
import com.graphhopper.trailmap.personal.FirebaseTokenVerifier;
import com.graphhopper.util.Helper;
import com.sun.net.httpserver.HttpServer;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.ws.rs.NotFoundException;
import jakarta.ws.rs.WebApplicationException;
import jakarta.ws.rs.core.Response;
import org.junit.jupiter.api.*;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.lang.reflect.Proxy;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.KeyPair;
import java.security.KeyPairGenerator;
import java.security.Signature;
import java.util.*;
import java.util.zip.GZIPOutputStream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * {@code /personal_route} as production runs it (cells from the activity service, no previewer), on Monaco with a
 * stand-in activity service: whose history a request routes on comes from the Firebase token only; without a token only
 * plain routing; a bad token is 401; the previewer's GET endpoints are off; the CORS preflight lets the web origins send
 * the Authorization header; no cells source = 404 everywhere.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
public class TrailmapPersonalRouteAuthTest {

    private static final String GH_LOCATION = "target/personal-route-auth-test-gh";
    private static final String PROFILE = "car";
    private static final String PROJECT = "trailmapv2";

    private GraphHopper hopper;
    private HttpServer activity;
    private KeyPair keys;
    private final ObjectMapper om = Jackson.newObjectMapper();
    private final List<String> fetched = Collections.synchronizedList(new ArrayList<>());
    private Path disk;

    @BeforeAll
    void setUp() throws Exception {
        Helper.removeDir(new File(GH_LOCATION));
        hopper = new GraphHopper().setGraphHopperLocation(GH_LOCATION).setOSMFile("../core/files/monaco.osm.gz")
                .setEncodedValuesString("car_access, car_average_speed")
                .setProfiles(TestProfiles.accessAndSpeed(PROFILE, "car"));
        hopper.importOrLoad();

        // user u1 has the set "test" (a few cells in Monaco); everyone else has no cells
        byte[] gz = gzip("x,y,rides\n544320,380860,3\n544321,380860,3\n");
        activity = HttpServer.create(new InetSocketAddress("127.0.0.1", 0), 0);
        activity.createContext("/internal/cells", ex -> {
            fetched.add(ex.getRequestURI().getPath() + " " + ex.getRequestHeaders().getFirst("Authorization"));
            String p = ex.getRequestURI().getPath();
            if (p.equals("/internal/cells/u1/test") || p.equals("/internal/cells/slow/test")) {
                ex.getResponseHeaders().add("ETag", "\"b1\"");
                ex.sendResponseHeaders(200, gz.length);
                // "slow": the headers come at once, the body stalls (a timeout on the headers alone would hang here)
                if (p.startsWith("/internal/cells/slow/")) {
                    try {
                        Thread.sleep(4_000);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
                ex.getResponseBody().write(gz);
            } else {
                ex.sendResponseHeaders(204, -1);
            }
            ex.close();
        });
        activity.setExecutor(java.util.concurrent.Executors.newCachedThreadPool());
        activity.start();
        disk = Files.createTempDirectory("personal-auth-disk");

        KeyPairGenerator g = KeyPairGenerator.getInstance("RSA");
        g.initialize(2048);
        keys = g.generateKeyPair();
        TrailmapPersonalRouteResource.setVerifierForTests(new FirebaseTokenVerifier(PROJECT,
                () -> new FirebaseTokenVerifier.Keys(Map.of("k1", keys.getPublic()), 3600_000), System::currentTimeMillis), PROJECT);
    }

    @AfterAll
    void tearDown() {
        TrailmapPersonalRouteResource.setVerifierForTests(null, null);
        if (activity != null) activity.stop(0);
        if (hopper != null) hopper.close();
        Helper.removeDir(new File(GH_LOCATION));
        Helper.removeDir(disk.toFile());
    }

    private static byte[] gzip(String s) throws Exception {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        try (GZIPOutputStream z = new GZIPOutputStream(out)) {
            z.write(s.getBytes(StandardCharsets.UTF_8));
        }
        return out.toByteArray();
    }

    private GraphHopperConfig productionLike() {
        GraphHopperConfig c = new GraphHopperConfig();
        c.putObject("personal_routing.cells_source", "http://127.0.0.1:" + activity.getAddress().getPort() + "/internal/cells");
        // the secret's env name; the test JVM has no such variable, so point it at one that exists
        c.putObject("personal_routing.cells_secret_env", "PATH");
        c.putObject("personal_routing.cells_disk_dir", disk.toString());
        return c;
    }

    private TrailmapPersonalRouteResource resource(GraphHopperConfig c) {
        return new TrailmapPersonalRouteResource(c, hopper, new ProfileResolver(hopper.getProfiles()), r -> r);
    }

    private String token(String sub) throws Exception {
        return token(Map.of("sub", sub), Map.of());
    }

    /** A signed token; claims and header entries given override the valid defaults. */
    private String token(Map<String, Object> claimsOver, Map<String, Object> headerOver) throws Exception {
        long now = System.currentTimeMillis() / 1000;
        Map<String, Object> claims = new LinkedHashMap<>(Map.of("iss", "https://securetoken.google.com/" + PROJECT, "aud", PROJECT,
                "sub", "u1", "iat", now - 5, "exp", now + 3000));
        claims.putAll(claimsOver);
        Map<String, Object> header = new LinkedHashMap<>(Map.of("alg", "RS256", "kid", "k1"));
        header.putAll(headerOver);
        Base64.Encoder b = Base64.getUrlEncoder().withoutPadding();
        String h = b.encodeToString(om.writeValueAsBytes(header));
        String p = b.encodeToString(om.writeValueAsBytes(claims));
        Signature s = Signature.getInstance("SHA256withRSA");
        s.initSign(keys.getPrivate());
        s.update((h + "." + p).getBytes(StandardCharsets.US_ASCII));
        return "Bearer " + h + "." + p + "." + b.encodeToString(s.sign());
    }

    private GHRequest request(Map<String, Object> extra) throws Exception {
        Map<String, Object> body = new LinkedHashMap<>();
        body.put("points", List.of(List.of(7.418737, 43.727687), List.of(7.436566, 43.74958)));
        body.put("profile", PROFILE);
        body.put("instructions", false);
        body.putAll(extra);
        return om.readValue(om.writeValueAsString(body), GHRequest.class);
    }

    private JsonNode personal(Response r) throws Exception {
        assertEquals(200, r.getStatus());
        return om.readTree(om.writeValueAsString(r.getEntity())).get("hints").get("personal");
    }

    private static int status(Executable e) {
        WebApplicationException ex = assertThrows(WebApplicationException.class, e::run);
        return ex.getResponse().getStatus();
    }

    interface Executable {
        void run() throws Exception;
    }

    @Test
    void theTokenDecidesWhoseHistory() throws Exception {
        TrailmapPersonalRouteResource r = resource(productionLike());
        Map<String, Object> favor = Map.of("personal_mode", "favor", "personal_cells", "test", "personal_strength", 1);

        JsonNode info = personal(r.doPost(request(favor), token("u1")));
        assertTrue(info.get("active").asBoolean(), "u1's own cells");
        assertEquals(2, info.get("cells").asInt());

        // a personal_user in the body is ignored: u2's token routes on u2's (empty) history, never u1's
        Map<String, Object> asU1 = new LinkedHashMap<>(favor);
        asU1.put("personal_user", "u1");
        info = personal(r.doPost(request(asU1), token("u2")));
        assertFalse(info.get("active").asBoolean());
        assertEquals("none", info.get("history").asText());
        assertTrue(fetched.stream().allMatch(f -> f.endsWith(" Bearer " + System.getenv("PATH"))), "the shared secret is sent");
    }

    @Test
    void withoutATokenOnlyPlainRouting() throws Exception {
        TrailmapPersonalRouteResource r = resource(productionLike());
        assertFalse(personal(r.doPost(request(Map.of()), null)).get("active").asBoolean());
        assertFalse(personal(r.doPost(request(Map.of("personal_user", "u1", "personal_cells", "test")), null)).get("active").asBoolean());
        assertEquals(401, status(() -> r.doPost(request(Map.of("personal_mode", "favor", "personal_user", "u1", "personal_cells", "test")), null)));
        assertEquals(401, status(() -> r.doPost(request(Map.of("algorithm", "bit_of_new", "bit_of_new_history", "avoid", "personal_user", "u1")), null)));
        // bit_of_new without history needs no user
        assertEquals(200, r.doPost(request(Map.of("algorithm", "bit_of_new")), null).getStatus());
    }

    @Test
    void aBadTokenIs401EvenForPlainRouting() throws Exception {
        TrailmapPersonalRouteResource r = resource(productionLike());
        String t = token("u1");
        String tampered = t.substring(0, t.length() - 4) + (t.endsWith("AAAA") ? "BBBB" : "AAAA");
        assertEquals(401, status(() -> r.doPost(request(Map.of()), tampered)));
        assertEquals(401, status(() -> r.doPost(request(Map.of()), "Basic abc")));
        assertEquals(401, status(() -> r.doPost(request(Map.of("personal_mode", "favor", "personal_cells", "test")), "Bearer not.a.token")));
    }

    /** S2: every kind of bad token is refused with 401 and never routes personal; an unknown kid is transient (503). */
    @Test
    void securityMatrixS2BadTokens() throws Exception {
        TrailmapPersonalRouteResource r = resource(productionLike());
        Map<String, Object> favor = Map.of("personal_mode", "favor", "personal_cells", "test");
        long now = System.currentTimeMillis() / 1000;
        Map<String, String> bad = new LinkedHashMap<>();
        bad.put("expired", token(Map.of("exp", now - 120), Map.of()));
        bad.put("wrong aud", token(Map.of("aud", "other-project"), Map.of()));
        bad.put("wrong iss", token(Map.of("iss", "https://securetoken.google.com/other-project"), Map.of()));
        bad.put("issued in the future", token(Map.of("iat", now + 600), Map.of()));
        bad.put("empty sub", token(Map.of("sub", ""), Map.of()));
        bad.put("alg none", token(Map.of(), Map.of("alg", "none")));
        bad.put("alg HS256", token(Map.of(), Map.of("alg", "HS256")));
        String t = token("u1");
        bad.put("bad signature", t.substring(0, t.lastIndexOf('.') + 1) + "AAAA" + t.substring(t.lastIndexOf('.') + 5));
        bad.put("malformed", "Bearer not.a.jwt");
        bad.put("not bearer", "Basic dTE6cHc=");
        for (Map.Entry<String, String> e : bad.entrySet()) {
            assertEquals(401, status(() -> r.doPost(request(favor), e.getValue())), e.getKey());
            assertEquals(401, status(() -> r.doPost(request(Map.of()), e.getValue())), e.getKey() + " (neutral)");
        }
        assertEquals(503, status(() -> r.doPost(request(favor), token(Map.of(), Map.of("kid", "unknown-kid")))), "unknown kid");
    }

    /** S2: neutral without a token is exactly /route. */
    @Test
    void securityMatrixS2NeutralWithoutTokenIsRoute() throws Exception {
        TrailmapPersonalRouteResource r = resource(productionLike());
        JsonNode got = om.readTree(om.writeValueAsString(r.doPost(request(Map.of("points_encoded", false)), null).getEntity()));
        GHRequest same = new GHRequest(43.727687, 7.418737, 43.74958, 7.436566).setProfile(PROFILE);
        same.putHint("instructions", false); // as request() asks
        com.graphhopper.GHResponse plain = hopper.route(same);
        assertFalse(plain.hasErrors());
        // the JSON keeps 3 decimals of a metre
        assertEquals(plain.getBest().getDistance(), got.get("paths").get(0).get("distance").asDouble(), 1e-3);
        assertEquals(plain.getBest().getPoints().size(), got.get("paths").get(0).get("points").get("coordinates").size());
    }

    /** S6: a cells service whose answer stalls costs at most the configured time, then routes plain (no disk copy). */
    @Test
    void securityMatrixS6SlowCellsServiceIsBounded() throws Exception {
        GraphHopperConfig c = productionLike();
        c.putObject("personal_routing.cells_timeout_ms", 1000);
        TrailmapPersonalRouteResource r = resource(c);
        long t0 = System.currentTimeMillis();
        JsonNode info = personal(r.doPost(request(Map.of("personal_mode", "favor", "personal_cells", "test")), token("slow")));
        long ms = System.currentTimeMillis() - t0;
        assertFalse(info.get("active").asBoolean());
        assertTrue(ms < 3_000, "answered in " + ms + " ms, the service stalls 4 s");
    }

    /** S6: a cells service that is down answers plain at once, never a 500. */
    @Test
    void securityMatrixS6CellsServiceDown() throws Exception {
        GraphHopperConfig c = productionLike();
        int closed;
        try (java.net.ServerSocket s = new java.net.ServerSocket(0, 1, java.net.InetAddress.getByName("127.0.0.1"))) {
            closed = s.getLocalPort();
        }
        c.putObject("personal_routing.cells_source", "http://127.0.0.1:" + closed + "/internal/cells");
        c.putObject("personal_routing.cells_disk_dir", Files.createTempDirectory("personal-auth-down").toString());
        TrailmapPersonalRouteResource r = resource(c);
        long t0 = System.currentTimeMillis();
        JsonNode info = personal(r.doPost(request(Map.of("personal_mode", "favor", "personal_cells", "test")), token("u1")));
        assertFalse(info.get("active").asBoolean());
        assertTrue(System.currentTimeMillis() - t0 < 5_000);
    }

    /** S6: an empty GH_CELLS_SECRET fails loudly: 503 to the client, an ERROR in the log. */
    @Test
    void securityMatrixS6MissingSecretIsLoud() throws Exception {
        GraphHopperConfig c = productionLike();
        c.putObject("personal_routing.cells_secret_env", "GH_CELLS_SECRET_NOT_SET_IN_TESTS");
        TrailmapPersonalRouteResource r = resource(c);
        ch.qos.logback.core.read.ListAppender<ch.qos.logback.classic.spi.ILoggingEvent> logs = captureLogs();
        try {
            assertEquals(503, status(() -> r.doPost(request(Map.of("personal_mode", "favor", "personal_cells", "test")), token("u1"))));
            assertTrue(logs.list.stream().anyMatch(e -> e.getLevel() == ch.qos.logback.classic.Level.ERROR
                    && e.getFormattedMessage().contains("GH_CELLS_SECRET_NOT_SET_IN_TESTS")));
        } finally {
            releaseLogs(logs);
        }
    }

    /** S6: neither tokens, nor the shared secret, nor cell rows reach the log. */
    @Test
    void securityMatrixS6NoSecretsInLogs() throws Exception {
        ch.qos.logback.core.read.ListAppender<ch.qos.logback.classic.spi.ILoggingEvent> logs = captureLogs();
        try {
            GraphHopperConfig c = productionLike();
            c.putObject("personal_routing.cells_disk_dir", Files.createTempDirectory("personal-auth-logs").toString());
            TrailmapPersonalRouteResource r = resource(c);
            String good = token("u1");
            r.doPost(request(Map.of("personal_mode", "favor", "personal_cells", "test")), good);
            String t = good.substring(7);
            String badSig = good.substring(0, good.lastIndexOf('.') + 1) + "AAAA" + good.substring(good.lastIndexOf('.') + 5);
            status(() -> r.doPost(request(Map.of()), badSig));
            String secret = System.getenv("PATH");
            for (ch.qos.logback.classic.spi.ILoggingEvent e : logs.list) {
                String m = e.getFormattedMessage();
                assertFalse(m.contains(t.substring(0, 20)) || m.contains(t.substring(t.lastIndexOf('.') + 1)), "token in log: " + m);
                assertFalse(m.contains(secret), "secret in log: " + m);
                assertFalse(m.contains("544320"), "cells in log: " + m);
            }
            assertFalse(logs.list.isEmpty(), "the capture saw the requests");
        } finally {
            releaseLogs(logs);
        }
    }

    private static ch.qos.logback.core.read.ListAppender<ch.qos.logback.classic.spi.ILoggingEvent> captureLogs() {
        ch.qos.logback.classic.Logger root = (ch.qos.logback.classic.Logger) org.slf4j.LoggerFactory.getLogger(org.slf4j.Logger.ROOT_LOGGER_NAME);
        ch.qos.logback.core.read.ListAppender<ch.qos.logback.classic.spi.ILoggingEvent> a = new ch.qos.logback.core.read.ListAppender<>();
        a.start();
        root.addAppender(a);
        return a;
    }

    private static void releaseLogs(ch.qos.logback.core.read.ListAppender<ch.qos.logback.classic.spi.ILoggingEvent> a) {
        ((ch.qos.logback.classic.Logger) org.slf4j.LoggerFactory.getLogger(org.slf4j.Logger.ROOT_LOGGER_NAME)).detachAppender(a);
    }

    @Test
    void googleKeysUnreachableIs503NotATokenError() throws Exception {
        String t = token("u1");
        try {
            TrailmapPersonalRouteResource.setVerifierForTests(new FirebaseTokenVerifier(PROJECT, () -> {
                throw new java.io.IOException("down");
            }, System::currentTimeMillis), PROJECT);
            TrailmapPersonalRouteResource r = resource(productionLike());
            assertEquals(503, status(() -> r.doPost(request(Map.of("personal_mode", "favor", "personal_cells", "test")), t)));
        } finally {
            TrailmapPersonalRouteResource.setVerifierForTests(new FirebaseTokenVerifier(PROJECT,
                    () -> new FirebaseTokenVerifier.Keys(Map.of("k1", keys.getPublic()), 3600_000), System::currentTimeMillis), PROJECT);
        }
    }

    @Test
    void previewerEndpointsAreOffInProduction() {
        TrailmapPersonalRouteResource r = resource(productionLike());
        assertThrows(NotFoundException.class, () -> r.dataSets("u1"));
        assertThrows(NotFoundException.class, () -> r.cells("u1", "test", "7.4,43.7,7.5,43.8"));
    }

    @Test
    void noCellsSourceIsOffEverywhere() {
        TrailmapPersonalRouteResource r = resource(new GraphHopperConfig());
        assertThrows(NotFoundException.class, () -> r.doPost(request(Map.of()), null));
        assertThrows(NotFoundException.class, () -> r.dataSets("u1"));
    }

    @Test
    void bothSourcesIsAConfigError() {
        GraphHopperConfig c = productionLike();
        c.putObject("personal_routing.cells_dir", disk.toString());
        TrailmapPersonalRouteResource r = resource(c);
        assertThrows(IllegalStateException.class, () -> r.doPost(request(Map.of("personal_mode", "favor", "personal_cells", "test")), token("u1")));
    }

    @Test
    void preflightLetsTheWebOriginsSendAuthorization() {
        TrailmapPersonalRouteResource r = resource(productionLike());
        // exactly the origins the web app is served at (lead_web, 2026-10-30)
        for (String origin : List.of("https://web.trailmap.fi", "https://beta.trailmap.fi", "https://web.trailmap.guide",
                "https://ng-test.trailmap.fi", "http://localhost:3000")) {
            Map<String, String> h = new HashMap<>();
            assertEquals(204, r.preflight(origin, response(h)).getStatus());
            assertEquals(origin, h.get("Access-Control-Allow-Origin"), origin);
            assertTrue(h.get("Access-Control-Allow-Headers").contains("Authorization"), origin);
            assertTrue(h.get("Access-Control-Allow-Headers").contains("Content-Type"), origin);
            assertTrue(h.get("Access-Control-Allow-Methods").contains("POST"), origin);
        }
        // redirect-only hosts never send an Origin; look-alikes, plain http and other ports are refused
        for (String origin : Arrays.asList("https://trailmap.fi", "https://www.trailmap.fi", "https://trailmap.guide",
                "https://global.trailmap.fi", "https://evil.example", "https://trailmap.fi.evil.example",
                "https://web.trailmap.fi.evil.example", "http://web.trailmap.fi", "https://web.trailmap.fi:8443",
                "https://evil.web.trailmap.fi", "http://localhost:3001", null)) {
            Map<String, String> h = new HashMap<>();
            r.preflight(origin, response(h));
            assertTrue(h.isEmpty(), "no CORS grant for " + origin);
        }
    }

    /** A servlet response that records the headers set on it. */
    private static HttpServletResponse response(Map<String, String> headers) {
        return (HttpServletResponse) Proxy.newProxyInstance(HttpServletResponse.class.getClassLoader(),
                new Class[]{HttpServletResponse.class}, (proxy, m, args) -> {
                    if (m.getName().equals("setHeader") || m.getName().equals("addHeader"))
                        headers.put((String) args[0], (String) args[1]);
                    return null;
                });
    }
}
