package com.graphhopper.trailmap.personal;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.trailmap.personal.FirebaseTokenVerifier.InvalidTokenException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.math.BigInteger;
import java.nio.charset.StandardCharsets;
import java.security.*;
import java.security.interfaces.RSAPublicKey;
import java.util.ArrayList;
import java.util.Base64;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

public class FirebaseTokenVerifierTest {

    private static final String PROJECT = "trailmapv2";
    private static final ObjectMapper JSON = new ObjectMapper();

    private KeyPair good, other;
    private long now = 1_800_000_000_000L;
    private int fetches;
    private Map<String, PublicKey> served;
    private FirebaseTokenVerifier verifier;

    @BeforeEach
    void setUp() throws Exception {
        KeyPairGenerator g = KeyPairGenerator.getInstance("RSA");
        g.initialize(2048);
        good = g.generateKeyPair();
        other = g.generateKeyPair();
        served = new LinkedHashMap<>(Map.of("k1", good.getPublic()));
        verifier = new FirebaseTokenVerifier(PROJECT, () -> {
            fetches++;
            return new FirebaseTokenVerifier.Keys(Map.copyOf(served), 3600_000);
        }, () -> now);
    }

    private Map<String, Object> claims() {
        long s = now / 1000;
        Map<String, Object> c = new LinkedHashMap<>();
        c.put("iss", "https://securetoken.google.com/" + PROJECT);
        c.put("aud", PROJECT);
        c.put("sub", "uid123");
        c.put("iat", s - 10);
        c.put("auth_time", s - 100);
        c.put("exp", s + 3000);
        return c;
    }

    private String token(String kid, PrivateKey key, Map<String, Object> claims) throws Exception {
        return token(Map.of("alg", "RS256", "kid", kid, "typ", "JWT"), key, claims);
    }

    private static String token(Map<String, Object> header, PrivateKey key, Map<String, Object> claims) throws Exception {
        Base64.Encoder b = Base64.getUrlEncoder().withoutPadding();
        String h = b.encodeToString(JSON.writeValueAsBytes(header));
        String p = b.encodeToString(JSON.writeValueAsBytes(claims));
        Signature s = Signature.getInstance("SHA256withRSA");
        s.initSign(key);
        s.update((h + "." + p).getBytes(StandardCharsets.US_ASCII));
        return h + "." + p + "." + b.encodeToString(s.sign());
    }

    private void assertInvalid(String token, String why) {
        InvalidTokenException e = assertThrows(InvalidTokenException.class, () -> verifier.verify(token));
        assertEquals(why, e.getMessage());
    }

    @Test
    void validTokenGivesSub() throws Exception {
        assertEquals("uid123", verifier.verify(token("k1", good.getPrivate(), claims())));
    }

    @Test
    void rejectsWrongSignatureKeyAndClaims() throws Exception {
        assertInvalid(token("k1", other.getPrivate(), claims()), "bad signature");
        // an unknown kid is never accepted; right after a fetch it counts as a possible rotation (transient)
        assertInvalid(token("nope", good.getPrivate(), claims()), "token keys unavailable");

        Map<String, Object> c = claims();
        c.put("aud", "someone-else");
        assertInvalid(token("k1", good.getPrivate(), c), "wrong aud");
        c = claims();
        c.put("iss", "https://securetoken.google.com/someone-else");
        assertInvalid(token("k1", good.getPrivate(), c), "wrong iss");
        c = claims();
        c.put("exp", now / 1000 - 61);
        assertInvalid(token("k1", good.getPrivate(), c), "expired");
        c = claims();
        c.put("iat", now / 1000 + 61);
        assertInvalid(token("k1", good.getPrivate(), c), "issued in the future");
        c = claims();
        c.put("sub", "");
        assertInvalid(token("k1", good.getPrivate(), c), "bad sub");
    }

    @Test
    void rejectsOtherAlgorithmsAndGarbage() throws Exception {
        assertInvalid(token(Map.of("alg", "HS256", "kid", "k1"), good.getPrivate(), claims()), "alg must be RS256");
        assertInvalid(token(Map.of("alg", "none", "kid", "k1"), good.getPrivate(), claims()), "alg must be RS256");
        assertInvalid("", "no token");
        assertInvalid("a.b", "not a JWT");
        assertInvalid("!!.!!.!!", "bad header");
        String t = token("k1", good.getPrivate(), claims());
        String[] p = t.split("\\.");
        // payload swapped after signing
        Map<String, Object> c = claims();
        c.put("sub", "attacker");
        String forged = p[0] + "." + Base64.getUrlEncoder().withoutPadding().encodeToString(JSON.writeValueAsBytes(c)) + "." + p[2];
        assertInvalid(forged, "bad signature");
    }

    @Test
    void keysAreCachedAndAnUnknownKidRefetchesAtMostOnceAMinute() throws Exception {
        verifier.verify(token("k1", good.getPrivate(), claims()));
        verifier.verify(token("k1", good.getPrivate(), claims()));
        assertEquals(1, fetches);

        // Google rotates in k2; within the 1-minute gap the token is "unavailable" (503, transient), not refetched
        served.put("k2", other.getPublic());
        assertInvalid(token("k2", other.getPrivate(), claims()), "token keys unavailable");
        assertEquals(1, fetches);
        now += 61_000;
        assertEquals("uid123", verifier.verify(token("k2", other.getPrivate(), claims())));
        assertEquals(2, fetches);
    }

    @Test
    void oldKeysStayWhileGoogleIsDown() throws Exception {
        boolean[] down = {false};
        FirebaseTokenVerifier v = new FirebaseTokenVerifier(PROJECT, () -> {
            if (down[0]) throw new IOException("down");
            return new FirebaseTokenVerifier.Keys(Map.of("k1", good.getPublic()), 3600_000);
        }, () -> now);
        assertEquals("uid123", v.verify(token("k1", good.getPrivate(), claims())));
        down[0] = true;
        now += 2 * 3600_000; // the keys' time is up, Google can't be reached: the old keys still verify
        assertEquals("uid123", v.verify(token("k1", good.getPrivate(), claims())));

        // never fetched and Google down: every token is refused
        FirebaseTokenVerifier none = new FirebaseTokenVerifier(PROJECT, () -> {
            throw new IOException("down");
        }, () -> now);
        InvalidTokenException e = assertThrows(InvalidTokenException.class, () -> none.verify(token("k1", good.getPrivate(), claims())));
        assertEquals("token keys unavailable", e.getMessage());
        assertInstanceOf(FirebaseTokenVerifier.KeysUnavailableException.class, e, "transient, not the token's fault");
    }

    @Test
    void staleKeysRefetchInTheBackgroundAndNeverHoldUpATokenCheck() throws Exception {
        List<Runnable> queued = new ArrayList<>();
        int[] calls = {0};
        FirebaseTokenVerifier v = new FirebaseTokenVerifier(PROJECT, () -> {
            calls[0]++;
            return new FirebaseTokenVerifier.Keys(Map.of("k1", good.getPublic()), 3600_000);
        }, () -> now, queued::add);
        assertEquals("uid123", v.verify(token("k1", good.getPrivate(), claims())));
        assertEquals(1, calls[0], "the first fetch runs in the request");
        assertTrue(queued.isEmpty());

        now += 2 * 3600_000; // stale: the check answers at once with the held keys, one refetch is queued
        assertEquals("uid123", v.verify(token("k1", good.getPrivate(), claims())));
        assertEquals("uid123", v.verify(token("k1", good.getPrivate(), claims())));
        assertEquals(1, queued.size(), "one refetch at a time");
        assertEquals(1, calls[0]);

        now += 61_000; // still running (queued, not done): no second one
        v.verify(token("k1", good.getPrivate(), claims()));
        assertEquals(1, queued.size());
        queued.get(0).run();
        assertEquals(2, calls[0]);

        // fresh again: nothing more queued
        v.verify(token("k1", good.getPrivate(), claims()));
        assertEquals(1, queued.size());
    }

    @Test
    void aJustRotatedKidIsTransientNotABadToken() throws Exception {
        List<Runnable> queued = new ArrayList<>();
        boolean[] down = {false};
        FirebaseTokenVerifier v = new FirebaseTokenVerifier(PROJECT, () -> {
            if (down[0]) throw new IOException("down");
            return new FirebaseTokenVerifier.Keys(Map.copyOf(served), 3600_000);
        }, () -> now, queued::add);
        v.verify(token("k1", good.getPrivate(), claims()));

        // Google rotates in k2; the first k2 token starts the refetch and, while it runs, gets "unavailable" (503)
        served.put("k2", other.getPublic());
        now += 61_000;
        String k2 = token("k2", other.getPrivate(), claims());
        assertThrows(FirebaseTokenVerifier.KeysUnavailableException.class, () -> v.verify(k2));
        assertEquals(1, queued.size());
        queued.get(0).run();
        assertEquals("uid123", v.verify(k2), "accepted once the refetch landed");

        // refetch failed (Google down): an unknown kid stays transient, the held keys keep working
        now += 2 * 3600_000;
        down[0] = true;
        String k3 = token("k3", other.getPrivate(), claims());
        assertThrows(FirebaseTokenVerifier.KeysUnavailableException.class, () -> v.verify(k3));
        queued.get(1).run();
        assertThrows(FirebaseTokenVerifier.KeysUnavailableException.class, () -> v.verify(k3), "never accepted");
        assertEquals("uid123", v.verify(token("k1", good.getPrivate(), claims())));

        // Google answers and has no such kid: still transient (503) and never accepted; each minute at most one refetch
        down[0] = false;
        now += 61_000;
        assertThrows(FirebaseTokenVerifier.KeysUnavailableException.class, () -> v.verify(k3));
        queued.get(2).run();
        assertThrows(FirebaseTokenVerifier.KeysUnavailableException.class, () -> v.verify(k3), "throttled");
        assertEquals(3, queued.size());
        // a token signed with the wrong key for a known kid is still a bad token (401)
        InvalidTokenException e = assertThrows(InvalidTokenException.class, () -> v.verify(token("k2", good.getPrivate(), claims())));
        assertFalse(e instanceof FirebaseTokenVerifier.KeysUnavailableException);
        assertEquals("bad signature", e.getMessage());
    }

    @Test
    void googleDownAtStartIsTriedAtMostOnceAMinute() throws Exception {
        int[] calls = {0};
        FirebaseTokenVerifier v = new FirebaseTokenVerifier(PROJECT, () -> {
            calls[0]++;
            throw new IOException("down");
        }, () -> now);
        String t = token("k1", good.getPrivate(), claims());
        assertThrows(InvalidTokenException.class, () -> v.verify(t));
        assertThrows(InvalidTokenException.class, () -> v.verify(t));
        assertEquals(1, calls[0]);
        now += 61_000;
        assertThrows(InvalidTokenException.class, () -> v.verify(t));
        assertEquals(2, calls[0]);
    }

    @Test
    void parsesAJwkSet() throws Exception {
        RSAPublicKey k = (RSAPublicKey) good.getPublic();
        Base64.Encoder b = Base64.getUrlEncoder().withoutPadding();
        String jwks = "{\"keys\":[{\"kty\":\"RSA\",\"alg\":\"RS256\",\"use\":\"sig\",\"kid\":\"abc\",\"n\":\""
                + b.encodeToString(unsigned(k.getModulus())) + "\",\"e\":\"" + b.encodeToString(unsigned(k.getPublicExponent())) + "\"}]}";
        Map<String, PublicKey> keys = FirebaseTokenVerifier.parseJwks(JSON.readTree(jwks));
        assertEquals(good.getPublic(), keys.get("abc"));
    }

    private static byte[] unsigned(BigInteger v) {
        byte[] a = v.toByteArray();
        return a[0] == 0 ? java.util.Arrays.copyOfRange(a, 1, a.length) : a;
    }
}
