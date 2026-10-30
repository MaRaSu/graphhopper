/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.math.BigInteger;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.security.GeneralSecurityException;
import java.security.KeyFactory;
import java.security.PublicKey;
import java.security.Signature;
import java.security.spec.RSAPublicKeySpec;
import java.time.Duration;
import java.util.Base64;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Checks a Firebase ID token (a JWT signed RS256 by Google) and returns its user id, the {@code sub} claim. The same
 * rules as Firebase's Admin SDK {@code verifyIdToken}: header {@code alg} RS256 and a {@code kid} among Google's
 * current keys; {@code iss} {@code https://securetoken.google.com/<project>}; {@code aud} the project; {@code exp} in
 * the future, {@code iat} and {@code auth_time} not in the future (60 s clock skew); {@code sub} non-empty. No
 * service account key is needed: Google's public keys are fetched from {@link #GOOGLE_JWKS} and kept for as long as
 * its {@code Cache-Control: max-age} says (at least 5 minutes). After that, or for an unknown {@code kid}, they are
 * refetched in the background at most once a minute while the held keys go on verifying (see {@link #key}).
 */
public final class FirebaseTokenVerifier {

    private static final Logger LOGGER = LoggerFactory.getLogger(FirebaseTokenVerifier.class);
    public static final String GOOGLE_JWKS = "https://www.googleapis.com/service_accounts/v1/jwk/securetoken@system.gserviceaccount.com";
    static final long SKEW_SECONDS = 60;
    private static final long MIN_KEYS_TTL_MS = 5 * 60_000, MIN_REFETCH_GAP_MS = 60_000;
    private static final Pattern MAX_AGE = Pattern.compile("max-age=(\\d+)");
    private static final ObjectMapper JSON = new ObjectMapper();

    /** A token that is not a valid Firebase ID token of this project; the message says why (never the token). */
    public static class InvalidTokenException extends Exception {
        public InvalidTokenException(String message) {
            super(message);
        }
    }

    /** Not the token's fault: Google's keys could not be fetched yet, so no token can be checked now (transient). */
    public static final class KeysUnavailableException extends InvalidTokenException {
        public KeysUnavailableException() {
            super("token keys unavailable");
        }
    }

    /** Google's current public keys by {@code kid}, and how long they may be kept. */
    public interface KeySource {
        Keys fetch() throws IOException;
    }

    public static final class Keys {
        final Map<String, PublicKey> byKid;
        final long ttlMillis;

        public Keys(Map<String, PublicKey> byKid, long ttlMillis) {
            this.byKid = byKid;
            this.ttlMillis = ttlMillis;
        }
    }

    public interface Clock {
        long nowMillis();
    }

    private final String projectId;
    private final KeySource keySource;
    private final Clock clock;
    /** Runs the refetches once keys are held, so a token check never waits for Google. */
    private final Executor refresher;
    private Keys keys;
    private long keysFetchedAt = Long.MIN_VALUE / 2;
    private long lastAttempt = Long.MIN_VALUE / 2;
    private boolean refreshing;
    private boolean lastFetchFailed;

    /** Refetches run in the calling thread (tests). */
    public FirebaseTokenVerifier(String projectId, KeySource keySource, Clock clock) {
        this(projectId, keySource, clock, Runnable::run);
    }

    public FirebaseTokenVerifier(String projectId, KeySource keySource, Clock clock, Executor refresher) {
        if (projectId == null || projectId.isBlank())
            throw new IllegalArgumentException("Firebase project id is required");
        this.projectId = projectId;
        this.keySource = keySource;
        this.clock = clock;
        this.refresher = refresher;
    }

    /** Verifies against Google's live keys, refetched in a background thread. */
    public static FirebaseTokenVerifier forProject(String projectId) {
        Executor background = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "personal-token-keys");
            t.setDaemon(true);
            return t;
        });
        return new FirebaseTokenVerifier(projectId, googleKeys(GOOGLE_JWKS), System::currentTimeMillis, background);
    }

    /** The user id of a valid token. */
    public String verify(String token) throws InvalidTokenException {
        if (token == null || token.isEmpty())
            throw new InvalidTokenException("no token");
        String[] parts = token.split("\\.", -1);
        if (parts.length != 3)
            throw new InvalidTokenException("not a JWT");
        JsonNode header = decodeJson(parts[0], "header");
        JsonNode claims = decodeJson(parts[1], "payload");
        if (!"RS256".equals(header.path("alg").asText()))
            throw new InvalidTokenException("alg must be RS256");
        String kid = header.path("kid").asText("");
        if (kid.isEmpty())
            throw new InvalidTokenException("no kid");

        PublicKey key = key(kid);
        if (key == null)
            throw new InvalidTokenException("unknown kid");
        byte[] sig;
        try {
            sig = Base64.getUrlDecoder().decode(parts[2]);
        } catch (IllegalArgumentException e) {
            throw new InvalidTokenException("bad signature encoding");
        }
        try {
            Signature s = Signature.getInstance("SHA256withRSA");
            s.initVerify(key);
            s.update((parts[0] + "." + parts[1]).getBytes(StandardCharsets.US_ASCII));
            if (!s.verify(sig))
                throw new InvalidTokenException("bad signature");
        } catch (GeneralSecurityException e) {
            throw new InvalidTokenException("bad signature");
        }

        long now = clock.nowMillis() / 1000;
        if (!("https://securetoken.google.com/" + projectId).equals(claims.path("iss").asText()))
            throw new InvalidTokenException("wrong iss");
        if (!projectId.equals(claims.path("aud").asText()))
            throw new InvalidTokenException("wrong aud");
        if (!claims.path("exp").canConvertToLong() || claims.path("exp").asLong() <= now - SKEW_SECONDS)
            throw new InvalidTokenException("expired");
        if (!claims.path("iat").canConvertToLong() || claims.path("iat").asLong() > now + SKEW_SECONDS)
            throw new InvalidTokenException("issued in the future");
        if (claims.has("auth_time") && claims.path("auth_time").asLong() > now + SKEW_SECONDS)
            throw new InvalidTokenException("auth_time in the future");
        String sub = claims.path("sub").asText("");
        if (sub.isEmpty() || sub.length() > 128)
            throw new InvalidTokenException("bad sub");
        return sub;
    }

    /**
     * The key for {@code kid}. Only the very first fetch (no keys yet) runs in the request, at most once a minute.
     * Once keys are held, a stale set or an unknown kid (Google rotates keys) starts a refetch in the background, at
     * most once a minute, and the request goes on with the keys held: while Google can't be reached the old keys keep
     * working, and no token check waits for it.
     */
    private PublicKey key(String kid) throws InvalidTokenException {
        boolean refresh;
        synchronized (this) {
            long now = clock.nowMillis();
            if (keys == null) {
                if (now - lastAttempt < MIN_REFETCH_GAP_MS)
                    throw new KeysUnavailableException();
                lastAttempt = now;
                if (!fetch())
                    throw new KeysUnavailableException();
                return keys.byKid.get(kid);
            }
            boolean stale = now - keysFetchedAt >= keys.ttlMillis;
            refresh = (stale || !keys.byKid.containsKey(kid)) && !refreshing && now - lastAttempt >= MIN_REFETCH_GAP_MS;
            if (refresh) {
                lastAttempt = now;
                refreshing = true;
            }
        }
        if (refresh) {
            try {
                refresher.execute(() -> {
                    try {
                        fetch(); // the network call holds no lock, so token checks go on meanwhile
                    } finally {
                        synchronized (this) {
                            refreshing = false;
                        }
                    }
                });
            } catch (RuntimeException e) {
                synchronized (this) {
                    refreshing = false;
                }
            }
        }
        synchronized (this) {
            PublicKey k = keys.byKid.get(kid);
            // a kid Google may have just rotated in, while the refetch runs, is throttled (last try < 1 min ago) or
            // failed: transient, never a bad token (clients turn personal routing off on 401); never accepted without
            // its key
            if (k == null && (refreshing || lastFetchFailed || clock.nowMillis() - lastAttempt < MIN_REFETCH_GAP_MS))
                throw new KeysUnavailableException();
            return k;
        }
    }

    /** One fetch of Google's keys; false (logged) when it failed, and the keys held stay. */
    private boolean fetch() {
        try {
            Keys fetched = keySource.fetch();
            synchronized (this) {
                keys = new Keys(fetched.byKid, Math.max(MIN_KEYS_TTL_MS, fetched.ttlMillis));
                keysFetchedAt = clock.nowMillis();
                lastFetchFailed = false;
            }
            return true;
        } catch (IOException | RuntimeException e) {
            LOGGER.warn("personal: fetching Google's token keys failed: {}", e.toString());
            synchronized (this) {
                lastFetchFailed = true;
            }
            return false;
        }
    }

    private static JsonNode decodeJson(String part, String what) throws InvalidTokenException {
        try {
            JsonNode n = JSON.readTree(Base64.getUrlDecoder().decode(part));
            if (n == null || !n.isObject())
                throw new InvalidTokenException("bad " + what);
            return n;
        } catch (IllegalArgumentException | IOException e) {
            throw new InvalidTokenException("bad " + what);
        }
    }

    /** Google's JWK set at {@code url}, with its {@code Cache-Control: max-age}. */
    static KeySource googleKeys(String url) {
        HttpClient client = HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(2)).build();
        return () -> {
            HttpRequest req = HttpRequest.newBuilder(URI.create(url)).timeout(Duration.ofSeconds(3)).GET().build();
            HttpResponse<byte[]> rsp;
            // the request timeout ends at the response headers; this bounds the body read too
            CompletableFuture<HttpResponse<byte[]>> f = client.sendAsync(req, HttpResponse.BodyHandlers.ofByteArray());
            try {
                rsp = f.get(5, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                f.cancel(true);
                Thread.currentThread().interrupt();
                throw new IOException("interrupted", e);
            } catch (TimeoutException e) {
                f.cancel(true);
                throw new IOException("no full answer from " + url + " in 5 s", e);
            } catch (ExecutionException e) {
                throw e.getCause() instanceof IOException ? (IOException) e.getCause() : new IOException(e.getCause());
            }
            if (rsp.statusCode() != 200)
                throw new IOException("HTTP " + rsp.statusCode() + " from " + url);
            long ttl = MIN_KEYS_TTL_MS;
            Matcher m = MAX_AGE.matcher(rsp.headers().firstValue("cache-control").orElse(""));
            if (m.find())
                ttl = Long.parseLong(m.group(1)) * 1000;
            return new Keys(parseJwks(JSON.readTree(rsp.body())), ttl);
        };
    }

    /** The RSA keys of a JWK set ({@code keys[].kid, n, e}). */
    static Map<String, PublicKey> parseJwks(JsonNode jwks) throws IOException {
        Map<String, PublicKey> out = new HashMap<>();
        try {
            KeyFactory kf = KeyFactory.getInstance("RSA");
            for (JsonNode k : jwks.path("keys")) {
                if (!"RSA".equals(k.path("kty").asText()) || k.path("kid").asText("").isEmpty())
                    continue;
                BigInteger n = new BigInteger(1, Base64.getUrlDecoder().decode(k.path("n").asText()));
                BigInteger e = new BigInteger(1, Base64.getUrlDecoder().decode(k.path("e").asText()));
                out.put(k.path("kid").asText(), kf.generatePublic(new RSAPublicKeySpec(n, e)));
            }
        } catch (GeneralSecurityException | IllegalArgumentException e) {
            throw new IOException("bad JWK set", e);
        }
        if (out.isEmpty())
            throw new IOException("JWK set has no RSA keys");
        return out;
    }
}
