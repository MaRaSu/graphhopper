/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.UncheckedIOException;
import java.net.URI;
import java.net.URLEncoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.nio.file.attribute.FileTime;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.TreeSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import java.util.zip.GZIPInputStream;
import java.util.zip.GZIPOutputStream;

/**
 * Data sets fetched from the activity service, with a disk copy.
 *
 * <p>{@code GET {source}/{uid}/{set}} returns the set as gzipped CSV (the {@link PersonalCells} format) with an
 * {@code ETag}; 304 for a matching {@code If-None-Match}; 204 when the user has no cells; 404 for an unknown set.
 * {@code GET {source}/{uid}} returns the user's index ({@code sets}). Every call carries the shared secret.
 *
 * <p>The disk copy, {@code <diskDir>/<uid>/<set>.csv.gz} plus {@code <set>.meta} (ETag and size), lets a returning user
 * cost a 304 instead of a download, and keeps routing personal while the activity service is down. The disk may be
 * node-local and briefly shared by two server processes, so every file is written to a temporary file and renamed, and
 * a copy is used only when its size matches its meta and it parses. Copies unused for {@code diskMaxAgeMillis} are
 * deleted by {@link #maintain}.
 *
 * <p>{@link PersonalStore} calls {@link #load} only when the set is not in memory, so a user who keeps routing is
 * served from memory and revalidated after the store has dropped them (a few idle minutes).
 */
public final class RemoteCellSource implements PersonalCellSource {

    private static final Logger LOGGER = LoggerFactory.getLogger(RemoteCellSource.class);
    private static final ObjectMapper JSON = new ObjectMapper();
    private static final Pattern UID = Pattern.compile("[A-Za-z0-9_-]{1,128}");
    /** Data set names become file names in the user's disk folder, so nothing that could leave it. */
    private static final Pattern SET = Pattern.compile("[A-Za-z0-9_-]{1,64}");
    private static final long MAINTAIN_EVERY_MS = 60 * 60_000L;

    private final String source;
    private final String secret;
    private final Path diskDir;
    private final long diskMaxAgeMillis;
    private final HttpClient client;
    private final Duration requestTimeout;
    private final AtomicBoolean firstFetchLogged = new AtomicBoolean();
    private volatile long lastMaintain;

    public RemoteCellSource(String source, String secret, Path diskDir, long diskMaxAgeMillis, Duration connectTimeout,
                            Duration requestTimeout) {
        if (source == null || !(source.startsWith("https://") || source.startsWith("http://")))
            throw new IllegalArgumentException("personal_routing.cells_source must be an http(s) URL");
        if (secret == null || secret.isEmpty())
            throw new IllegalArgumentException("the cells secret is required with personal_routing.cells_source");
        this.source = source.endsWith("/") ? source.substring(0, source.length() - 1) : source;
        this.secret = secret;
        this.diskDir = diskDir == null ? null : diskDir.toAbsolutePath().normalize();
        this.diskMaxAgeMillis = diskMaxAgeMillis;
        this.client = HttpClient.newBuilder().connectTimeout(connectTimeout).followRedirects(HttpClient.Redirect.NEVER).build();
        this.requestTimeout = requestTimeout;
    }

    @Override
    public Loaded load(String uid, String set) throws IOException {
        checkUid(uid);
        if (set == null || !SET.matcher(set).matches())
            throw new IllegalArgumentException("bad data set name");
        Copy disk = readDisk(uid, set);
        HttpResponse<byte[]> rsp;
        long t0 = System.currentTimeMillis();
        try {
            HttpRequest.Builder b = request(source + "/" + enc(uid) + "/" + enc(set));
            if (disk != null) b.header("If-None-Match", disk.etag);
            rsp = send(b.build());
        } catch (IOException e) {
            logFetch(set, "failed: " + e, t0);
            return fallback(uid, set, disk);
        }
        int status = rsp.statusCode();
        logFetch(set, "HTTP " + status + " " + rsp.body().length + " B", t0);
        switch (status) {
            case 304:
                if (disk == null) return null; // cannot happen without If-None-Match; route plain
                touch(disk);
                return new Loaded(disk.cells, disk.etag);
            case 200: {
                String etag = rsp.headers().firstValue("ETag").orElse("");
                PersonalCells cells;
                byte[] gz;
                try {
                    gz = isGzip(rsp.body()) ? rsp.body() : gzip(rsp.body());
                    cells = parse(gz); // a body that does not parse never reaches the disk
                } catch (IOException | RuntimeException e) {
                    // the class only: a parse message can quote a field of the user's cells
                    LOGGER.warn("personal: the cells of {} did not parse ({})", set, e.getClass().getSimpleName());
                    return fallback(uid, set, disk);
                }
                writeDisk(uid, set, gz, etag);
                return new Loaded(cells, etag);
            }
            case 204:
            case 404:
                // no cells / no such set: routes as /route, and an old copy must not keep routing personal
                deleteDisk(uid, set);
                return null;
            default:
                return fallback(uid, set, disk);
        }
    }

    /** Memory entries stay until the store drops them; revalidation happens at the next load. */
    @Override
    public boolean isCurrent(String uid, String set, String version) {
        return true;
    }

    @Override
    public List<String> dataSets(String uid) throws IOException {
        checkUid(uid);
        HttpResponse<byte[]> rsp = send(request(source + "/" + enc(uid)).build());
        if (rsp.statusCode() == 204 || rsp.statusCode() == 404) return List.of();
        if (rsp.statusCode() != 200) throw new IOException("cells index: HTTP " + rsp.statusCode());
        // the activity service's index: {"sets": {"<group>_<season>": {...}, ...}, ...}
        TreeSet<String> names = new TreeSet<>();
        JSON.readTree(rsp.body()).path("sets").fieldNames().forEachRemaining(names::add);
        return List.copyOf(names);
    }

    /** Deletes disk copies unused for longer than the maximum age; at most once an hour. */
    @Override
    public void maintain() {
        long now = System.currentTimeMillis();
        if (diskDir == null || now - lastMaintain < MAINTAIN_EVERY_MS) return;
        lastMaintain = now;
        if (!Files.isDirectory(diskDir)) return;
        int deleted = 0;
        try (Stream<Path> files = Files.walk(diskDir, 2)) {
            for (Path f : (Iterable<Path>) files::iterator) {
                String n = f.getFileName().toString();
                if (!Files.isRegularFile(f) || !(n.endsWith(".csv.gz") || n.endsWith(".meta") || n.contains(".tmp")))
                    continue;
                try {
                    if (now - Files.getLastModifiedTime(f).toMillis() > diskMaxAgeMillis && Files.deleteIfExists(f))
                        deleted++;
                } catch (IOException e) {
                    // another process may have just replaced or removed it
                }
            }
        } catch (IOException | UncheckedIOException e) {
            LOGGER.warn("personal: cells disk cleanup failed: {}", e.toString());
        }
        if (deleted > 0) LOGGER.info("personal: deleted {} cells disk files unused for {} days", deleted, diskMaxAgeMillis / 86_400_000L);
    }

    /**
     * One call, bounded as a whole: the request timeout alone ends at the response headers, so a body that stalls
     * would hold the routing request; this caps headers and body together.
     */
    private HttpResponse<byte[]> send(HttpRequest req) throws IOException {
        CompletableFuture<HttpResponse<byte[]>> f = client.sendAsync(req, HttpResponse.BodyHandlers.ofByteArray());
        try {
            return f.get(requestTimeout.toMillis(), TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            f.cancel(true);
            Thread.currentThread().interrupt();
            throw new IOException("interrupted", e);
        } catch (TimeoutException e) {
            f.cancel(true);
            throw new IOException("no full answer in " + requestTimeout.toMillis() + " ms", e);
        } catch (ExecutionException e) {
            throw e.getCause() instanceof IOException ? (IOException) e.getCause() : new IOException(e.getCause());
        }
    }

    private HttpRequest.Builder request(String url) {
        return HttpRequest.newBuilder(URI.create(url)).timeout(requestTimeout).header("Authorization", "Bearer " + secret).GET();
    }

    private Loaded fallback(String uid, String set, Copy disk) {
        if (disk == null) {
            LOGGER.warn("personal: activity service unavailable and no disk copy of {}; routing without history", set);
            return null;
        }
        LOGGER.warn("personal: activity service unavailable; using the disk copy of {} ({})", set, disk.etag);
        return new Loaded(disk.cells, disk.etag);
    }

    private void logFetch(String set, String result, long t0) {
        long ms = System.currentTimeMillis() - t0;
        if (firstFetchLogged.compareAndSet(false, true))
            LOGGER.info("personal: FIRST cells fetch from {} since start: set {}, {}, {} ms", source, set, result, ms);
        else
            LOGGER.info("personal: cells fetch set {}: {}, {} ms", set, result, ms);
    }

    // --- disk copy ---

    private static final class Copy {
        final PersonalCells cells;
        final String etag;
        final Path gz, meta;

        Copy(PersonalCells cells, String etag, Path gz, Path meta) {
            this.cells = cells;
            this.etag = etag;
            this.gz = gz;
            this.meta = meta;
        }
    }

    private Path gzOf(String uid, String set) {
        return diskDir.resolve(uid).resolve(set + ".csv.gz");
    }

    private Path metaOf(String uid, String set) {
        return diskDir.resolve(uid).resolve(set + ".meta");
    }

    /** The disk copy if it is complete and parses, else null. */
    private Copy readDisk(String uid, String set) {
        if (diskDir == null) return null;
        Path gz = gzOf(uid, set), meta = metaOf(uid, set);
        try {
            if (!Files.isRegularFile(gz) || !Files.isRegularFile(meta)) return null;
            List<String> m = Files.readAllLines(meta, StandardCharsets.UTF_8);
            if (m.size() < 2 || m.get(0).isEmpty()) return null;
            byte[] bytes = Files.readAllBytes(gz);
            if (bytes.length != Long.parseLong(m.get(1).trim())) return null; // the other half of a concurrent write
            return new Copy(parse(bytes), m.get(0), gz, meta);
        } catch (IOException | RuntimeException e) {
            LOGGER.warn("personal: ignoring a broken disk copy of {} ({})", set, e.getClass().getSimpleName());
            return null;
        }
    }

    private void writeDisk(String uid, String set, byte[] gz, String etag) {
        if (diskDir == null || etag.isEmpty()) return; // without an ETag a copy could never be revalidated
        try {
            Path dir = diskDir.resolve(uid);
            Files.createDirectories(dir);
            moveIn(dir, gz, gzOf(uid, set));
            moveIn(dir, (etag + "\n" + gz.length + "\n").getBytes(StandardCharsets.UTF_8), metaOf(uid, set));
        } catch (IOException e) {
            LOGGER.warn("personal: could not keep a disk copy of {}: {}", set, e.toString());
        }
    }

    private static void moveIn(Path dir, byte[] bytes, Path target) throws IOException {
        Path tmp = Files.createTempFile(dir, target.getFileName().toString(), ".tmp");
        try {
            Files.write(tmp, bytes);
            Files.move(tmp, target, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
        } finally {
            Files.deleteIfExists(tmp);
        }
    }

    private void deleteDisk(String uid, String set) {
        if (diskDir == null) return;
        try {
            Files.deleteIfExists(metaOf(uid, set));
            Files.deleteIfExists(gzOf(uid, set));
        } catch (IOException e) {
            LOGGER.warn("personal: could not delete the disk copy of {}: {}", set, e.toString());
        }
    }

    private static void touch(Copy c) {
        FileTime now = FileTime.fromMillis(System.currentTimeMillis());
        try {
            Files.setLastModifiedTime(c.gz, now);
            Files.setLastModifiedTime(c.meta, now);
        } catch (IOException e) {
            // only the eviction age is affected
        }
    }

    // --- helpers ---

    private static void checkUid(String uid) {
        if (uid == null || !UID.matcher(uid).matches())
            throw new IllegalArgumentException("bad user id");
    }

    private static String enc(String s) {
        return URLEncoder.encode(s, StandardCharsets.UTF_8);
    }

    private static boolean isGzip(byte[] b) {
        return b.length >= 2 && (b[0] & 0xff) == 0x1f && (b[1] & 0xff) == 0x8b;
    }

    private static byte[] gzip(byte[] plain) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream(plain.length / 4 + 64);
        try (GZIPOutputStream z = new GZIPOutputStream(out)) {
            z.write(plain);
        }
        return out.toByteArray();
    }

    static PersonalCells parse(byte[] gz) throws IOException {
        try (Reader r = new InputStreamReader(new GZIPInputStream(new ByteArrayInputStream(gz)), StandardCharsets.UTF_8)) {
            return PersonalCells.read(r);
        }
    }

    // for tests
    List<Path> diskFiles() throws IOException {
        List<Path> out = new ArrayList<>();
        if (diskDir == null || !Files.isDirectory(diskDir)) return out;
        try (Stream<Path> s = Files.walk(diskDir)) {
            s.filter(Files::isRegularFile).forEach(out::add);
        }
        return out;
    }
}
