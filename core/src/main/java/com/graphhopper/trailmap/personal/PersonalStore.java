/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.graphhopper.storage.BaseGraph;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Users' cell data and their lazily filled edge familiarity, kept in memory for a while.
 *
 * <p>The data sets come from a {@link PersonalCellSource}: local files ({@link DirCellSource}) or the activity service
 * ({@link RemoteCellSource}).
 *
 * <p>An entry is reused until it has not been used for {@code ttlMillis}, the source says it changed, or more
 * than {@code maxEntries} users are held (least recently used goes first). Each entry holds two
 * bytes per graph edge (about 10 MB for Finland) plus the cells. A background sweep drops idle
 * entries even when no further request comes, so a user who stops routing frees the memory within
 * about {@code ttlMillis} plus one sweep interval.
 */
public final class PersonalStore implements AutoCloseable {

    private static final Logger LOGGER = LoggerFactory.getLogger(PersonalStore.class);

    private final PersonalCellSource source;
    private final BaseGraph graph;
    private final long ttlMillis;
    private final int maxEntries;
    private final Map<String, Entry> entries = new LinkedHashMap<>(16, 0.75f, true);
    private final ScheduledExecutorService sweeper;

    public PersonalStore(Path cellsDir, BaseGraph graph, long ttlMillis, int maxEntries) {
        this(new DirCellSource(cellsDir), graph, ttlMillis, maxEntries);
    }

    public PersonalStore(PersonalCellSource source, BaseGraph graph, long ttlMillis, int maxEntries) {
        this.source = source;
        this.graph = graph;
        this.ttlMillis = ttlMillis;
        this.maxEntries = Math.max(1, maxEntries);
        this.sweeper = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "personal-store-sweep");
            t.setDaemon(true);
            return t;
        });
        long every = sweepIntervalMillis(ttlMillis);
        sweeper.scheduleWithFixedDelay(this::sweep, every, every, TimeUnit.MILLISECONDS);
    }

    /** A fifth of the time to live, between 1 s and 60 s. */
    static long sweepIntervalMillis(long ttlMillis) {
        return Math.max(1_000, Math.min(60_000, ttlMillis / 5));
    }

    /** Drops the entries idle for longer than the time to live, then lets the source tidy up; the sweeper calls it. */
    void sweep() {
        try {
            synchronized (this) {
                evictExpired(System.currentTimeMillis());
            }
            source.maintain();
        } catch (RuntimeException e) {
            LOGGER.warn("personal: sweep failed", e);
        }
    }

    /** The number of users' data sets held now. */
    public synchronized int size() {
        return entries.size();
    }

    /** Stops the sweep and drops every entry. */
    @Override
    public synchronized void close() {
        sweeper.shutdownNow();
        entries.clear();
    }

    /** What a lookup returns: the familiarity plus whether it was already in memory. */
    public static final class Lookup {
        public final PersonalFamiliarity familiarity;
        public final boolean cached;
        public final long loadMillis;

        Lookup(PersonalFamiliarity familiarity, boolean cached, long loadMillis) {
            this.familiarity = familiarity;
            this.cached = cached;
            this.loadMillis = loadMillis;
        }
    }

    private static final class Entry {
        final PersonalFamiliarity familiarity;
        final String version;
        long lastUsed;

        Entry(PersonalFamiliarity familiarity, String version) {
            this.familiarity = familiarity;
            this.version = version;
        }
    }

    /** The user's familiarity, or {@code null} when the user has no such data set (routes as /route). */
    public Lookup get(String user, String cells) throws IOException {
        String key = user + "/" + cells;
        synchronized (this) {
            long now = System.currentTimeMillis();
            evictExpired(now);
            Entry e = entries.get(key);
            if (e != null && source.isCurrent(user, cells, e.version)) {
                e.lastUsed = now;
                return new Lookup(e.familiarity, true, 0);
            }
        }
        // loaded outside the lock: a fetch from the activity service must not hold up other users' requests
        long t0 = System.currentTimeMillis();
        PersonalCellSource.Loaded loaded = source.load(user, cells);
        synchronized (this) {
            if (loaded == null) {
                entries.remove(key);
                return null;
            }
            Entry e = new Entry(new PersonalFamiliarity(loaded.cells, graph), loaded.version);
            e.lastUsed = System.currentTimeMillis();
            entries.put(key, e);
            while (entries.size() > maxEntries) {
                Iterator<String> it = entries.keySet().iterator();
                LOGGER.info("personal: dropping {} (store full)", it.next());
                it.remove();
            }
            long ms = System.currentTimeMillis() - t0;
            LOGGER.info("personal: loaded {} ({} cells, zoom {}) in {} ms", cells, loaded.cells.size(), loaded.cells.getGrid().getZoom(), ms);
            return new Lookup(e.familiarity, false, ms);
        }
    }

    /** The cells only, for the previewer's cell layer. */
    public PersonalCells cells(String user, String cells) throws IOException {
        Lookup l = get(user, cells);
        if (l == null)
            throw new IllegalArgumentException("No cell data '" + cells + "' for user '" + user + "'");
        return l.familiarity.getCells();
    }

    /** The data sets a user has, sorted; empty if none. */
    public List<String> dataSets(String user) throws IOException {
        return source.dataSets(user);
    }

    private void evictExpired(long now) {
        entries.entrySet().removeIf(en -> {
            boolean old = now - en.getValue().lastUsed > ttlMillis;
            if (old) LOGGER.info("personal: dropping {} (unused for {} min)", en.getKey(), ttlMillis / 60000);
            return old;
        });
    }
}
