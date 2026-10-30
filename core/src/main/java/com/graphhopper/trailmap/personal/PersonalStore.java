/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.graphhopper.storage.BaseGraph;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

/**
 * Users' cell data and their lazily filled edge familiarity, kept in memory for a while.
 *
 * <p>Prototype source: one file per user and data set, {@code <cellsDir>/<user>/<cells>.csv}, as
 * written by the activity pipeline. A production version reads the same rows from the user
 * database; only {@link #load} changes.
 *
 * <p>An entry is reused until it has not been used for {@code ttlMillis}, the file changes, or more
 * than {@code maxEntries} users are held (least recently used goes first). Each entry holds two
 * bytes per graph edge (about 10 MB for Finland) plus the cells.
 */
public final class PersonalStore {

    private static final Logger LOGGER = LoggerFactory.getLogger(PersonalStore.class);

    private final Path cellsDir;
    private final BaseGraph graph;
    private final long ttlMillis;
    private final int maxEntries;
    private final Map<String, Entry> entries = new LinkedHashMap<>(16, 0.75f, true);

    public PersonalStore(Path cellsDir, BaseGraph graph, long ttlMillis, int maxEntries) {
        this.cellsDir = cellsDir;
        this.graph = graph;
        this.ttlMillis = ttlMillis;
        this.maxEntries = Math.max(1, maxEntries);
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
        final long fileMillis;
        long lastUsed;

        Entry(PersonalFamiliarity familiarity, long fileMillis) {
            this.familiarity = familiarity;
            this.fileMillis = fileMillis;
        }
    }

    /** The user's familiarity, or {@code null} when the user has no such data set (routes as /route). */
    public synchronized Lookup get(String user, String cells) throws IOException {
        long now = System.currentTimeMillis();
        evictExpired(now);
        Path file = fileOf(user, cells);
        if (!Files.isRegularFile(file))
            return null;
        long fileMillis = Files.getLastModifiedTime(file).toMillis();
        String key = user + "/" + cells;
        Entry e = entries.get(key);
        if (e != null && e.fileMillis == fileMillis) {
            e.lastUsed = now;
            return new Lookup(e.familiarity, true, 0);
        }
        long t0 = System.currentTimeMillis();
        PersonalCells c = load(file);
        e = new Entry(new PersonalFamiliarity(c, graph), fileMillis);
        e.lastUsed = now;
        entries.put(key, e);
        while (entries.size() > maxEntries) {
            Iterator<String> it = entries.keySet().iterator();
            LOGGER.info("personal: dropping {} (store full)", it.next());
            it.remove();
        }
        long ms = System.currentTimeMillis() - t0;
        LOGGER.info("personal: loaded {} ({} cells, zoom {}) in {} ms", key, c.size(), c.getGrid().getZoom(), ms);
        return new Lookup(e.familiarity, false, ms);
    }

    /** The cells only, for the previewer's cell layer. */
    public PersonalCells cells(String user, String cells) throws IOException {
        Lookup l = get(user, cells);
        if (l == null)
            throw new IllegalArgumentException("No cell data '" + cells + "' for user '" + user + "'");
        return l.familiarity.getCells();
    }

    /** Sub-folders of a user's folder that also hold data sets (the pipeline's per-sport/period CSV export). */
    private static final String[] SUBDIRS = {"", "csv"};

    /** The data sets a user has (file names without .csv), sorted; empty if none. */
    public List<String> dataSets(String user) throws IOException {
        Path userDir = cellsDir.resolve(user).normalize();
        if (!userDir.startsWith(cellsDir.normalize())) return List.of();
        java.util.TreeSet<String> names = new java.util.TreeSet<>();
        for (String sub : SUBDIRS) {
            Path dir = sub.isEmpty() ? userDir : userDir.resolve(sub);
            if (!Files.isDirectory(dir)) continue;
            try (Stream<Path> s = Files.list(dir)) {
                s.map(p -> p.getFileName().toString())
                        .filter(n -> n.endsWith(".csv") && !n.endsWith("_by_activity.csv") && !n.endsWith("_activities.csv"))
                        .forEach(n -> names.add(n.substring(0, n.length() - 4)));
            }
        }
        return List.copyOf(names);
    }

    /** {@code <user>/<cells>.csv}, else {@code <user>/csv/<cells>.csv}. */
    private Path fileOf(String user, String cells) {
        Path first = null;
        for (String sub : SUBDIRS) {
            Path dir = cellsDir.resolve(user);
            if (!sub.isEmpty()) dir = dir.resolve(sub);
            Path f = dir.resolve(cells + ".csv").normalize();
            if (!f.startsWith(cellsDir.normalize()))
                throw new IllegalArgumentException("Bad user or cells name");
            if (first == null) first = f;
            if (Files.isRegularFile(f)) return f;
        }
        return first;
    }

    private static PersonalCells load(Path file) throws IOException {
        try (Reader r = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
            return PersonalCells.read(r);
        }
    }

    private void evictExpired(long now) {
        entries.entrySet().removeIf(en -> {
            boolean old = now - en.getValue().lastUsed > ttlMillis;
            if (old) LOGGER.info("personal: dropping {} (unused for {} min)", en.getKey(), ttlMillis / 60000);
            return old;
        });
    }
}
