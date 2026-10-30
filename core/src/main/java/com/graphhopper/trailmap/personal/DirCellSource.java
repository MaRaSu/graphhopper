/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import java.io.IOException;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.TreeSet;
import java.util.stream.Stream;

/**
 * Data sets as local files, {@code <cellsDir>/<user>/<set>.csv} or {@code <cellsDir>/<user>/csv/<set>.csv}, as the
 * activity pipeline writes them locally. The version is the file's modification time, so an edited file is reloaded.
 */
public final class DirCellSource implements PersonalCellSource {

    /** Sub-folders of a user's folder that also hold data sets (the pipeline's per-sport/period CSV export). */
    private static final String[] SUBDIRS = {"", "csv"};

    private final Path cellsDir;

    public DirCellSource(Path cellsDir) {
        this.cellsDir = cellsDir.toAbsolutePath().normalize();
    }

    @Override
    public Loaded load(String user, String set) throws IOException {
        Path file = fileOf(user, set);
        if (!Files.isRegularFile(file))
            return null;
        String version = String.valueOf(Files.getLastModifiedTime(file).toMillis());
        try (Reader r = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
            return new Loaded(PersonalCells.read(r), version);
        }
    }

    @Override
    public boolean isCurrent(String user, String set, String version) throws IOException {
        Path file = fileOf(user, set);
        return Files.isRegularFile(file) && String.valueOf(Files.getLastModifiedTime(file).toMillis()).equals(version);
    }

    @Override
    public List<String> dataSets(String user) throws IOException {
        Path userDir = cellsDir.resolve(user).normalize();
        if (!userDir.startsWith(cellsDir)) return List.of();
        TreeSet<String> names = new TreeSet<>();
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

    /** {@code <user>/<set>.csv}, else {@code <user>/csv/<set>.csv}. */
    private Path fileOf(String user, String set) {
        Path first = null;
        for (String sub : SUBDIRS) {
            Path dir = cellsDir.resolve(user);
            if (!sub.isEmpty()) dir = dir.resolve(sub);
            Path f = dir.resolve(set + ".csv").normalize();
            if (!f.startsWith(cellsDir))
                throw new IllegalArgumentException("Bad user or cells name");
            if (first == null) first = f;
            if (Files.isRegularFile(f)) return f;
        }
        return first;
    }
}
