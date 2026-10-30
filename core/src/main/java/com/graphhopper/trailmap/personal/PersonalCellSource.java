/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import java.io.IOException;
import java.util.List;

/**
 * Where users' cell data sets come from: local files ({@link DirCellSource}, the previewer and tests) or the activity
 * service ({@link RemoteCellSource}, production). {@link PersonalStore} keeps what it loads in memory.
 */
public interface PersonalCellSource {

    /** One data set and the version it was loaded at. */
    final class Loaded {
        public final PersonalCells cells;
        public final String version;

        public Loaded(PersonalCells cells, String version) {
            this.cells = cells;
            this.version = version;
        }
    }

    /** The user's data set, or {@code null} when the user has none by that name (then the user routes as /route). */
    Loaded load(String user, String set) throws IOException;

    /** Whether a data set held in memory at {@code version} may be used again without loading it. */
    boolean isCurrent(String user, String set, String version) throws IOException;

    /** The names of the user's data sets, sorted; empty if none. */
    List<String> dataSets(String user) throws IOException;

    /** Housekeeping the store's background sweep calls now and then (e.g. a disk cache's eviction). */
    default void maintain() {
    }
}
