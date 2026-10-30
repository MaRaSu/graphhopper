/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

/**
 * The grid the activity history is stored in: standard Web-Mercator XYZ tile cells at a fixed zoom
 * (default 20, about 19 m at 60°N). A cell id is {@code x * 2^zoom + y}, the same number the
 * activity pipeline (user-dw) writes, so both sides compute identical ids without a shared library.
 */
public final class PersonalCellGrid {

    public static final int DEFAULT_ZOOM = 20;

    private final int zoom;
    private final double n;

    public PersonalCellGrid(int zoom) {
        if (zoom < 10 || zoom > 24)
            throw new IllegalArgumentException("Cell zoom must be 10..24, was " + zoom);
        this.zoom = zoom;
        this.n = Math.pow(2, zoom);
    }

    public int getZoom() {
        return zoom;
    }

    public long cellX(double lon) {
        long x = (long) Math.floor((lon + 180.0) / 360.0 * n);
        return Math.max(0, Math.min((long) n - 1, x));
    }

    public long cellY(double lat) {
        double latRad = Math.toRadians(lat);
        long y = (long) Math.floor((1.0 - Math.log(Math.tan(latRad) + 1.0 / Math.cos(latRad)) / Math.PI) / 2.0 * n);
        return Math.max(0, Math.min((long) n - 1, y));
    }

    public long cellId(double lat, double lon) {
        return id(cellX(lon), cellY(lat));
    }

    public long id(long x, long y) {
        return x * (long) n + y;
    }

    public long xOf(long id) {
        return id / (long) n;
    }

    public long yOf(long id) {
        return id % (long) n;
    }

    /** West longitude of column x. */
    public double lonOf(long x) {
        return x / n * 360.0 - 180.0;
    }

    /** North latitude of row y. */
    public double latOf(long y) {
        double m = Math.PI - 2.0 * Math.PI * y / n;
        return Math.toDegrees(Math.atan(Math.sinh(m)));
    }
}
