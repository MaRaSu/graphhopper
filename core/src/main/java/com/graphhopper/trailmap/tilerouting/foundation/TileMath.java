package com.graphhopper.trailmap.tilerouting.foundation;

/**
 * Web-Mercator slippy-map tiles (VeloViewer / Squadrats z14, squadratinhos z17). A tile is a lon/lat-aligned
 * rectangle, so its inset square (shrunk by a margin in metres) is a lon/lat rectangle too.
 */
public final class TileMath {

    public static final double M_PER_DEG_LAT = 110_540;
    public static final double M_PER_DEG_LON_EQ = 111_320;

    private TileMath() {
    }

    public static int lonToX(double lon, int z) {
        return (int) Math.floor((lon + 180) / 360 * (1 << z));
    }

    public static int latToY(double lat, int z) {
        double r = Math.toRadians(lat);
        return (int) Math.floor((1 - asinh(Math.tan(r)) / Math.PI) / 2 * (1 << z));
    }

    public static double xToLon(int x, int z) {
        return (double) x / (1 << z) * 360 - 180;
    }

    public static double yToLat(int y, int z) {
        return Math.toDegrees(Math.atan(Math.sinh(Math.PI * (1 - 2.0 * y / (1 << z)))));
    }

    private static double asinh(double v) {
        return Math.log(v + Math.sqrt(v * v + 1));
    }

    /** One tile with its full and inset rectangle. Key is Squadrats' "x-y". */
    public static final class Tile {
        public final int z, x, y;
        public final String key;
        public final double minLon, minLat, maxLon, maxLat;
        public final double inMinLon, inMinLat, inMaxLon, inMaxLat;
        public final double marginM;

        public Tile(int z, int x, int y, double marginM) {
            this.z = z;
            this.x = x;
            this.y = y;
            this.key = x + "-" + y;
            this.marginM = marginM;
            minLon = xToLon(x, z);
            maxLon = xToLon(x + 1, z);
            maxLat = yToLat(y, z);
            minLat = yToLat(y + 1, z);
            double cLat = (minLat + maxLat) / 2;
            double dLat = marginM / M_PER_DEG_LAT;
            double dLon = marginM / (M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(cLat)));
            inMinLat = minLat + dLat;
            inMaxLat = maxLat - dLat;
            inMinLon = minLon + dLon;
            inMaxLon = maxLon - dLon;
        }

        public static Tile parse(String key, int z, double marginM) {
            String[] p = key.split("-");
            return new Tile(z, Integer.parseInt(p[0]), Integer.parseInt(p[1]), marginM);
        }

        public static Tile containing(double lat, double lon, int z, double marginM) {
            return new Tile(z, lonToX(lon, z), latToY(lat, z), marginM);
        }

        public double centreLat() {
            return (minLat + maxLat) / 2;
        }

        public double centreLon() {
            return (minLon + maxLon) / 2;
        }

        public boolean insetContains(double lat, double lon) {
            return lat > inMinLat && lat < inMaxLat && lon > inMinLon && lon < inMaxLon;
        }

        /** Does the segment a→b touch the inset rectangle (Liang–Barsky, in degrees)? */
        public boolean segmentEntersInset(double lat1, double lon1, double lat2, double lon2) {
            return clip(lat1, lon1, lat2, lon2) != null;
        }

        /**
         * Liang–Barsky clip of a→b against the inset rectangle: {t0, t1} of the inside part, or null.
         * t0 > 0 means the segment enters the inset at a + t0·(b−a).
         */
        public double[] clip(double lat1, double lon1, double lat2, double lon2) {
            double t0 = 0, t1 = 1;
            double dx = lon2 - lon1, dy = lat2 - lat1;
            double[] p = {-dx, dx, -dy, dy};
            double[] q = {lon1 - inMinLon, inMaxLon - lon1, lat1 - inMinLat, inMaxLat - lat1};
            for (int i = 0; i < 4; i++) {
                if (p[i] == 0) {
                    if (q[i] < 0) return null;
                } else {
                    double t = q[i] / p[i];
                    if (p[i] < 0) {
                        if (t > t1) return null;
                        if (t > t0) t0 = t;
                    } else {
                        if (t < t0) return null;
                        if (t < t1) t1 = t;
                    }
                }
            }
            return t0 < t1 ? new double[]{t0, t1} : null;
        }

        /** Which inset border is closest to a point on it: N, E, S or W. */
        public String side(double lat, double lon) {
            double mPerLon = M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(lat));
            double dn = (inMaxLat - lat) * M_PER_DEG_LAT, ds = (lat - inMinLat) * M_PER_DEG_LAT;
            double de = (inMaxLon - lon) * mPerLon, dw = (lon - inMinLon) * mPerLon;
            double m = Math.min(Math.min(Math.abs(dn), Math.abs(ds)), Math.min(Math.abs(de), Math.abs(dw)));
            if (m == Math.abs(dn)) return "N";
            if (m == Math.abs(ds)) return "S";
            if (m == Math.abs(de)) return "E";
            return "W";
        }
    }
}
