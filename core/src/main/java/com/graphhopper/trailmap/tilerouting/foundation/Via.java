package com.graphhopper.trailmap.tilerouting.foundation;

/** One via point of the final route. Heading NaN = free; role = home | portal | centre. */
public final class Via {
    public final double lat, lon, heading;
    public final String role, tileKey;

    public Via(double lat, double lon, double heading, String role, String tileKey) {
        this.lat = lat;
        this.lon = lon;
        this.heading = heading;
        this.role = role;
        this.tileKey = tileKey;
    }

    public static Via of(Portal p) {
        return new Via(p.lat, p.lon, p.heading, "portal", p.tileKey);
    }
}
