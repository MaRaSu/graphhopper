/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.carrotsearch.hppc.LongIntHashMap;
import com.carrotsearch.hppc.cursors.LongIntCursor;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * One user's activity history as grid cells: for each cell the user has been in, the number of
 * distinct outings that passed through it. Read from the aggregate file the activity pipeline
 * writes: optional {@code #} comment lines (the first may name {@code zoom}), an optional header
 * line, then rows {@code x,y,rides[,more columns]}.
 */
public final class PersonalCells {

    private static final Pattern ZOOM = Pattern.compile("zoom\\D{0,3}(\\d{1,2})");

    private final PersonalCellGrid grid;
    private final LongIntHashMap ridesByCell;
    private final int maxRides;

    PersonalCells(PersonalCellGrid grid, LongIntHashMap ridesByCell) {
        this.grid = grid;
        this.ridesByCell = ridesByCell;
        int max = 0;
        for (LongIntCursor c : ridesByCell)
            max = Math.max(max, c.value);
        this.maxRides = max;
    }

    public static PersonalCells read(Reader in) throws IOException {
        BufferedReader r = new BufferedReader(in);
        int zoom = PersonalCellGrid.DEFAULT_ZOOM;
        PersonalCellGrid grid = null;
        LongIntHashMap map = new LongIntHashMap();
        String line;
        int lineNo = 0;
        while ((line = r.readLine()) != null) {
            lineNo++;
            line = line.trim();
            if (line.isEmpty()) continue;
            if (line.startsWith("#")) {
                Matcher m = ZOOM.matcher(line);
                if (grid == null && m.find()) zoom = Integer.parseInt(m.group(1));
                continue;
            }
            if (!Character.isDigit(line.charAt(0))) continue; // header line
            if (grid == null) grid = new PersonalCellGrid(zoom);
            String[] f = line.split(",", 4);
            if (f.length < 3)
                throw new IOException("Line " + lineNo + ": expected x,y,rides");
            try {
                long x = Long.parseLong(f[0].trim());
                long y = Long.parseLong(f[1].trim());
                int rides = Math.max(1, Integer.parseInt(f[2].trim()));
                long id = grid.id(x, y);
                map.put(id, Math.max(rides, map.getOrDefault(id, 0)));
            } catch (NumberFormatException e) {
                throw new IOException("Line " + lineNo + ": " + e.getMessage());
            }
        }
        if (grid == null) grid = new PersonalCellGrid(zoom);
        return new PersonalCells(grid, map);
    }

    public PersonalCellGrid getGrid() {
        return grid;
    }

    /** Rides through the cell at this point, 0 if never. */
    public int ridesAt(double lat, double lon) {
        return ridesByCell.getOrDefault(grid.cellId(lat, lon), 0);
    }

    public int size() {
        return ridesByCell.size();
    }

    public int getMaxRides() {
        return maxRides;
    }

    /** Calls {@code consumer(cellId, rides)} for every cell whose x/y lies in the given range. */
    public void forEachIn(long minX, long maxX, long minY, long maxY, LongIntConsumer consumer) {
        for (LongIntCursor c : ridesByCell) {
            long x = grid.xOf(c.key), y = grid.yOf(c.key);
            if (x >= minX && x <= maxX && y >= minY && y <= maxY)
                consumer.accept(c.key, c.value);
        }
    }

    @FunctionalInterface
    public interface LongIntConsumer {
        void accept(long cell, int rides);
    }
}
