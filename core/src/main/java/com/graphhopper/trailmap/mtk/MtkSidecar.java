/*
 * Trailmap - MTK (Maastotietokanta, National Land Survey of Finland, CC BY 4.0) sidecar.
 *
 * Read-only table of MTK values per OSM way, written by the mtk-enrich match job:
 * mtk_way_attrs_finland_<date>.csv.gz. One row per piece of a way; a way's pieces are contiguous
 * fractions [from_frac, to_frac) of the way's length, in way order. Columns (header row required):
 *
 *   way_id, node_ids_hash, way_len_m, from_frac, to_frac, mtk_class, class_share,
 *   mtk_paved, paved_share, match_share, mtk_snapshot, matcher_version
 *
 * A missing, unreadable or malformed file gives an empty table (no MTK values); it never fails
 * the import. Single malformed rows are skipped; a way whose rows disagree (hash, length) or
 * whose pieces overlap is dropped as a whole.
 */
package com.graphhopper.trailmap.mtk;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.zip.GZIPInputStream;

public final class MtkSidecar {
    private static final Logger LOGGER = LoggerFactory.getLogger(MtkSidecar.class);

    static final String[] HEADER = {"way_id", "node_ids_hash", "way_len_m", "from_frac", "to_frac", "mtk_class",
            "class_share", "mtk_paved", "paved_share", "match_share", "mtk_snapshot", "matcher_version"};

    public static final byte CLASS_NONE = 0, CLASS_AJOTIE = 1, CLASS_AJOPOLKU = 2, CLASS_TALVITIE = 3, CLASS_POLKU = 4;
    public static final byte PAVED_NONE = 0, PAVED_PAVED = 1, PAVED_UNPAVED = 2;

    /** Overlap tolerance between consecutive pieces (fractions carry 4 decimals). */
    private static final double FRAC_EPS = 1e-4;
    private static final int MAX_LOGGED_BAD_ROWS = 10;

    // per way, sorted by way id
    private final long[] wayIds;
    private final long[] hashes;
    private final double[] wayLens;
    private final int[] firstPiece; // length = ways + 1
    // per piece, grouped by way in way-id order, sorted by from_frac within a way
    private final float[] from, to, classShare, pavedShare;
    private final byte[] mtkClass, mtkPaved;

    private final String status;

    private MtkSidecar(long[] wayIds, long[] hashes, double[] wayLens, int[] firstPiece, float[] from, float[] to,
                       byte[] mtkClass, float[] classShare, byte[] mtkPaved, float[] pavedShare, String status) {
        this.wayIds = wayIds;
        this.hashes = hashes;
        this.wayLens = wayLens;
        this.firstPiece = firstPiece;
        this.from = from;
        this.to = to;
        this.mtkClass = mtkClass;
        this.classShare = classShare;
        this.mtkPaved = mtkPaved;
        this.pavedShare = pavedShare;
        this.status = status;
    }

    public static MtkSidecar empty(String status) {
        return new MtkSidecar(new long[0], new long[0], new double[0], new int[]{0}, new float[0], new float[0],
                new byte[0], new float[0], new byte[0], new float[0], status);
    }

    public boolean isEmpty() {
        return wayIds.length == 0;
    }

    public int wayCount() {
        return wayIds.length;
    }

    public int pieceCount() {
        return firstPiece[wayIds.length];
    }

    public String getStatus() {
        return status;
    }

    /** @return the way's index, or -1 if the sidecar has no row for it. */
    public int findWay(long wayId) {
        int i = Arrays.binarySearch(wayIds, wayId);
        return i >= 0 ? i : -1;
    }

    public long getHash(int way) {
        return hashes[way];
    }

    public double getWayLength(int way) {
        return wayLens[way];
    }

    /**
     * @return the index of the way's piece that covers the fraction {@code frac} (from &lt;= frac &lt; to, a piece
     * ending at 1 also covers 1), or -1 if the fraction falls outside every piece (an unmatched stretch).
     */
    public int findPiece(int way, double frac) {
        for (int p = firstPiece[way]; p < firstPiece[way + 1]; p++) {
            if (frac >= from[p] && (frac < to[p] || (to[p] >= 1f && frac <= 1.0)))
                return p;
        }
        return -1;
    }

    public byte getMtkClass(int piece) {
        return mtkClass[piece];
    }

    public float getClassShare(int piece) {
        return classShare[piece];
    }

    public byte getMtkPaved(int piece) {
        return mtkPaved[piece];
    }

    public float getPavedShare(int piece) {
        return pavedShare[piece];
    }

    /**
     * Loads the sidecar. Never throws: any problem with the file as a whole gives an empty table and a warning.
     */
    public static MtkSidecar load(String file) {
        if (file == null || file.isBlank()) {
            LOGGER.warn("MTK sidecar: no file configured (mtk_way_attrs|file=...), no MTK values");
            return empty("no file configured");
        }
        Path path = Paths.get(file);
        if (!Files.isRegularFile(path)) {
            LOGGER.warn("MTK sidecar: {} not found, no MTK values", file);
            return empty("file not found");
        }
        try (InputStream raw = new BufferedInputStream(Files.newInputStream(path), 1 << 16);
             InputStream in = file.endsWith(".gz") ? new GZIPInputStream(raw, 1 << 16) : raw;
             BufferedReader reader = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8), 1 << 16)) {
            MtkSidecar s = parse(reader, file);
            LOGGER.info("MTK sidecar: {}", s.status);
            return s;
        } catch (IOException | RuntimeException e) {
            LOGGER.warn("MTK sidecar: cannot read {} ({}), no MTK values", file, e.toString());
            return empty("unreadable: " + e);
        }
    }

    static MtkSidecar parse(BufferedReader reader, String name) throws IOException {
        String header = reader.readLine();
        if (header == null) {
            LOGGER.warn("MTK sidecar: {} is empty, no MTK values", name);
            return empty("empty file");
        }
        if (header.startsWith("﻿"))
            header = header.substring(1);
        String[] cols = header.trim().split(",", -1);
        if (!Arrays.equals(cols, HEADER)) {
            LOGGER.warn("MTK sidecar: {} has header '{}', expected '{}', no MTK values", name, header, String.join(",", HEADER));
            return empty("unexpected header");
        }

        Rows rows = new Rows();
        long lineNo = 1, badRows = 0;
        String line;
        while ((line = reader.readLine()) != null) {
            lineNo++;
            if (line.isEmpty())
                continue;
            String error = rows.add(line);
            if (error != null) {
                badRows++;
                if (badRows <= MAX_LOGGED_BAD_ROWS)
                    LOGGER.warn("MTK sidecar: {} line {} skipped: {}", name, lineNo, error);
            }
        }
        return rows.build(badRows);
    }

    private static final class Rows {
        int n = 0;
        long[] wayId = new long[1 << 16], hash = new long[1 << 16];
        double[] len = new double[1 << 16];
        float[] from = new float[1 << 16], to = new float[1 << 16], cShare = new float[1 << 16], pShare = new float[1 << 16];
        byte[] cls = new byte[1 << 16], paved = new byte[1 << 16];

        String add(String line) {
            String[] f = line.split(",", -1);
            if (f.length != HEADER.length)
                return f.length + " columns, expected " + HEADER.length;
            long id;
            long h;
            double l, fr, t, cs, ps;
            byte c, p;
            try {
                id = Long.parseLong(f[0].trim());
                String hx = f[1].trim();
                if (hx.length() != 16)
                    return "node_ids_hash '" + hx + "' is not 16 hex chars";
                h = Long.parseUnsignedLong(hx, 16);
                l = Double.parseDouble(f[2].trim());
                fr = Double.parseDouble(f[3].trim());
                t = Double.parseDouble(f[4].trim());
                cs = parseShare(f[6]);
                ps = parseShare(f[8]);
            } catch (NumberFormatException e) {
                return "number format: " + e.getMessage();
            }
            if (id <= 0)
                return "way_id " + id;
            if (!(l > 0) || Double.isInfinite(l))
                return "way_len_m " + l;
            if (!(fr >= 0 && t <= 1 && fr < t))
                return "fractions " + fr + ".." + t;
            if (!(cs >= 0 && cs <= 1 && ps >= 0 && ps <= 1))
                return "share out of 0..1";
            c = parseClass(f[5].trim());
            p = parsePaved(f[7].trim());
            if (c < 0)
                return "mtk_class '" + f[5] + "'";
            if (p < 0)
                return "mtk_paved '" + f[7] + "'";
            if (n == wayId.length)
                grow();
            wayId[n] = id;
            hash[n] = h;
            len[n] = l;
            from[n] = (float) fr;
            to[n] = (float) t;
            cls[n] = c;
            cShare[n] = (float) cs;
            paved[n] = p;
            pShare[n] = (float) ps;
            n++;
            return null;
        }

        private static double parseShare(String s) {
            s = s.trim();
            return s.isEmpty() ? 0 : Double.parseDouble(s);
        }

        private void grow() {
            int m = wayId.length * 2;
            wayId = Arrays.copyOf(wayId, m);
            hash = Arrays.copyOf(hash, m);
            len = Arrays.copyOf(len, m);
            from = Arrays.copyOf(from, m);
            to = Arrays.copyOf(to, m);
            cls = Arrays.copyOf(cls, m);
            cShare = Arrays.copyOf(cShare, m);
            paved = Arrays.copyOf(paved, m);
            pShare = Arrays.copyOf(pShare, m);
        }

        MtkSidecar build(long badRows) {
            // sort row indices by (way id, from_frac)
            Integer[] order = new Integer[n];
            for (int i = 0; i < n; i++) order[i] = i;
            Arrays.sort(order, (a, b) -> {
                int c = Long.compare(wayId[a], wayId[b]);
                return c != 0 ? c : Float.compare(from[a], from[b]);
            });

            long[] wIds = new long[n], wHash = new long[n];
            double[] wLen = new double[n];
            int[] first = new int[n + 1];
            float[] pFrom = new float[n], pTo = new float[n], pCs = new float[n], pPs = new float[n];
            byte[] pCls = new byte[n], pPaved = new byte[n];
            int ways = 0, pieces = 0;
            long droppedWays = 0;
            int i = 0;
            while (i < n) {
                int j = i;
                long id = wayId[order[i]];
                while (j < n && wayId[order[j]] == id) j++;
                // rows i..j-1 belong to one way
                boolean ok = true;
                for (int k = i + 1; k < j && ok; k++) {
                    int a = order[k - 1], b = order[k];
                    if (hash[a] != hash[b] || Math.abs(len[a] - len[b]) > 0.05 || from[b] < to[a] - FRAC_EPS)
                        ok = false;
                }
                if (ok) {
                    wIds[ways] = id;
                    wHash[ways] = hash[order[i]];
                    wLen[ways] = len[order[i]];
                    first[ways] = pieces;
                    for (int k = i; k < j; k++) {
                        int r = order[k];
                        pFrom[pieces] = from[r];
                        pTo[pieces] = to[r];
                        pCls[pieces] = cls[r];
                        pCs[pieces] = cShare[r];
                        pPaved[pieces] = paved[r];
                        pPs[pieces] = pShare[r];
                        pieces++;
                    }
                    ways++;
                } else {
                    droppedWays++;
                }
                i = j;
            }
            first[ways] = pieces;
            String status = String.format("%d ways, %d pieces loaded; %d malformed rows skipped, %d inconsistent ways dropped",
                    ways, pieces, badRows, droppedWays);
            return new MtkSidecar(Arrays.copyOf(wIds, ways), Arrays.copyOf(wHash, ways), Arrays.copyOf(wLen, ways),
                    Arrays.copyOf(first, ways + 1), Arrays.copyOf(pFrom, pieces), Arrays.copyOf(pTo, pieces),
                    Arrays.copyOf(pCls, pieces), Arrays.copyOf(pCs, pieces), Arrays.copyOf(pPaved, pieces),
                    Arrays.copyOf(pPs, pieces), status);
        }
    }

    static byte parseClass(String s) {
        switch (s) {
            case "":
                return CLASS_NONE;
            case "ajotie":
                return CLASS_AJOTIE;
            case "ajopolku":
                return CLASS_AJOPOLKU;
            case "talvitie":
                return CLASS_TALVITIE;
            case "polku":
                return CLASS_POLKU;
            default:
                return -1;
        }
    }

    static byte parsePaved(String s) {
        switch (s) {
            case "":
                return PAVED_NONE;
            case "paved":
                return PAVED_PAVED;
            case "unpaved":
                return PAVED_UNPAVED;
            default:
                return -1;
        }
    }
}
