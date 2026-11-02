package com.graphhopper.trailmap.tilerouting.foundation;

import java.util.Arrays;

/**
 * Directed edge keys → dense slots 0..size-1 (open addressing, linear probing). Search state that grows with what a
 * search touches instead of with the graph: a tile tour covers a region of at most ~150 km, while the production graph
 * is all of Europe. Callers keep their per-key values in their own arrays indexed by slot, sized with
 * {@link #capacity()}. {@link #clear()} costs the number of keys held, not the table size. Not thread safe.
 */
public final class SparseKeys {

    private int[] table; // slot + 1, 0 = empty
    private int[] keyOf, posOf;
    private int size, mask, peak;

    public SparseKeys() {
        this(1 << 10);
    }

    public SparseKeys(int initialCapacity) {
        int cap = Integer.highestOneBit(Math.max(16, initialCapacity) - 1) << 1;
        table = new int[2 * cap];
        mask = table.length - 1;
        keyOf = new int[cap];
        posOf = new int[cap];
    }

    private static int mix(int key) {
        int h = key * 0x9E3779B1;
        return h ^ (h >>> 16);
    }

    /** The key's slot, or -1. */
    public int slot(int key) {
        int i = mix(key) & mask;
        while (true) {
            int s = table[i];
            if (s == 0) return -1;
            if (keyOf[s - 1] == key) return s - 1;
            i = (i + 1) & mask;
        }
    }

    public boolean contains(int key) {
        return slot(key) >= 0;
    }

    /** A new slot for a key that has none (check with {@link #slot} first). */
    public int insert(int key) {
        if (size == keyOf.length) grow();
        int i = mix(key) & mask;
        while (table[i] != 0) i = (i + 1) & mask;
        table[i] = size + 1;
        keyOf[size] = key;
        posOf[size] = i;
        if (size + 1 > peak) peak = size + 1;
        return size++;
    }

    /** Set by {@link #slotOrInsert}: whether its key was new. */
    public boolean added;

    /** The key's slot, inserting it if absent (one probe); {@link #added} tells which. */
    public int slotOrInsert(int key) {
        if (size == keyOf.length) grow();
        int i = mix(key) & mask;
        while (true) {
            int s = table[i];
            if (s == 0) break;
            if (keyOf[s - 1] == key) {
                added = false;
                return s - 1;
            }
            i = (i + 1) & mask;
        }
        table[i] = size + 1;
        keyOf[size] = key;
        posOf[size] = i;
        if (size + 1 > peak) peak = size + 1;
        added = true;
        return size++;
    }

    /** Adds the key if absent; true if it was added. */
    public boolean add(int key) {
        if (slot(key) >= 0) return false;
        insert(key);
        return true;
    }

    /** BitSet-style use as a key set: {@link #contains}. */
    public boolean get(int key) {
        return contains(key);
    }

    /** BitSet-style use as a key set: {@link #add}. */
    public void set(int key) {
        add(key);
    }

    /** BitSet-style use as a key set: {@link #size}. */
    public int cardinality() {
        return size;
    }

    public int key(int slot) {
        return keyOf[slot];
    }

    public int size() {
        return size;
    }

    /** Callers' per-slot arrays must be at least this long after an {@link #insert}. */
    public int capacity() {
        return keyOf.length;
    }

    /** Most keys held at once since this instance was made. */
    public int peak() {
        return peak;
    }

    public void clear() {
        for (int s = 0; s < size; s++) table[posOf[s]] = 0;
        size = 0;
    }

    private void grow() {
        int cap = 2 * keyOf.length;
        keyOf = Arrays.copyOf(keyOf, cap);
        posOf = Arrays.copyOf(posOf, cap);
        table = new int[2 * cap];
        mask = table.length - 1;
        for (int s = 0; s < size; s++) {
            int i = mix(keyOf[s]) & mask;
            while (table[i] != 0) i = (i + 1) & mask;
            table[i] = s + 1;
            posOf[s] = i;
        }
    }
}
