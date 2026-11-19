package org.firstinspires.ftc.teamcode.logging;

import java.nio.charset.StandardCharsets;

// builds WPILOG 1.0 records (the file format AdvantageScope and BuzzScope read) in a byte array in memory.
// every number is little-endian. not thread safe: LogCore holds its lock around every call
final class WpiLogWriter {
    private byte[] buf;
    private int len;
    private int nextEntryId = 1;

    WpiLogWriter(int capacity) {
        buf = new byte[capacity];
    }

    byte[] array() {
        return buf;
    }

    int length() {
        return len;
    }

    // someone else now owns the filled array, keep writing into a fresh one
    void swap(byte[] fresh) {
        buf = fresh;
        len = 0;
    }

    // goes once at the very start of the file
    void header(String extra) {
        byte[] e = extra.getBytes(StandardCharsets.UTF_8);
        ensure(12 + e.length);
        bytes("WPILOG".getBytes(StandardCharsets.US_ASCII), 6);
        little(0x0100, 2);
        little(e.length, 4);
        bytes(e, e.length);
    }

    // declares a channel. every later record for it uses the id this returns
    int startEntry(String name, String type, long timestampUs) {
        int id = nextEntryId++;
        byte[] n = name.getBytes(StandardCharsets.UTF_8);
        byte[] t = type.getBytes(StandardCharsets.UTF_8);
        record(0, 1 + 4 + 4 + n.length + 4 + t.length + 4, timestampUs);
        buf[len++] = 0;   // control record type 0 = start
        little(id, 4);
        little(n.length, 4);
        bytes(n, n.length);
        little(t.length, 4);
        bytes(t, t.length);
        little(0, 4);   // no metadata
        return id;
    }

    void appendDouble(int entry, double value, long timestampUs) {
        record(entry, 8, timestampUs);
        little(Double.doubleToRawLongBits(value), 8);
    }

    void appendInt64(int entry, long value, long timestampUs) {
        record(entry, 8, timestampUs);
        little(value, 8);
    }

    void appendBoolean(int entry, boolean value, long timestampUs) {
        record(entry, 1, timestampUs);
        buf[len++] = (byte) (value ? 1 : 0);
    }

    void appendString(int entry, String value, long timestampUs) {
        byte[] b = value.getBytes(StandardCharsets.UTF_8);
        record(entry, b.length, timestampUs);
        bytes(b, b.length);
    }

    void appendDoubleArray(int entry, double[] values, int count, long timestampUs) {
        record(entry, count * 8, timestampUs);
        for (int i = 0; i < count; i++) little(Double.doubleToRawLongBits(values[i]), 8);
    }

    // bytes written as they are, for struct channels
    void appendRaw(int entry, byte[] value, int count, long timestampUs) {
        record(entry, count, timestampUs);
        bytes(value, count);
    }

    // record header: one byte saying how many bytes the next three numbers take, then entry id, payload size, time.
    // also makes room for the payload, so the callers can write straight into buf
    private void record(int entry, int payloadSize, long timestampUs) {
        int entryLen = byteLength(entry, 4);
        int sizeLen = byteLength(payloadSize, 4);
        int timeLen = byteLength(timestampUs, 8);
        ensure(1 + entryLen + sizeLen + timeLen + payloadSize);
        buf[len++] = (byte) ((entryLen - 1) | ((sizeLen - 1) << 2) | ((timeLen - 1) << 4));
        little(entry, entryLen);
        little(payloadSize, sizeLen);
        little(timestampUs, timeLen);
    }

    private static int byteLength(long value, int maxBytes) {
        int length = 1;
        long shifted = value >>> 8;
        while (shifted != 0 && length < maxBytes) {
            length++;
            shifted >>>= 8;
        }
        return length;
    }

    private void little(long value, int count) {
        for (int i = 0; i < count; i++) buf[len++] = (byte) (value >>> (8 * i));
    }

    private void bytes(byte[] src, int count) {
        System.arraycopy(src, 0, buf, len, count);
        len += count;
    }

    private void ensure(int extra) {
        if (len + extra <= buf.length) return;
        byte[] bigger = new byte[Math.max(buf.length * 2, len + extra)];
        System.arraycopy(buf, 0, bigger, 0, len);
        buf = bigger;
    }
}
