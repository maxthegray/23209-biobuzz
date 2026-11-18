package org.firstinspires.ftc.teamcode.logging;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// reads a WPILOG back, only for the tests. written from the format description, not from WpiLogWriter,
// so a mistake in the writer doesn't cancel itself out here
final class WpiLogReader {

    static final class Record {
        final String name, type;
        final long timestampUs;
        final byte[] payload;

        Record(String name, String type, long timestampUs, byte[] payload) {
            this.name = name;
            this.type = type;
            this.timestampUs = timestampUs;
            this.payload = payload;
        }

        double asDouble() {
            return ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN).getDouble();
        }

        long asLong() {
            return ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN).getLong();
        }

        boolean asBoolean() {
            return payload[0] != 0;
        }

        String asString() {
            return new String(payload, StandardCharsets.UTF_8);
        }

        double[] asDoubleArray() {
            ByteBuffer b = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN);
            double[] out = new double[payload.length / 8];
            for (int i = 0; i < out.length; i++) out[i] = b.getDouble();
            return out;
        }
    }

    final String extraHeader;
    final List<Record> records = new ArrayList<>();
    final Map<String, String> types = new HashMap<>();   // channel name -> type

    WpiLogReader(byte[] file) {
        ByteBuffer b = ByteBuffer.wrap(file).order(ByteOrder.LITTLE_ENDIAN);
        byte[] magic = new byte[6];
        b.get(magic);
        if (!"WPILOG".equals(new String(magic, StandardCharsets.US_ASCII))) throw new IllegalStateException("not a WPILOG");
        if (b.getShort() != 0x0100) throw new IllegalStateException("wrong version");
        extraHeader = string(b);
        Map<Integer, String[]> entries = new HashMap<>();
        while (b.hasRemaining()) {
            int lengths = b.get() & 0xff;
            int entry = (int) little(b, (lengths & 3) + 1);
            int size = (int) little(b, ((lengths >> 2) & 3) + 1);
            long ts = little(b, ((lengths >> 4) & 7) + 1);
            byte[] payload = new byte[size];
            b.get(payload);
            if (entry == 0) {
                ByteBuffer c = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN);
                if (c.get() != 0) continue;   // only "start" control records are written
                int id = c.getInt();
                String name = string(c), type = string(c);
                entries.put(id, new String[]{name, type});
                types.put(name, type);
            } else {
                String[] e = entries.get(entry);
                if (e == null) throw new IllegalStateException("data for entry " + entry + " before it was started");
                records.add(new Record(e[0], e[1], ts, payload));
            }
        }
    }

    List<Record> of(String name) {
        List<Record> out = new ArrayList<>();
        for (Record r : records) if (r.name.equals(name)) out.add(r);
        return out;
    }

    private static String string(ByteBuffer b) {
        byte[] s = new byte[b.getInt()];
        b.get(s);
        return new String(s, StandardCharsets.UTF_8);
    }

    private static long little(ByteBuffer b, int count) {
        long v = 0;
        for (int i = 0; i < count; i++) v |= (b.get() & 0xffL) << (8 * i);
        return v;
    }
}
