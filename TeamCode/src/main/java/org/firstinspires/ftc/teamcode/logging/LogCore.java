package org.firstinspires.ftc.teamcode.logging;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

// the part of the flight recorder that knows nothing about the robot, so it can be tested on a laptop.
//
// the robot loop only copies numbers into memory here. a background thread does the slow part (writing the file),
// so a slow SD card can never make the loop wait: if the writer falls far behind, samples are skipped and counted.
//
// put(...) remembers a value for this loop. once per loop the caller asks sampleDue() and then calls commit()
// (write everything remembered, all with the same timestamp) or discard() (this loop isn't a sample). that keeps
// the file to at most 100 samples a second however fast the loop runs. event(...) and record(...) skip that and
// write right away with their own timestamp.
final class LogCore {

    interface Clock {
        long nanos();
    }

    // where the bytes end up. on the robot this is the log file
    interface Sink {
        void write(byte[] data, int length) throws IOException;

        void sync() throws IOException;   // make sure it's really on the disk, in case the battery gets pulled

        void close() throws IOException;
    }

    static final long SAMPLE_INTERVAL_NS = 10_000_000L;   // 100 samples a second at most
    static final int CHUNK_BYTES = 32 * 1024;
    static final int HANDOFF_BYTES = 16 * 1024;   // give the writer thread what we have once it's this big...
    static final long HANDOFF_NS = 250_000_000L;   // ...or this old
    static final int PAUSE_BYTES = 256 * 1024;   // writer this far behind = stop taking samples until it catches up
    static final int QUEUE_CHUNKS = 32;
    static final long SYNC_NS = 2_000_000_000L;
    static final long TICK_NS = 250_000_000L;   // how often the background job runs

    private static final int DOUBLE = 0, LONG = 1, BOOLEAN = 2, STRING = 3, ARRAY = 4, RAW = 5;
    private static final String[] TYPE_NAMES = {"double", "int64", "boolean", "string", "double[]"};

    private static final class Channel {
        final String name;
        final int kind;
        final String typeName;
        int entry = -1;   // not in the file yet
        boolean pending;
        double d;
        long l;
        boolean b;
        String s, lastWritten;
        double[] a = new double[0];
        byte[] raw = new byte[0];
        int count;

        Channel(String name, int kind, String typeName) {
            this.name = name;
            this.kind = kind;
            this.typeName = typeName;
        }
    }

    private static final class Chunk {
        final byte[] data;
        final int length;

        Chunk(byte[] data, int length) {
            this.data = data;
            this.length = length;
        }
    }

    private static final Chunk END = new Chunk(null, 0);

    private final Clock clock;
    private final Sink sink;
    private final Runnable backgroundJob;
    private final WpiLogWriter writer = new WpiLogWriter(CHUNK_BYTES);
    private final long startNs;
    private final HashMap<String, Channel> channels = new HashMap<>();
    private final ArrayList<Channel> pending = new ArrayList<>();
    private final ArrayBlockingQueue<Chunk> queue = new ArrayBlockingQueue<>(QUEUE_CHUNKS);
    private final ArrayBlockingQueue<byte[]> spareArrays = new ArrayBlockingQueue<>(8);
    private final CountDownLatch finished = new CountDownLatch(1);
    private final Thread thread;
    private final int eventsEntry;

    private long nextSampleNs = Long.MIN_VALUE;
    private long lastHandoffNs;
    private long lastEventUs = -1;
    private boolean closed;
    private boolean paused;
    private long skippedSamples;
    private volatile boolean failed;

    // backgroundJob (may be null) runs on the writer thread about 4 times a second, for slow reads that shouldn't
    // be in the robot loop
    LogCore(Clock clock, Sink sink, String header, Runnable backgroundJob) {
        this.clock = clock;
        this.sink = sink;
        this.backgroundJob = backgroundJob;
        startNs = clock.nanos();
        lastHandoffNs = startNs;
        writer.header(header);
        eventsEntry = writer.startEntry("events", "string", 0);
        thread = new Thread(this::writeLoop, "FlightRecorder");
        thread.setDaemon(true);
        thread.setPriority(Thread.MIN_PRIORITY);
        thread.start();
    }

    synchronized void put(String name, double value) {
        Channel c = channel(name, DOUBLE, null);
        if (c != null) c.d = value;
    }

    synchronized void put(String name, long value) {
        Channel c = channel(name, LONG, null);
        if (c != null) c.l = value;
    }

    synchronized void put(String name, boolean value) {
        Channel c = channel(name, BOOLEAN, null);
        if (c != null) c.b = value;
    }

    // strings only get written when they change, so a status that stays the same costs nothing
    synchronized void put(String name, String value) {
        Channel c = channel(name, STRING, null);
        if (c != null) c.s = value == null ? "" : value;
    }

    // the array is copied, so the caller can reuse its own
    synchronized void put(String name, double[] values) {
        Channel c = channel(name, ARRAY, null);
        if (c == null) return;
        if (c.a.length < values.length) c.a = new double[values.length];
        System.arraycopy(values, 0, c.a, 0, values.length);
        c.count = values.length;
    }

    // bytes for a struct channel, typeName is the WPILOG type like "struct:Pose2d"
    synchronized void putRaw(String name, String typeName, byte[] value) {
        Channel c = channel(name, RAW, typeName);
        if (c == null) return;
        if (c.raw.length < value.length) c.raw = new byte[value.length];
        System.arraycopy(value, 0, c.raw, 0, value.length);
        c.count = value.length;
    }

    // true when something already put this channel since the last commit/discard
    synchronized boolean pending(String name) {
        Channel c = channels.get(name);
        return c != null && c.pending;
    }

    // the number put in this channel since the last commit/discard, NaN if there isn't one
    synchronized double pendingDouble(String name) {
        Channel c = channels.get(name);
        return c != null && c.pending && c.kind == DOUBLE ? c.d : Double.NaN;
    }

    // true when this loop should be a sample. doesn't change anything, commit() does
    synchronized boolean sampleDue() {
        return !closed && !failed && clock.nanos() >= nextSampleNs;
    }

    // writes everything put(...) since the last commit/discard, all with the same timestamp
    synchronized void commit() {
        if (closed || failed) {
            clearPending();
            return;
        }
        long now = clock.nanos();
        if (nextSampleNs == Long.MIN_VALUE || now >= nextSampleNs) {
            // skip ahead past any samples we were too slow for, instead of trying to catch up
            long behind = nextSampleNs == Long.MIN_VALUE ? 0 : (now - nextSampleNs) / SAMPLE_INTERVAL_NS;
            nextSampleNs = (nextSampleNs == Long.MIN_VALUE ? now : nextSampleNs) + (behind + 1) * SAMPLE_INTERVAL_NS;
        }
        if (paused) {
            skippedSamples++;
        } else {
            long ts = (now - startNs) / 1000;
            for (int i = 0; i < pending.size(); i++) write(pending.get(i), ts);
        }
        clearPending();
        handOff(now, false);
    }

    // forget what was put this loop
    synchronized void discard() {
        clearPending();
        if (!closed && !failed) handOff(clock.nanos(), false);
    }

    // something that happened, with its own exact time. never rate limited
    synchronized void event(String message) {
        // the writer thread is stuck and memory is filling up: better to lose events than to run out of memory
        if (closed || failed || writer.length() > 4 * PAUSE_BYTES) return;
        long now = clock.nanos();
        // viewers keep one string per timestamp, so two events in the same microsecond need different ones
        long ts = Math.max((now - startNs) / 1000, lastEventUs + 1);
        lastEventUs = ts;
        writer.appendString(eventsEntry, message, ts);
        handOff(now, false);
    }

    // values written right away, all with the same timestamp. for things read slowly on the background thread
    // (battery volts + amps, which the tools need at the same instant), don't call it every loop
    synchronized void record(String[] names, double[] values, int count) {
        if (closed || failed || paused) return;
        long now = clock.nanos();
        long ts = (now - startNs) / 1000;
        for (int i = 0; i < count; i++) {
            Channel c = channels.get(names[i]);
            if (c == null) {
                c = new Channel(names[i], DOUBLE, TYPE_NAMES[DOUBLE]);
                channels.put(names[i], c);
            }
            if (c.kind != DOUBLE) continue;
            c.d = values[i];
            write(c, ts);
        }
        handOff(now, false);   // so this still reaches the file while the loop isn't calling update()
    }

    void record(String name, double value) {
        record(new String[]{name}, new double[]{value}, 1);
    }

    // a struct layout, written once at time 0 so viewers can decode the struct channels
    synchronized void schema(String typeName, String layout) {
        if (closed || failed) return;
        int entry = writer.startEntry("/.schema/struct:" + typeName, "structschema", 0);
        byte[] b = layout.getBytes(java.nio.charset.StandardCharsets.UTF_8);
        writer.appendRaw(entry, b, b.length, 0);
    }

    synchronized boolean isOpen() {
        return !closed && !failed;
    }

    synchronized long skippedSamples() {
        return skippedSamples;
    }

    // hands the last data to the writer thread and tells it to finish. doesn't wait for the file to close,
    // use awaitClosed for that
    void close() {
        Chunk last = null;
        synchronized (this) {
            if (closed) return;
            closed = true;
            clearPending();
            if (writer.length() > 0) last = new Chunk(writer.array(), writer.length());
        }
        try {
            if (last != null && !failed) queue.offer(last, 1, TimeUnit.SECONDS);
            if (!queue.offer(END, 1, TimeUnit.SECONDS)) thread.interrupt();
        } catch (InterruptedException e) {
            thread.interrupt();
            Thread.currentThread().interrupt();
        }
    }

    boolean awaitClosed(long ms) throws InterruptedException {
        return finished.await(ms, TimeUnit.MILLISECONDS);
    }

    private Channel channel(String name, int kind, String typeName) {
        if (closed || failed) return null;
        Channel c = channels.get(name);
        if (c == null) {
            c = new Channel(name, kind, typeName != null ? typeName : TYPE_NAMES[kind]);
            channels.put(name, c);
        } else if (c.kind != kind) {
            return null;   // the first put decides a channel's type, anything else is ignored
        }
        if (!c.pending) {
            c.pending = true;
            pending.add(c);
        }
        return c;
    }

    private void clearPending() {
        for (int i = 0; i < pending.size(); i++) pending.get(i).pending = false;
        pending.clear();
    }

    private void write(Channel c, long ts) {
        if (c.kind == STRING && c.s.equals(c.lastWritten)) return;
        if (c.entry < 0) c.entry = writer.startEntry(c.name, c.typeName, ts);
        switch (c.kind) {
            case DOUBLE: writer.appendDouble(c.entry, c.d, ts); break;
            case LONG: writer.appendInt64(c.entry, c.l, ts); break;
            case BOOLEAN: writer.appendBoolean(c.entry, c.b, ts); break;
            case STRING:
                writer.appendString(c.entry, c.s, ts);
                c.lastWritten = c.s;
                break;
            case ARRAY: writer.appendDoubleArray(c.entry, c.a, c.count, ts); break;
            default: writer.appendRaw(c.entry, c.raw, c.count, ts); break;
        }
    }

    // gives the filled array to the writer thread if it's big or old enough. never waits: if the writer thread
    // is behind, keep filling the same array, and past PAUSE_BYTES stop sampling until there's room again
    private void handOff(long now, boolean force) {
        if (writer.length() == 0) return;
        if (!force && writer.length() < HANDOFF_BYTES && now - lastHandoffNs < HANDOFF_NS) return;
        if (queue.offer(new Chunk(writer.array(), writer.length()))) {
            byte[] spare = spareArrays.poll();
            writer.swap(spare != null ? spare : new byte[CHUNK_BYTES]);
            lastHandoffNs = now;
            if (paused) {
                paused = false;
                long ts = Math.max((now - startNs) / 1000, lastEventUs + 1);
                lastEventUs = ts;
                writer.appendString(eventsEntry,
                        "RECORDER SKIPPED " + skippedSamples + " samples so far (storage was slow)", ts);
            }
        } else if (writer.length() > PAUSE_BYTES) {
            paused = true;
        }
    }

    // the background thread: writes chunks to the file, syncs every couple of seconds, runs the background job
    private void writeLoop() {
        long lastSync = clock.nanos(), lastTick = lastSync;
        try {
            while (true) {
                Chunk c = queue.poll(50, TimeUnit.MILLISECONDS);
                if (c == END) break;
                if (c != null) {
                    sink.write(c.data, c.length);
                    if (c.data.length == CHUNK_BYTES) spareArrays.offer(c.data);
                }
                long now = clock.nanos();
                if (now - lastSync >= SYNC_NS) {
                    sink.sync();
                    lastSync = now;
                }
                if (backgroundJob != null && now - lastTick >= TICK_NS) {
                    lastTick = now;
                    try {
                        backgroundJob.run();
                    } catch (Throwable ignored) {
                        // a failed battery read is not a reason to stop logging
                    }
                }
            }
        } catch (Throwable t) {
            // full SD card, file error, or interrupted: the recorder is off for the rest of the run
            failed = true;
        } finally {
            try {
                sink.sync();
            } catch (Throwable ignored) {
            }
            try {
                sink.close();
            } catch (Throwable ignored) {
            }
            queue.clear();
            finished.countDown();
        }
    }
}
