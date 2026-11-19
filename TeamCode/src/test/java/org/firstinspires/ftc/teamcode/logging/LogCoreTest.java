package org.firstinspires.ftc.teamcode.logging;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicLong;

public class LogCoreTest {

    // a clock the test moves by hand
    private final AtomicLong nanos = new AtomicLong(1_000_000_000L);
    private final LogCore.Clock clock = nanos::get;

    private static final class MemorySink implements LogCore.Sink {
        final ByteArrayOutputStream out = new ByteArrayOutputStream();
        volatile boolean closed;
        int syncs;

        @Override
        public synchronized void write(byte[] data, int length) {
            out.write(data, 0, length);
        }

        @Override
        public synchronized void sync() {
            syncs++;
        }

        @Override
        public void close() {
            closed = true;
        }

        synchronized byte[] bytes() {
            return out.toByteArray();
        }
    }

    private WpiLogReader finish(LogCore core, MemorySink sink) throws Exception {
        core.close();
        assertTrue("writer thread didn't finish", core.awaitClosed(5000));
        assertTrue(sink.closed);
        return new WpiLogReader(sink.bytes());
    }

    private void ms(long count) {
        nanos.addAndGet(count * 1_000_000L);
    }

    @Test
    public void everyTypeComesBackTheSame() throws Exception {
        MemorySink sink = new MemorySink();
        LogCore core = new LogCore(clock, sink, "test-header", null);
        ms(5);
        core.put("a/double", 12.5);
        core.put("a/long", 123456789012345L);
        core.put("a/boolean", true);
        core.put("a/string", "hello");
        core.put("a/array", new double[]{1, -2, 3.5});
        core.putRaw("a/raw", "struct:Thing", new byte[]{9, 8, 7});
        assertTrue(core.sampleDue());
        core.commit();

        WpiLogReader log = finish(core, sink);
        assertEquals("test-header", log.extraHeader);
        assertEquals(12.5, log.of("a/double").get(0).asDouble(), 0);
        assertEquals(123456789012345L, log.of("a/long").get(0).asLong());
        assertTrue(log.of("a/boolean").get(0).asBoolean());
        assertEquals("hello", log.of("a/string").get(0).asString());
        assertArrayEquals(new double[]{1, -2, 3.5}, log.of("a/array").get(0).asDoubleArray(), 0);
        assertArrayEquals(new byte[]{9, 8, 7}, log.of("a/raw").get(0).payload);
        assertEquals("double", log.types.get("a/double"));
        assertEquals("int64", log.types.get("a/long"));
        assertEquals("boolean", log.types.get("a/boolean"));
        assertEquals("string", log.types.get("a/string"));
        assertEquals("double[]", log.types.get("a/array"));
        assertEquals("struct:Thing", log.types.get("a/raw"));
        // everything in one commit shares one timestamp: 5 ms after the recorder started
        for (WpiLogReader.Record r : log.records) assertEquals(5000, r.timestampUs);
    }

    // the recorder uses this to skip its own ultrasonic reading when the opmode already logged one this loop
    @Test
    public void pendingSaysWhatWasPutThisLoop() throws Exception {
        MemorySink sink = new MemorySink();
        LogCore core = new LogCore(clock, sink, "", null);
        assertFalse(core.pending("u/leftIn"));
        core.put("u/leftIn", 12.5);
        assertTrue(core.pending("u/leftIn"));
        assertEquals(12.5, core.pendingDouble("u/leftIn"), 0);
        assertTrue(Double.isNaN(core.pendingDouble("u/other")));
        core.commit();
        assertFalse(core.pending("u/leftIn"));
        assertTrue(Double.isNaN(core.pendingDouble("u/leftIn")));
        core.put("u/leftIn", 3);
        core.discard();
        assertFalse(core.pending("u/leftIn"));
        finish(core, sink);
    }

    @Test
    public void aFastLoopStillOnlySaves100SamplesASecond() throws Exception {
        MemorySink sink = new MemorySink();
        LogCore core = new LogCore(clock, sink, "", null);
        // a 2000 Hz loop for one second
        for (int i = 0; i < 2000; i++) {
            nanos.addAndGet(500_000);
            core.put("x", i);
            if (core.sampleDue()) core.commit();
            else core.discard();
        }
        int samples = finish(core, sink).of("x").size();
        assertTrue("got " + samples, samples >= 99 && samples <= 101);
    }

    @Test
    public void aSlowLoopSavesEveryLoop() throws Exception {
        MemorySink sink = new MemorySink();
        LogCore core = new LogCore(clock, sink, "", null);
        for (int i = 0; i < 20; i++) {
            ms(33);   // a camera loop, about 30 Hz
            core.put("x", i);
            assertTrue(core.sampleDue());
            core.commit();
        }
        assertEquals(20, finish(core, sink).of("x").size());
    }

    @Test
    public void stringsAreOnlyWrittenWhenTheyChange() throws Exception {
        MemorySink sink = new MemorySink();
        LogCore core = new LogCore(clock, sink, "", null);
        String[] values = {"idle", "idle", "driving", "driving", "driving", "idle"};
        for (String v : values) {
            ms(20);
            core.put("status", v);
            core.commit();
        }
        List<WpiLogReader.Record> got = finish(core, sink).of("status");
        assertEquals(3, got.size());
        assertEquals("idle", got.get(0).asString());
        assertEquals("driving", got.get(1).asString());
        assertEquals("idle", got.get(2).asString());
    }

    @Test
    public void theFirstPutDecidesTheTypeAndTheLastPutInALoopWins() throws Exception {
        MemorySink sink = new MemorySink();
        LogCore core = new LogCore(clock, sink, "", null);
        ms(20);
        core.put("x", 1.0);
        core.put("x", 2.0);
        core.commit();
        ms(20);
        core.put("x", "oops");   // wrong type, ignored
        core.commit();
        List<WpiLogReader.Record> got = finish(core, sink).of("x");
        assertEquals(1, got.size());
        assertEquals(2.0, got.get(0).asDouble(), 0);
    }

    @Test
    public void eventsKeepTheirOrderAndNeverShareATimestamp() throws Exception {
        MemorySink sink = new MemorySink();
        LogCore core = new LogCore(clock, sink, "", null);
        core.event("init Test");
        core.event("start");   // same microsecond on the clock
        ms(100);
        core.event("stop");
        List<WpiLogReader.Record> got = finish(core, sink).of("events");
        assertEquals(3, got.size());
        assertEquals("init Test", got.get(0).asString());
        assertEquals("start", got.get(1).asString());
        assertEquals("stop", got.get(2).asString());
        assertTrue(got.get(1).timestampUs > got.get(0).timestampUs);
        assertEquals(100_000, got.get(2).timestampUs);
    }

    @Test
    public void aStuckSdCardNeverMakesTheLoopWait() throws Exception {
        final CountDownLatch release = new CountDownLatch(1);
        final MemorySink memory = new MemorySink();
        LogCore.Sink stuck = new LogCore.Sink() {
            @Override
            public void write(byte[] data, int length) throws IOException {
                try {
                    release.await();
                } catch (InterruptedException e) {
                    throw new IOException(e);
                }
                memory.write(data, length);
            }

            @Override
            public void sync() {
            }

            @Override
            public void close() {
                memory.close();
            }
        };
        LogCore core = new LogCore(clock, stuck, "", null);
        double[] big = new double[64];
        long started = System.nanoTime();
        // enough data to fill every buffer many times over while the file can't be written at all
        for (int i = 0; i < 20_000; i++) {
            ms(10);
            core.put("big", big);
            core.put("i", (double) i);
            core.commit();
        }
        long tookMs = (System.nanoTime() - started) / 1_000_000;
        assertTrue("the loop was held up: " + tookMs + " ms", tookMs < 3000);
        assertTrue("nothing was skipped?", core.skippedSamples() > 0);

        // the card comes back: logging carries on and the file is still readable
        release.countDown();
        long deadline = System.currentTimeMillis() + 5000;
        double last = -1;
        while (System.currentTimeMillis() < deadline) {
            ms(10);
            core.put("i", ++last + 1_000_000);
            core.commit();
            Thread.sleep(2);
            if (!core.isOpen()) break;
            if (last > 200) break;
        }
        WpiLogReader log = finish(core, memory);
        List<WpiLogReader.Record> i = log.of("i");
        assertTrue(i.size() > 0);
        assertTrue("no samples after the card came back", i.get(i.size() - 1).asDouble() >= 1_000_000);
        boolean said = false;
        for (WpiLogReader.Record r : log.of("events")) if (r.asString().startsWith("RECORDER SKIPPED")) said = true;
        assertTrue("the log should say samples were skipped", said);
    }

    @Test
    public void aFullSdCardTurnsTheRecorderOffQuietly() throws Exception {
        LogCore.Sink broken = new LogCore.Sink() {
            @Override
            public void write(byte[] data, int length) throws IOException {
                throw new IOException("No space left on device");
            }

            @Override
            public void sync() {
            }

            @Override
            public void close() {
            }
        };
        LogCore core = new LogCore(clock, broken, "", null);
        for (int i = 0; i < 2000; i++) {
            ms(10);
            core.put("x", i);
            core.put("big", new double[64]);
            core.commit();   // must never throw
            core.event("e" + i);
        }
        assertTrue(core.awaitClosed(5000));
        assertFalse(core.isOpen());
        core.close();
    }

    @Test
    public void theBackgroundJobRunsAndCanRecord() throws Exception {
        MemorySink sink = new MemorySink();
        final LogCore[] holder = new LogCore[1];
        final AtomicLong runs = new AtomicLong();
        LogCore core = new LogCore(clock, sink, "", () -> {
            runs.incrementAndGet();
            holder[0].record("battery", 12.6);
        });
        holder[0] = core;
        long deadline = System.currentTimeMillis() + 5000;
        while (runs.get() < 3 && System.currentTimeMillis() < deadline) {
            ms(300);
            Thread.sleep(60);
        }
        assertTrue(runs.get() >= 3);
        List<WpiLogReader.Record> got = finish(core, sink).of("battery");
        assertTrue(got.size() >= 3);
        assertEquals(12.6, got.get(0).asDouble(), 0);
        assertTrue("synced " + sink.syncs, sink.syncs >= 1);
    }

    @Test
    public void nothingIsWrittenAfterClose() throws Exception {
        MemorySink sink = new MemorySink();
        LogCore core = new LogCore(clock, sink, "", null);
        ms(20);
        core.put("x", 1.0);
        core.commit();
        WpiLogReader log = finish(core, sink);
        core.put("x", 2.0);
        core.commit();
        core.event("late");
        core.record("battery", 1);
        core.close();
        assertFalse(core.sampleDue());
        assertEquals(1, log.of("x").size());
        assertEquals(sink.bytes().length, new MemorySink().bytes().length + sink.bytes().length);
    }

    @Test
    public void pruneKeepsTheNewestLogs() throws Exception {
        File dir = Files.createTempDirectory("logs").toFile();
        for (int i = 0; i < 35; i++) {
            File f = new File(dir, "Run-" + i + ".wpilog");
            new FileOutputStream(f).close();
            assertTrue(f.setLastModified(1_700_000_000_000L + i * 60_000L));
        }
        File other = new File(dir, "notes.txt");
        new FileOutputStream(other).close();
        FlightRecorder.prune(dir, 29);
        assertEquals(29, dir.listFiles((d, n) -> n.endsWith(".wpilog")).length);
        assertFalse(new File(dir, "Run-5.wpilog").exists());
        assertTrue(new File(dir, "Run-6.wpilog").exists());
        assertTrue(new File(dir, "Run-34.wpilog").exists());
        assertTrue(other.exists());
    }

    // writes a small but realistic log for the Python tools' tests (tools/logs, tools/buzzscope) to read
    @Test
    public void writesTheSampleLogForTheTools() throws Exception {
        File dir = new File(System.getProperty("user.dir"), "build/sample-logs");
        dir.mkdirs();
        File file = new File(dir, "SampleTeleOp-20261116-180000-000-1.wpilog");
        final FileOutputStream out = new FileOutputStream(file);
        LogCore core = new LogCore(clock, new LogCore.Sink() {
            @Override
            public void write(byte[] data, int length) throws IOException {
                out.write(data, 0, length);
            }

            @Override
            public void sync() {
            }

            @Override
            public void close() throws IOException {
                out.close();
            }
        }, "23209-biobuzz", null);
        core.schema("Translation2d", "double x;double y;");
        core.schema("Rotation2d", "double value;");
        core.schema("Pose2d", "Translation2d translation;Rotation2d rotation;");
        core.event("init SampleTeleOp");
        ms(500);
        core.event("start");
        // 4 seconds driving forward at 20 in/s, 100 samples a second
        for (int i = 0; i < 400; i++) {
            ms(10);
            double t = i / 100.0;
            core.put("loop/totalNanos", 10_000_000L);
            core.put("loop/windowMaxTotalNanos", 10_000_000L);
            core.put("gamepad1/axes", new double[]{0, 0.5, 0, 0, 0, 0});
            core.put("gamepad1/buttons", i < 100 ? 1L : 0L);
            core.put("Drive/motors/power", new double[]{0.5, 0.5, 0.5, 0.5});
            core.put("Drive/motors/velocityTps", new double[]{900, 900, 900, 900});
            core.put("pose", new double[]{20 + 20 * t, 30, 0});
            core.put("velocity", new double[]{20, 0, 0});
            core.put("Ultrasonic/leftIn", 24 - 4 * t);
            core.put("status", t < 2 ? "driving" : "settling");
            core.commit();
            if (i % 25 == 0) {
                core.record(new String[]{"hubs/control/currentAmps", "hubs/control/volts", "battery"},
                        new double[]{6.0, 12.8 - 0.1 * t, 12.8 - 0.1 * t}, 3);
            }
            if (i == 200) core.event("TUNED UltrasonicConfig.java UltrasonicConfig.leftScale 1.2649 -> 1.31");
        }
        core.event("stop");
        core.close();
        assertTrue(core.awaitClosed(5000));
        WpiLogReader log = new WpiLogReader(Files.readAllBytes(file.toPath()));
        assertEquals(400, log.of("pose").size());
        assertEquals(2, log.of("status").size());
        // the battery tools pair volts with amps by timestamp
        assertEquals(log.of("hubs/control/volts").get(3).timestampUs, log.of("hubs/control/currentAmps").get(3).timestampUs);
    }
}
