package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.List;

public class TunablesTest {

    static class Sample {
        public static double gain = 0.5;
        public static int count = 3;
        public static boolean on = true;
        public static final double CAP = 1;   // final: not tunable
        static double hidden = 2;   // not public: not tunable
        public double notStatic = 4;
        public static String name = "x";   // not a number

        static {
            Tunables.track(Sample.class);
        }
    }

    static class Outer {
        static class Inner {
            public static double kp = 0.06;

            static {
                Tunables.track(Inner.class);
            }
        }
    }

    private static List<Tunables.Change> of(String owner) {
        List<Tunables.Change> all = Tunables.changes();
        all.removeIf(c -> !c.owner.equals(owner));
        return all;
    }

    @Test
    public void nothingToCopyUntilSomethingChanges() {
        Sample.gain = 0.5;
        Sample.count = 3;
        Sample.on = true;
        assertTrue(Tunables.tracks(Sample.class));
        assertTrue(of("Sample").isEmpty());

        Sample.gain = 0.75;   // what Panels does
        List<Tunables.Change> changes = of("Sample");
        assertEquals(1, changes.size());
        Tunables.Change c = changes.get(0);
        assertEquals("TunablesTest.java", c.file);
        assertEquals("gain", c.field);
        assertEquals("0.5", c.code);
        assertEquals("0.75", c.now);
        assertEquals("TUNED TunablesTest.java Sample.gain 0.5 -> 0.75", c.event());

        Sample.gain = 0.5;   // put back: nothing to copy any more
        assertTrue(of("Sample").isEmpty());
    }

    @Test
    public void intsAndBooleansCountFinalAndPrivateOnesDont() {
        Sample.gain = 0.5;
        Sample.count = 4;
        Sample.on = false;
        Sample.hidden = 99;
        Sample.name = "y";
        List<Tunables.Change> changes = of("Sample");
        assertEquals(2, changes.size());
        Sample.count = 3;
        Sample.on = true;
    }

    @Test
    public void aClassInsideAnotherClassNamesTheOuterFile() {
        Outer.Inner.kp = 0.08;
        List<Tunables.Change> changes = of("Inner");
        assertEquals(1, changes.size());
        assertEquals("TunablesTest.java", changes.get(0).file);
        assertEquals("TUNED TunablesTest.java Inner.kp 0.06 -> 0.08", changes.get(0).event());
        Outer.Inner.kp = 0.06;
    }

    @Test
    public void numbersLookLikeYouWouldTypeThem() {
        assertEquals("0.0005", Tunables.show(0.0005));
        assertEquals("0.0025", Tunables.show(0.002 + 0.0005));   // really 0.0025000000000000005
        assertEquals("1.2649", Tunables.show(1.2649));
        assertEquals("0", Tunables.show(0.0));
        assertEquals("-0.35", Tunables.show(-0.35));
        assertEquals("12", Tunables.show(12.0));
        assertEquals("3", Tunables.show(3));
        assertEquals("true", Tunables.show(true));
    }

    @Test
    public void addingAndTakingAwayTheSameAmountIsNotAChange() {
        Sample.gain = 0.5;
        for (int i = 0; i < 7; i++) Sample.gain += 0.0005;
        for (int i = 0; i < 7; i++) Sample.gain -= 0.0005;
        assertFalse(Sample.gain == 0.5 && false);   // it may or may not be exactly 0.5 again, that's the point
        assertTrue(of("Sample").isEmpty());
        Sample.gain = 0.5;
    }

    // the real robot classes: each one has to call Tunables.track BELOW its numbers. if someone adds a number
    // under the static block, it gets remembered as 0 and shows up here as a change nobody made
    @Test
    public void theRobotsTunableClassesAreTrackedAndStartOutMatchingTheCode() throws Exception {
        String[] names = {
                "org.firstinspires.ftc.teamcode.config.UltrasonicConfig",
                "org.firstinspires.ftc.teamcode.config.BallVisionConfig",
                "org.firstinspires.ftc.teamcode.config.DriveConfig",
                "org.firstinspires.ftc.teamcode.mechanisms.WallApproach",
                "org.firstinspires.ftc.teamcode.opmodes.tests.flowerdash.UltrasonicAutoAlignment",
        };
        for (String name : names) {
            Class<?> c = Class.forName(name);   // loading it runs its static block
            assertTrue(name + " never calls Tunables.track", Tunables.tracks(c));
            assertTrue(name + " has no @Configurable, Panels won't show it",
                    c.isAnnotationPresent(com.bylazar.configurables.annotations.Configurable.class));
            for (Tunables.Change change : Tunables.changes()) {
                assertFalse(change.event() + ": is the number declared below the static block?",
                        change.owner.equals(c.getSimpleName()));
            }
        }
        org.firstinspires.ftc.teamcode.config.UltrasonicConfig.leftScale += 0.05;
        try {
            List<Tunables.Change> changes = of("UltrasonicConfig");
            assertEquals(1, changes.size());
            assertEquals("UltrasonicConfig.java", changes.get(0).file);
            assertEquals("leftScale", changes.get(0).field);
        } finally {
            org.firstinspires.ftc.teamcode.config.UltrasonicConfig.leftScale -= 0.05;
        }
        assertTrue(of("UltrasonicConfig").isEmpty());
    }

    @Test
    public void trackNeverThrows() {
        Tunables.track(null);
        Tunables.track(Sample.class);   // twice is fine and keeps the code values
        Sample.gain = 0.5;
        assertTrue(of("Sample").isEmpty());
        assertFalse(Tunables.tracks(String.class));
    }
}
