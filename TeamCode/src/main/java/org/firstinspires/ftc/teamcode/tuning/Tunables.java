package org.firstinspires.ftc.teamcode.tuning;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.math.BigDecimal;
import java.math.MathContext;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

// remembers what the tunable numbers are in the code, so after you change some live in Panels
// (http://192.168.43.1:8001) the robot can tell you exactly what to copy back into the code.
// Panels changes are NOT saved anywhere: they're gone after a restart or a hot reload, the code is the only
// place tuning is kept.
//
// how to make numbers tunable:
//
//     @Configurable                               // com.bylazar.configurables.annotations.Configurable
//     public class MyThing {
//         public static double kp = 0.05;         // public static, NOT final, one per line
//         public static double tolerance = 0.3;
//         static final double maxPower = 0.35;    // safety limits stay final so a typo in Panels can't change them
//
//         static { Tunables.track(MyThing.class); }   // must be BELOW the numbers
//     }
//
// the flight recorder does the rest: while a number is different from the code it shows "COPY INTO CODE" on the
// driver hub and saves a TUNED event in the log. `make tuned` on the laptop lists them with the file and line.
//
// nothing in here can throw: it gets called while other classes are loading, and an error there would break
// every opmode that uses them
public final class Tunables {

    // one number that is different from the code right now
    public static final class Change {
        public final String file;    // UltrasonicConfig.java
        public final String owner;   // UltrasonicConfig, or WallApproach for a class inside another class
        public final String field;   // leftScale
        public final String code;    // what the code says
        public final String now;     // what it is right now

        Change(String file, String owner, String field, String code, String now) {
            this.file = file;
            this.owner = owner;
            this.field = field;
            this.code = code;
            this.now = now;
        }

        // what goes in the log's event list. tools/logs/tuned.py reads this, keep them in step
        public String event() {
            return "TUNED " + file + " " + owner + "." + field + " " + code + " -> " + now;
        }
    }

    private static final class Tracked {
        final Field field;
        final String file, owner, code;

        Tracked(Field field, String file, String owner, String code) {
            this.field = field;
            this.file = file;
            this.owner = owner;
            this.code = code;
        }
    }

    private static final List<Tracked> tracked = new ArrayList<>();

    private Tunables() {
    }

    // remembers the current value of every public static non-final number (and boolean) in the class as "what the
    // code says". call it from a static block at the bottom of the class, so it runs when the class loads, which
    // is always before Panels can change anything
    public static void track(Class<?> c) {
        try {
            synchronized (tracked) {
                // after a hot reload the same class comes around again as a new copy, forget the old one
                for (Iterator<Tracked> it = tracked.iterator(); it.hasNext(); ) {
                    if (it.next().field.getDeclaringClass().getName().equals(c.getName())) it.remove();
                }
                Class<?> outer = c;
                while (outer.getEnclosingClass() != null) outer = outer.getEnclosingClass();
                String file = outer.getSimpleName() + ".java";
                for (Field f : c.getDeclaredFields()) {
                    int m = f.getModifiers();
                    if (!Modifier.isStatic(m) || !Modifier.isPublic(m) || Modifier.isFinal(m)) continue;
                    Class<?> t = f.getType();
                    if (t != double.class && t != float.class && t != int.class && t != long.class && t != boolean.class) continue;
                    tracked.add(new Tracked(f, file, c.getSimpleName(), show(f.get(null))));
                }
            }
        } catch (Throwable ignored) {
            // worst case this class's numbers just don't get reported
        }
    }

    // every tracked number that is different from the code right now. empty when nothing was changed
    public static List<Change> changes() {
        List<Change> out = new ArrayList<>();
        try {
            synchronized (tracked) {
                for (Tracked t : tracked) {
                    String now = show(t.field.get(null));
                    if (!now.equals(t.code)) out.add(new Change(t.file, t.owner, t.field.getName(), t.code, now));
                }
            }
        } catch (Throwable ignored) {
        }
        return out;
    }

    // true if track() has been called for this class. for the tests
    public static boolean tracks(Class<?> c) {
        synchronized (tracked) {
            for (Tracked t : tracked) if (t.field.getDeclaringClass() == c) return true;
        }
        return false;
    }

    // numbers the way you'd type them in the code: 0.0005 not 5.0E-4, and 0.0025 not 0.0025000000000000005
    static String show(Object value) {
        if (value instanceof Double || value instanceof Float) {
            double d = ((Number) value).doubleValue();
            if (Double.isNaN(d) || Double.isInfinite(d)) return String.valueOf(d);
            if (d == 0) return "0";
            return new BigDecimal(d).round(new MathContext(8)).stripTrailingZeros().toPlainString();
        }
        return String.valueOf(value);
    }
}
