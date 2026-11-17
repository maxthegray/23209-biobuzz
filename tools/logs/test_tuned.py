import glob
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tuned  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

JAVA = """package x;

public class Thing {
    public static double kp = 0.05;
    //    public static double scale = 1.2237;
    public static double scale = 1.2649;
    public static double kP = 0.0040 / zoominess;
    public static boolean on = true;
    static final double settleSec = 1.0;

    public static class Inner {
        public static double settleSec = 0.3;
        public static double kp = 0.06;
    }
}
"""


def change(owner, field, code, now, file="Thing.java"):
    return {"file": file, "owner": owner, "field": field, "code": code, "now": now}


class TunedTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = self.tmp.name
        folder = os.path.join(self.root, "TeamCode/src/main/java/x")
        os.makedirs(folder)
        with open(os.path.join(folder, "Thing.java"), "w") as f:
            f.write(JAVA)

    def tearDown(self):
        self.tmp.cleanup()

    def test_the_last_event_for_a_number_wins_and_put_back_ones_drop_out(self):
        got = tuned.changes_from_events([
            "init Thing",
            "TUNED Thing.java Thing.kp 0.05 -> 0.07",
            "TUNED Thing.java Thing.scale 1.2649 -> 1.3",
            "start",
            "TUNED Thing.java Thing.kp 0.05 -> 0.08",
            "TUNED Thing.java Thing.scale 1.2649 -> 1.2649",
            "stop",
        ])
        self.assertEqual([("kp", "0.08")], [(c["field"], c["now"]) for c in got])

    def test_a_plain_number_that_still_matches_is_ready_to_copy(self):
        got = tuned.resolve(self.root, change("Thing", "kp", "0.05", "0.08"))
        self.assertEqual(("copy", 4, "0.05"), (got["action"], got["line"], got["declared"]))
        self.assertEqual("TeamCode/src/main/java/x/Thing.java", got["path"])

    def test_a_commented_out_line_is_not_the_declaration(self):
        got = tuned.resolve(self.root, change("Thing", "scale", "1.2649", "1.31"))
        self.assertEqual(("copy", 6), (got["action"], got["line"]))

    def test_a_formula_is_pointed_at_but_not_called_a_plain_copy(self):
        got = tuned.resolve(self.root, change("Thing", "kP", "0.002", "0.0025"))
        self.assertEqual(("expr", 7, "0.0040 / zoominess"), (got["action"], got["line"], got["declared"]))

    def test_a_class_inside_a_class_finds_its_own_line_not_the_outer_one(self):
        got = tuned.resolve(self.root, change("Inner", "kp", "0.06", "0.09"))
        self.assertEqual(("copy", 13), (got["action"], got["line"]))
        got = tuned.resolve(self.root, change("Inner", "settleSec", "0.3", "0.5"))
        self.assertEqual(("copy", 12), (got["action"], got["line"]))

    def test_code_that_changed_since_the_run_is_flagged(self):
        got = tuned.resolve(self.root, change("Thing", "kp", "0.04", "0.08"))
        self.assertEqual("stale", got["action"])
        got = tuned.resolve(self.root, change("Thing", "kp", "0.04", "0.05"))
        self.assertEqual("done", got["action"])

    def test_booleans_and_missing_things(self):
        self.assertEqual("copy", tuned.resolve(self.root, change("Thing", "on", "true", "false"))["action"])
        self.assertEqual("lost", tuned.resolve(self.root, change("Thing", "nope", "1", "2"))["action"])
        self.assertEqual("lost", tuned.resolve(self.root, change("Gone", "kp", "1", "2", file="Gone.java"))["action"])
        self.assertEqual("lost", tuned.resolve(self.root, change("NoInner", "kp", "1", "2"))["action"])

    def test_the_log_the_robot_code_writes_points_at_the_real_line(self):
        # written by the Java unit tests (make test runs those first)
        logs = glob.glob(os.path.join(ROOT, "TeamCode/build/sample-logs/*.wpilog"))
        if not logs:
            self.skipTest("no sample log, run the Java tests first")
        got = tuned.tuned(ROOT, logs[0])
        self.assertEqual(1, len(got))
        self.assertEqual("leftScale", got[0]["field"])
        self.assertTrue(got[0]["path"].endswith("config/UltrasonicConfig.java"))
        self.assertIsNotNone(got[0]["line"])


if __name__ == "__main__":
    unittest.main()
