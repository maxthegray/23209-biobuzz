package org.firstinspires.ftc.teamcode.auto.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// drives to a spot with plain odometry feedback. the closer it gets the slower it goes, and it never uses full power
// pedro paths go full power and brake at the end, which stutters on short moves and overshoots into walls
public class PointDrive {

    static final double kpDrive = 0.11, kdDrive = 0.018;   // power per inch, power per in/s (the second one is the brake)
    static final double kpStrafe = 0.14, kdStrafe = 0.021;
    static final double kpTurn = 0.02, kdTurn = 0.0026;   // power per degree, power per deg/s
    static final double maxDrive = 0.65, maxTurn = 0.5;
    static final double minPower = 0.06, minTurn = 0.05;
    static final double pushRate = 0.6;   // power per second added while a direction is stuck
    static final double maxPush = 0.25;

    public static final double closeIn = 0.5, closeDeg = 1.5;
    public static final double roughIn = 1.5, roughDeg = 4;   // for spots it's only passing through

    private final ElapsedTime sinceUpdate = new ElapsedTime();
    private double drivePush, strafePush, turnPush;
    private double blockedTime;

    public void start() {
        sinceUpdate.reset();
        drivePush = strafePush = turnPush = 0;
        blockedTime = 0;
    }

    // pushing as hard as it's allowed and still not moving = it's against a wall, stop trying
    public boolean blocked() {
        return blockedTime > 0.3;
    }

    public boolean update(Follower follower, Pose target) {
        return update(follower, target, closeIn, closeDeg, true);
    }

    // close is good enough and it doesn't wait to stop, the next move takes it from there
    public boolean updateRough(Follower follower, Pose target) {
        return update(follower, target, roughIn, roughDeg, false);
    }

    // sets pedro's manual powers, caller still calls follower.update(). true = it's there (and stopped, if mustStop)
    public boolean update(Follower follower, Pose target, double tolIn, double tolDeg, boolean mustStop) {
        double dt = Math.min(sinceUpdate.seconds(), 0.1);
        sinceUpdate.reset();

        Pose p = follower.pose();
        Velocity v = follower.velocity();
        double h = p.heading(), dx = target.x() - p.x(), dy = target.y() - p.y();
        double forward = dx * Math.cos(h) + dy * Math.sin(h);
        double left = -dx * Math.sin(h) + dy * Math.cos(h);
        double turn = Math.toDegrees(Math.atan2(Math.sin(target.heading() - h), Math.cos(target.heading() - h)));
        double forwardSpeed = v.vx * Math.cos(h) + v.vy * Math.sin(h);
        double leftSpeed = -v.vx * Math.sin(h) + v.vy * Math.cos(h);
        double turnSpeed = Math.toDegrees(v.omega);

        boolean there = Math.hypot(forward, left) < tolIn && Math.abs(turn) < tolDeg;
        boolean stopped = Math.hypot(forwardSpeed, leftSpeed) < 2 && Math.abs(turnSpeed) < 10;
        if (there && (stopped || !mustStop)) {
            follower.manual(0, 0, 0);
            return true;
        }

        // each direction only has to get inside its share of the tolerance
        double inTol = tolIn * 0.6;
        drivePush = push(drivePush, forward, inTol, Math.abs(forwardSpeed) > 1.5, dt);
        strafePush = push(strafePush, left, inTol, Math.abs(leftSpeed) > 1.5, dt);
        turnPush = push(turnPush, turn, tolDeg * 0.6, Math.abs(turnSpeed) > 4, dt);

        boolean maxed = drivePush >= maxPush || strafePush >= maxPush || turnPush >= maxPush;
        boolean still = Math.hypot(forwardSpeed, leftSpeed) < 1.5 && Math.abs(turnSpeed) < 4;
        blockedTime = maxed && still ? blockedTime + dt : 0;
        if (blocked()) {
            follower.manual(0, 0, 0);
            return false;
        }

        double drivePower = power(forward, forwardSpeed, kpDrive, kdDrive, maxDrive, inTol, 1.5, minPower + drivePush);
        double strafePower = power(left, leftSpeed, kpStrafe, kdStrafe, maxDrive, inTol, 1.5, minPower + strafePush);
        double turnPower = power(turn, turnSpeed, kpTurn, kdTurn, maxTurn, tolDeg * 0.6, 4, minTurn + turnPush);
        follower.manual(drivePower, strafePower, turnPower);   // forward, left, ccw (robot-centric)
        return false;
    }

    // pull toward the target minus a brake on speed. if it's barely moving outside the tolerance it gets at least min power
    private static double power(double error, double speed, double kp, double kd, double max, double tolerance, double stuckSpeed, double min) {
        double power = Range.clip(kp * error - kd * speed, -max, max);
        if (Math.abs(error) > tolerance && Math.abs(speed) < 3 * stuckSpeed) power = Math.copySign(Math.max(Math.abs(power), min), error);
        return power;
    }

    // grows while that direction is stuck, stays once it's moving, back to 0 once it's there
    private static double push(double push, double error, double tolerance, boolean moving, double dt) {
        if (Math.abs(error) <= tolerance) return 0;
        if (!moving) push += pushRate * dt;
        return Math.min(push, maxPush);
    }
}
