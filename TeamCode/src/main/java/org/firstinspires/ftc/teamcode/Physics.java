package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;

public class Physics {
    // both of these are in meters
    private final double SHOOTER_Z = .4064; // calibrated 2-13-26
    private final double GOAL_Z = 0.7874; // calibrated 2-13-26
    public Physics() { // set up class

    }

    public LaunchSetpoints getNeededStaticVelocity(
            Pose robotPose,
            Pose goalPose )
    {
        LaunchSetpoints setpoints = new LaunchSetpoints(0, 0, 0);

        // convert from inches to meters
        double robotX = inchesToMeters(robotPose.getX());
        double robotY = inchesToMeters(robotPose.getY());

        double goalX = inchesToMeters(goalPose.getX());
        double goalY = inchesToMeters(goalPose.getY());

        final double G = 9.81;
        final double ANGLE = Math.PI / 3.0;
        setpoints.setHoodPos(ANGLE);

        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double d = Math.hypot(dx, dy);
        double h = GOAL_Z - SHOOTER_Z;

        double cos = Math.cos(ANGLE);
        double tan = Math.tan(ANGLE);

        double denom = 2 * cos * cos * (d * tan - h);
        if (denom <= 0) return null;

        double v = Math.sqrt((G * d * d) / denom);
        double RPM = velocityToRPM(v);
        setpoints.setRPM(RPM);
        return setpoints;
    }

    public LaunchSetpoints getNeededVelocity(
            Pose robotPose,
            Vector robotVelocity,
            Pose goalPose )
    {
        // TODO: convert this all to meters (including vector if necessary)
        LaunchSetpoints setpoints = new LaunchSetpoints(0, 0, 0);

        // convert from inches to meters
        double robotX = inchesToMeters(robotPose.getX());
        double robotY = inchesToMeters(robotPose.getY());

        double goalX = inchesToMeters(goalPose.getX());
        double goalY = inchesToMeters(goalPose.getY());

        final double G = 9.81;
        final double ANGLE = Math.PI / 3.0;
        setpoints.setHoodPos(ANGLE);
        final double SHOT_DELAY = 0.15;
        double robotVx = robotVelocity.getXComponent();
        double robotVy = robotVelocity.getYComponent();
        double fireX = robotPose.getX() + robotVx * SHOT_DELAY;
        double fireY = robotPose.getY() + robotVy * SHOT_DELAY;

        double dx = goalPose.getX() - fireX;
        double dy = goalPose.getY() - fireY;
        double d = Math.hypot(dx, dy);
        double h = GOAL_Z - SHOOTER_Z;

        double ux = dx / d;
        double uy = dy / d;

        double vAlong = robotVx * ux + robotVy * uy;

        double cos = Math.cos(ANGLE);
        double sin = Math.sin(ANGLE);

        double bestV = -1;
        double bestErr = Double.MAX_VALUE;

        for (double v = 1.0; v <= 20.0; v += 0.02) {
            double vx = v * cos + vAlong;
            if (vx <= 0) continue;

            double t = d / vx;
            if (t <= 0) continue;

            double z = v * sin * t - 0.5 * G * t * t;
            double err = Math.abs(z - h);

            if (err < bestErr) {
                bestErr = err;
                bestV = v;
            }
        }

        if (bestV < 0) return null;

        double RPM = velocityToRPM(bestV);
        setpoints.setRPM(RPM);
        return setpoints;
    }

    private double velocityToRPM(double velocity)
    {
        double RPM = (velocity + 0.475726) /0.00246641;
        return RPM;
    }

    private double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}

