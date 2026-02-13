package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;

public class Physics {
    private final double SHOOTER_Z = .4064; // calibrated 2-13-26
    private final double GOAL_Z = 0.7874; // calibrated 2-13-26
    public Physics() { // set up class

    }

    public LaunchSetpoints getNeededStaticVelocity(
            Pose robotPose,
            Pose goalPose )
    {
        LaunchSetpoints setpoints = new LaunchSetpoints(0, 0, 0);
        final double G = 9.81;
        final double ANGLE = Math.PI / 3.0;
        setpoints.setHoodPos(ANGLE);

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
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
        LaunchSetpoints setpoints = new LaunchSetpoints(0, 0, 0);

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

    public double velocityToRPM(double velocity)
    {
        double RPM = (velocity + 0.475726) /0.00246641;
        return RPM;
    }
}

