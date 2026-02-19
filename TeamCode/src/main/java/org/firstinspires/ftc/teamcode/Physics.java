package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;

public class Physics {
    // both of these must be in meters
    private final double SHOOTER_Z = .4064; // calibrated 2-13-26
    private final double GOAL_Z = 0.7874; // calibrated 2-13-26

    public Physics() {} // set up class

    public LaunchSetpoints getNeededVelocityStatic (
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
        setpoints.setHoodRadians(ANGLE);

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

        double desiredAbsoluteHeading = Math.atan2(dy, dx);
        double turretRadians = desiredAbsoluteHeading - robotPose.getHeading();

        setpoints.setTurretPos(turretRadians);

        return setpoints;
    }

    public LaunchSetpoints getNeededVelocityDynamic (
            Pose robotPose,
            Vector robotVector,
            Pose goalPose,
            double SHOT_DELAY)
    {
        LaunchSetpoints setpoints = new LaunchSetpoints(0, 0, 0);

        // convert from inches to meters
        double robotX = inchesToMeters(robotPose.getX());
        double robotY = inchesToMeters(robotPose.getY());

        double goalX = inchesToMeters(goalPose.getX());
        double goalY = inchesToMeters(goalPose.getY());

        // Pedro Pathing vectors are also stored in inches and thus need to be converted
        double robotVx = inchesToMeters(robotVector.getXComponent());
        double robotVy = inchesToMeters(robotVector.getYComponent());

        final double G = 9.81;
        final double ANGLE = Math.PI / 3.0;
        setpoints.setHoodRadians(ANGLE);

        //SHOT DELAY TUNE NOW
        double fireX = robotX + robotVx * SHOT_DELAY;
        double fireY = robotY + robotVy * SHOT_DELAY;

        double dx = goalX - fireX;
        double dy = goalY - fireY;
        double d = Math.hypot(dx, dy);
        double h = GOAL_Z - SHOOTER_Z;

        double theta = Math.atan(dy/dx);
        double phi = 0;
        double num1 = 2 * Math.sin(ANGLE) * d - 0.762;
        double denom1 = d * d * G;
        double sqrt = Math.sqrt(num1/denom1);
        double num = robotVy * Math.cos((Math.PI/2) - theta) + robotVx * Math.cos(theta) - robotVy * Math.sin((Math.PI/2) - theta) - sqrt;
        double denom = robotVx * Math.sin(theta);
        phi = Math.acos(num / denom);

        if (phi >= Math.PI)
        {
            phi = 2 * Math.PI - phi;
        }

        setpoints.setTurretPos(phi);

        double num2 = -(robotVy * Math.sin((Math.PI/2) - theta)) + robotVx * Math.sin(theta);
        double denom2 = Math.sin(phi);
        double bestV = num2/denom2;

        double RPM = velocityToRPM(bestV);
        setpoints.setRPM(RPM);

        return setpoints;
    }

    private double velocityToRPM(double velocity)
    {
        // calibrated 2-13-26
        return (velocity + 0.475726) / 0.00246641 + Tunables.physicsRPMOffset;
    }

    private double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}

