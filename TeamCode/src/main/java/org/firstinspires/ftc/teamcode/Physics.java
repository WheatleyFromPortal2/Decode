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



    public LaunchSetpoints getNeededVelocityDynamic(
            Pose robotPose,
            Vector robotVector,
            Pose goalPose,
            double shotDelay)
    {
        LaunchSetpoints setpoints = new LaunchSetpoints(0, 0, 0);

        shotDelay = Tunables.staticShotDelay;

        // Convert positions and velocities from inches to meters
        double robotX = inchesToMeters(robotPose.getX());
        double robotY = inchesToMeters(robotPose.getY());
        double goalX = inchesToMeters(goalPose.getX());
        double goalY = inchesToMeters(goalPose.getY());

        double robotVx = inchesToMeters(robotVector.getXComponent());
        double robotVy = inchesToMeters(robotVector.getYComponent());

        final double G = 9.81;
        final double ANGLE = Math.PI / 3.0; // 60 degrees
        setpoints.setHoodRadians(ANGLE);

        //Predict robot position at firing time
        double fireX = robotX + robotVx * shotDelay;
        double fireY = robotY + robotVy * shotDelay;

        //Compute horizontal displacement to goal
        double dx = goalX - fireX;
        double dy = goalY - fireY;
        double d = Math.hypot(dx, dy);
        double h = GOAL_Z - SHOOTER_Z;

        //Compute required projectile speed ignoring robot motion
        double cosAngle = Math.cos(ANGLE);
        double denom = 2 * cosAngle * cosAngle * (d * Math.tan(ANGLE) - h);
        if (denom <= 0) {
            // No physical solution possible
            return setpoints;
        }
        double vReq = Math.sqrt(G * d * d / denom);

        //Compute required field-frame velocity vector
        double thetaField = Math.atan2(dy, dx);
        double vReqX = vReq * Math.cos(thetaField);
        double vReqY = vReq * Math.sin(thetaField);

        //Compute required velocity relative to robot
        double vShotX = vReqX - robotVx;
        double vShotY = vReqY - robotVy;

        //Compute turret angle relative to robot
        double phi = Math.atan2(vShotY, vShotX);
        setpoints.setTurretPos(phi - robotPose.getHeading());

        //Compute shooter speed
        double vShot = Math.hypot(vShotX, vShotY);
        double RPM = velocityToRPM(vShot);
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

