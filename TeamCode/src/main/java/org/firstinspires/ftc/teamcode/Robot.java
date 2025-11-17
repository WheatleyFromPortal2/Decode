package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.pedropathing.geometry.Pose;

public class Robot {
    private static Robot instance;
    public DcMotorEx intake, launch;
    public Servo lowerTransfer, upperTransfer;

    public static final int TICKS_PER_REV = 28;
    public static final double launchRatio = (double) 16 / 20;

    public static final double launchP = 300;
    public static final double launchI = 0.1;
    public static final double launchD = 0.2;
    public static final double launchF = (double) 1 / 2800;
    public static final double lowerTransferLowerLimit = 0.28;
    public static final double lowerTransferUpperLimit = 0.49;

    public static final double upperTransferClosed = 0.36;
    public static final double upperTransferOpen = 0.66;

    public static final int launchDelay = 250;
    public final double scoreRPMMargin = 100;
    public static Pose goalPose;

    private static final double[] calibRPM = {
            1782.8571, 1714.2857, 2331.4285, 3668.5714, 2091.42, 3668.5714
    };

    private static final double[] calibVel = {
            2.636, 3.136, 0.5356, 1.7417, 1.0557, 1.1592
    };

    public Robot(HardwareMap hw) {
        intake = hw.get(DcMotorEx.class, "intake");
        launch = hw.get(DcMotorEx.class, "launch");
        lowerTransfer = hw.get(Servo.class, "lowerTransfer");
        upperTransfer = hw.get(Servo.class, "upperTransfer");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launch.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients pidfNew = new PIDFCoefficients(launchP, launchI, launchD, launchF);
        launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
    }

    public static Robot getInstance(HardwareMap hw) {
        if (instance == null) {
            instance = new Robot(hw);
        }
        return instance;
    }

    public double getDstFromGoal(Pose currentPosition) {
        double xDst = Math.abs(currentPosition.getX() - goalPose.getX());
        double yDst = Math.abs(currentPosition.getY() - goalPose.getY());
        return Math.pow(Math.pow(xDst, 2) + Math.pow(yDst, 2), 0.5);
    }

    public double getGoalHeading(Pose currentPosition) {
        double xDst = goalPose.getX() - currentPosition.getX();
        double yDst = goalPose.getY() - currentPosition.getY();
        return Math.atan2(yDst, xDst);
    }

    public double getTangentialSpeed(Pose currentPosition) {
        double d = getDstFromGoal(currentPosition);
        double numerator = 19.62 * Math.pow(d, 2);
        double denominator = (Math.pow(3, 0.5) * d) - 0.8;
        return Math.pow(numerator / denominator, 0.5);
    }

    private double[] linReg(double[] x, double[] y) {
        int n = x.length;
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }
        double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double intercept = (sumY - slope * sumX) / n;
        return new double[]{slope, intercept};
    }

    public double getNeededVelocity(double tangentialSpeed) {
        double[] params = linReg(calibRPM, calibVel);
        double slope = params[0];
        double intercept = params[1];
        double neededRPM = (tangentialSpeed - intercept) / slope;
        return RPMToTPS(neededRPM);
    }

    public double setAutomatedLaunch(Pose currentPosition) {
        double neededTangentialSpeed = getTangentialSpeed(currentPosition);
        double neededVelocity = getNeededVelocity(neededTangentialSpeed);
        launch.setVelocity(neededVelocity);
        return neededVelocity;
    }

    public double TPSToRPM(double TPS) {
        return (TPS / TICKS_PER_REV) * 60 * launchRatio;
    }

    public double RPMToTPS(double RPM) {
        return (RPM * TICKS_PER_REV / 60) / launchRatio;
    }

    public double getLaunchRPM() {
        return TPSToRPM(launch.getVelocity());
    }

    public double getLaunchRadians() {
        return ((launch.getVelocity() / TICKS_PER_REV) * 2 * Math.PI);
    }

    public double getLaunchCurrent() {
        return launch.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isLaunchWithinMargin(double desiredScoreRPM) {
        return Math.abs(desiredScoreRPM - TPSToRPM(launch.getVelocity())) < scoreRPMMargin;
    }

    public double getIntakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public void initServos() {
        upperTransfer.setPosition(Robot.upperTransferClosed);
        lowerTransfer.setPosition(Robot.lowerTransferLowerLimit);
    }

    public void launchBall() throws InterruptedException {
        upperTransfer.setPosition(upperTransferOpen);
        sleep(launchDelay);
        lowerTransfer.setPosition(lowerTransferUpperLimit);
        sleep(launchDelay);
        upperTransfer.setPosition(upperTransferClosed);
        lowerTransfer.setPosition(lowerTransferLowerLimit);
    }
}
