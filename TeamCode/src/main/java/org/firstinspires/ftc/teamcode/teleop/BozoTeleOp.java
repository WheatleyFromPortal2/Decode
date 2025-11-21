package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.VisionLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp(name = "BozoTeleOp", group = "TeleOp")
public class BozoTeleOp extends OpMode {
    private static final double TURN_RATE_MULTIPLIER = 0.75;
    private static final int ADJUST_RPM = 50;
    private static final double INITIAL_LAUNCH_RPM = 2400.0;

    private Robot robot;
    private Follower follower;
    private TelemetryManager telemetryM;
    private VisionLocalizer visionLocalizer;
    public static Pose startingPose;

    private boolean automatedDrive = false;
    private boolean automatedLaunch = false;
    private boolean isIntakePowered = false;
    private boolean isRobotCentric = false;
    private double targetHeading = 0.0;
    private double launchVelocity; // ticks per second

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        visionLocalizer = new VisionLocalizer(hardwareMap);
        launchVelocity = robot.RPMToTPS(INITIAL_LAUNCH_RPM);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        robot.initServos();
    }

    @Override
    public void loop() {
        follower.update();
        Pose odometryPose = follower.getPose();
        Pose visionPose = visionLocalizer.tryGetVisionPose(odometryPose);
        if (visionPose != null) {
            Pose fusedPose = fusePoses(odometryPose, visionPose, VisionLocalizer.VISION_ALPHA);
            follower.setPose(fusedPose);
            odometryPose = fusedPose;
        }

        telemetryM.update();

        if (gamepad1.aWasReleased()) {
            isIntakePowered = !isIntakePowered;
        }
        if (gamepad1.yWasReleased()) {
            automatedLaunch = !automatedLaunch;
        }
        if (gamepad1.rightBumperWasReleased()) {
            try {
                robot.launchBall();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if (gamepad1.startWasReleased()) {
            isRobotCentric = !isRobotCentric;
        }
        if (gamepad1.backWasReleased()) {
            Pose headingPose = follower.getPose();
            headingPose = headingPose.setHeading(Math.toRadians(0));
            follower.setPose(headingPose);
        }
        if (gamepad1.dpadUpWasPressed()) {
            launchVelocity += robot.RPMToTPS(ADJUST_RPM);
        }
        if (gamepad1.dpadDownWasPressed()) {
            launchVelocity -= robot.RPMToTPS(ADJUST_RPM);
        }

        if (!automatedDrive) {
            double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1;
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * TURN_RATE_MULTIPLIER * slowModeMultiplier,
                    isRobotCentric
            );
            if (gamepad1.xWasReleased()) {
                teleOpLaunchPrep();
            }
        } else {
            if (gamepad1.xWasPressed() || !follower.isBusy()) {
                follower.startTeleOpDrive();
                automatedDrive = false;
            }
        }

        robot.intake.setPower(isIntakePowered ? 1 : 0);

        double desiredLaunchTPS;
        if (automatedLaunch) {
            desiredLaunchTPS = launchVelocity;
            robot.launch.setVelocity(desiredLaunchTPS);
        } else {
            double manualLaunchTPS = gamepad1.right_trigger * 2800;
            desiredLaunchTPS = manualLaunchTPS;
            if (manualLaunchTPS == 0) {
                robot.launch.setPower(0);
            } else {
                robot.launch.setVelocity(manualLaunchTPS);
            }
        }

        telemetryM.debug("target heading: " + targetHeading);
        telemetryM.debug("launch within margin?: " + robot.isLaunchWithinMargin(desiredLaunchTPS));
        telemetryM.debug("automated drive?: " + automatedDrive);
        telemetryM.debug("automated launch?: " + automatedLaunch);
        telemetryM.debug("follower busy?: " + follower.isBusy());
        telemetryM.debug("desired launch RPM: " + robot.TPSToRPM(desiredLaunchTPS));
        telemetryM.debug("launch RPM: " + robot.getLaunchRPM());
        telemetryM.debug("launch current: " + robot.getLaunchCurrent());
        telemetryM.debug("intake current: " + robot.getIntakeCurrent());
        telemetryM.debug("lowerTransfer: " + robot.lowerTransfer.getPosition());
        telemetryM.debug("x: " + follower.getPose().getX());
        telemetryM.debug("y: " + follower.getPose().getY());
        telemetryM.debug("heading: " + follower.getPose().getHeading());
        telemetryM.debug("Vision Status", visionLocalizer.getStatus());
        telemetryM.debug("AprilTags Detected", visionLocalizer.getLastDetectionCount());
        Pose lastVisionPose = visionLocalizer.getLastVisionPose();
        if (lastVisionPose != null) {
            telemetryM.debug("Vision X", lastVisionPose.getX());
            telemetryM.debug("Vision Y", lastVisionPose.getY());
            telemetryM.debug("Vision Heading", lastVisionPose.getHeading());
        }
        telemetryM.update(telemetry);
    }

    private Pose fusePoses(Pose odometryPose, Pose visionPose, double alpha) {
        double fusedX = (1 - alpha) * odometryPose.getX() + alpha * visionPose.getX();
        double fusedY = (1 - alpha) * odometryPose.getY() + alpha * visionPose.getY();
        double headingDelta = AngleUnit.normalizeRadians(visionPose.getHeading() - odometryPose.getHeading());
        double fusedHeading = AngleUnit.normalizeRadians(odometryPose.getHeading() + alpha * headingDelta);
        return new Pose(fusedX, fusedY, fusedHeading);
    }

    public void teleOpLaunchPrep() {
        double neededTangentialSpeed = robot.getTangentialSpeed(follower.getPose());
        double neededVelocity = robot.getNeededVelocity(neededTangentialSpeed);
        robot.launch.setVelocity(neededVelocity);

        targetHeading = robot.getGoalHeading(follower.getPose());
        PathChain turnPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), follower.getPose()))
                .setLinearHeadingInterpolation(follower.getHeading(), targetHeading)
                .build();

        follower.followPath(turnPath);
        automatedDrive = true;
        automatedLaunch = true;
    }

    @Override
    public void stop() {
        if (visionLocalizer != null) {
            visionLocalizer.close();
        }
    }
}
