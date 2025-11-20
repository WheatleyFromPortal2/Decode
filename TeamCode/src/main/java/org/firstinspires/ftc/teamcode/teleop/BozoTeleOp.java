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
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final double APRILTAG_UPDATE_INTERVAL = 0.05; // 50 ms
    private static final double MAX_POSE_JUMP_DISTANCE = 12.0; // inches
    private static final double MAX_HEADING_JUMP = Math.toRadians(45.0);
    private static final double VISION_ALPHA = 0.2; // complementary filter gain for vision
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
            Pose fusedPose = fusePoses(odometryPose, visionPose, VISION_ALPHA);
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

    private static class VisionLocalizer {
        private static final double TAG_SIZE_METERS = 0.206375;
        private static final double FIELD_LENGTH_METERS = 6.096;
        private static final double TAG_HEIGHT_METERS = 0.20;

        private static final double CAM_X_R = 0.10; // meters forward from robot center
        private static final double CAM_Y_R = 0.00; // meters left from robot center
        private static final double CAM_Z_R = 0.25; // meters above floor
        private static final double CAM_YAW_DEG = 0.0;
        private static final double CAM_PITCH_DEG = -90.0;
        private static final double CAM_ROLL_DEG = 0.0;

        private static final double MAX_TAG_DISAGREEMENT = 6.0; // inches

        private final AprilTagProcessor aprilTagProcessor;
        private final VisionPortal visionPortal;
        private int lastDetectionCount = 0;
        private double lastQueryTime = 0.0;
        private double lastSuccessfulUpdateTime = 0.0;
        private Pose lastVisionPose = null;
        private String status = "Initializing";

        VisionLocalizer(HardwareMap hardwareMap) {
            Position cameraPosition = new Position(
                    DistanceUnit.METER,
                    CAM_X_R,
                    CAM_Y_R,
                    CAM_Z_R,
                    0
            );
            YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
                    AngleUnit.DEGREES,
                    CAM_YAW_DEG,
                    CAM_PITCH_DEG,
                    CAM_ROLL_DEG,
                    0
            );

            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setTagLibrary(buildTagLibrary())
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .setDrawAxes(true)
                    .setDrawTagID(true)
                    .build();
            aprilTagProcessor.setDecimation(2);

            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
            builder.addProcessor(aprilTagProcessor);
            visionPortal = builder.build();
        }

        Pose tryGetVisionPose(Pose referencePose) {
            double now = System.currentTimeMillis() / 1000.0;
            if (now - lastQueryTime < APRILTAG_UPDATE_INTERVAL) {
                return null;
            }
            lastQueryTime = now;

            if (aprilTagProcessor == null) {
                status = "Processor offline";
                return null;
            }

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            if (detections == null) {
                status = "No detections";
                lastDetectionCount = 0;
                return null;
            }

            lastDetectionCount = detections.size();
            List<VisionObservation> observations = new ArrayList<>();
            for (AprilTagDetection detection : detections) {
                VisionObservation observation = createObservation(detection);
                if (observation != null) {
                    observations.add(observation);
                }
            }

            if (observations.isEmpty()) {
                updateIdleStatus(now);
                return null;
            }

            VisionObservation bestObservation = selectObservation(observations, referencePose);
            if (bestObservation == null) {
                updateIdleStatus(now);
                return null;
            }

            Pose comparisonPose = referencePose != null ? referencePose : bestObservation.pose;
            double distanceChange = getDistance(comparisonPose, bestObservation.pose);
            if (distanceChange > MAX_POSE_JUMP_DISTANCE) {
                status = String.format("Rejected jump %.1f in", distanceChange);
                return null;
            }

            double headingChange = Math.abs(AngleUnit.normalizeRadians(
                    bestObservation.pose.getHeading() - comparisonPose.getHeading()
            ));
            if (headingChange > MAX_HEADING_JUMP) {
                status = String.format("Rejected heading %.1f deg", Math.toDegrees(headingChange));
                return null;
            }

            status = "ACTIVE - Localizing";
            lastSuccessfulUpdateTime = now;
            lastVisionPose = bestObservation.pose;
            return bestObservation.pose;
        }

        int getLastDetectionCount() {
            return lastDetectionCount;
        }

        String getStatus() {
            return status;
        }

        Pose getLastVisionPose() {
            return lastVisionPose;
        }

        void close() {
            if (visionPortal != null) {
                visionPortal.close();
            }
        }

        private void updateIdleStatus(double now) {
            double timeSinceUpdate = now - lastSuccessfulUpdateTime;
            if (timeSinceUpdate > 1.0) {
                status = String.format("No tags - %.1fs", timeSinceUpdate);
            } else {
                status = "Searching for tags...";
            }
        }

        private VisionObservation selectObservation(List<VisionObservation> observations, Pose referencePose) {
            if (observations.isEmpty()) {
                return null;
            }
            if (observations.size() == 1) {
                return observations.get(0);
            }

            VisionObservation first = observations.get(0);
            VisionObservation second = observations.get(1);

            double disagreement = getDistance(first.pose, second.pose);
            if (disagreement <= MAX_TAG_DISAGREEMENT) {
                Pose averaged = averagePose(first.pose, second.pose);
                double bestRange = Math.min(first.range, second.range);
                return new VisionObservation(averaged, bestRange);
            }

            if (referencePose != null) {
                double firstDelta = getDistance(referencePose, first.pose);
                double secondDelta = getDistance(referencePose, second.pose);
                return firstDelta <= secondDelta ? first : second;
            }
            return first.range <= second.range ? first : second;
        }

        private Pose averagePose(Pose a, Pose b) {
            double avgX = 0.5 * (a.getX() + b.getX());
            double avgY = 0.5 * (a.getY() + b.getY());
            double headingDelta = AngleUnit.normalizeRadians(b.getHeading() - a.getHeading());
            double avgHeading = AngleUnit.normalizeRadians(a.getHeading() + headingDelta * 0.5);
            return new Pose(avgX, avgY, avgHeading);
        }

        private double getDistance(Pose a, Pose b) {
            double dx = a.getX() - b.getX();
            double dy = a.getY() - b.getY();
            return Math.hypot(dx, dy);
        }

        private VisionObservation createObservation(AprilTagDetection detection) {
            if (detection.metadata == null || detection.robotPose == null) {
                return null;
            }
            if (detection.id != 1 && detection.id != 2) {
                return null;
            }
            double x = convertPositionComponent(detection.robotPose.getPosition(), DistanceUnit.INCH, Axis.X);
            double y = convertPositionComponent(detection.robotPose.getPosition(), DistanceUnit.INCH, Axis.Y);
            double heading = Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            Pose pose = new Pose(x, y, heading);
            double range = detection.ftcPose != null ? detection.ftcPose.range : Double.MAX_VALUE;
            return new VisionObservation(pose, range);
        }

        private AprilTagLibrary buildTagLibrary() {
            AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
            addTag(builder, 1, "CornerLeft", 0.0, 0.0);
            addTag(builder, 2, "CornerRight", FIELD_LENGTH_METERS, 0.0);
            return builder.build();
        }

        private void addTag(AprilTagLibrary.Builder builder, int id, String name, double xMeters, double yMeters) {
            VectorF position = new VectorF((float) xMeters, (float) yMeters, (float) TAG_HEIGHT_METERS);
            Quaternion orientation = new Quaternion(1, 0, 0, 0, 0);
            builder.addTag(id, name, TAG_SIZE_METERS, position, DistanceUnit.METER, orientation);
        }

        private double convertPositionComponent(Position position, DistanceUnit desiredUnit, Axis axis) {
            double raw;
            switch (axis) {
                case X:
                    raw = position.x;
                    break;
                case Y:
                    raw = position.y;
                    break;
                case Z:
                    raw = position.z;
                    break;
                default:
                    raw = 0;
            }
            return desiredUnit.fromUnit(position.unit, raw);
        }

        private enum Axis {
            X, Y, Z
        }

        private static class VisionObservation {
            final Pose pose;
            final double range;

            VisionObservation(Pose pose, double range) {
                this.pose = pose;
                this.range = range;
            }
        }
    }
}
