package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class VisionLocalizer {
    public static final String WEBCAM_NAME = "Webcam 1";
    public static final double APRILTAG_UPDATE_INTERVAL = 0.05; // 50 ms
    public static final double MAX_POSE_JUMP_DISTANCE = 12.0; // inches
    public static final double MAX_HEADING_JUMP = Math.toRadians(45.0);
    public static final double VISION_ALPHA = 0.2; // complementary filter gain for vision
    public static final double TAG_SIZE_METERS = 0.206375;
    public static final double FIELD_LENGTH_METERS = 6.096;
    public static final double TAG_HEIGHT_METERS = 0.20;

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

    public VisionLocalizer(HardwareMap hardwareMap) {
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

    public Pose tryGetVisionPose(Pose referencePose) {
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

    public int getLastDetectionCount() {
        return lastDetectionCount;
    }

    public String getStatus() {
        return status;
    }

    public Pose getLastVisionPose() {
        return lastVisionPose;
    }

    public void close() {
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
