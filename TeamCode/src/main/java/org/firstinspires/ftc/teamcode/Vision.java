package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    private static String status = "initializing...";

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private int lastDetectionCount = 0;
    private double lastQueryTime = 0.0;
    private double lastSuccessfulUpdateTime = 0.0;
    private Pose lastVisionPose = null;

    public Vision(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary()) // we're going to try using the default tag library first
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS) // use our units
                .setCameraPose(Tunables.cameraPosition, Tunables.cameraOrientation)
                // debug settings
                .setDrawAxes(Tunables.drawAxis)
                .setDrawCubeProjection(Tunables.drawCubeProjections)
                .setDrawTagOutline(Tunables.drawTagOutline)
                .setDrawTagID(Tunables.drawTagID)
                .build();
        aprilTagProcessor.setDecimation(Tunables.decimation);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(Robot.camera);
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    public Pose tryGetVisionPose(Pose referencePose) {
        long now = System.currentTimeMillis();
        if (now - lastQueryTime < Tunables.aprilTagUpdateInterval) {
            return null;
        }
        lastQueryTime = now;

        if (aprilTagProcessor == null) {
            status = "processor offline";
            return null;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null) {
            status = "no detections";
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
        if (distanceChange > Tunables.maxPoseJumpDistance) {
            status = String.format("rejected jump %.1f in", distanceChange);
            return null;
        }

        double headingChange = Math.abs(AngleUnit.normalizeRadians(
                bestObservation.pose.getHeading() - comparisonPose.getHeading()
        ));
        if (headingChange > Tunables.maxHeadingJump) {
            status = String.format("rejected heading %.1f deg", Math.toDegrees(headingChange));
            return null;
        }

        status = "ACTIVE - localizing";
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
            status = String.format("no tags - %.1fs", timeSinceUpdate);
        } else {
            status = "searching for tags...";
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
        if (disagreement <= Tunables.maxTagDisagreement) {
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
