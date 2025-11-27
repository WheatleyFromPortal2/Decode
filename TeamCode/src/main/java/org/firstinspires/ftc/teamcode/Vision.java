package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

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
    private double lastPositionJump; // store our last change in position from reference pose (provided by odo) and our vision
    private double lastHeadingJump; // store our last change in heading from reference pose (provided by odo) and our vision
    private double lastSuccessfulUpdateTime = 0.0;
    private Pose lastVisionPose = null;
    private Robot.Pattern pattern = Robot.Pattern.UNKNOWN; // stores the pattern that we think we have (GPP, PGP, PPG), start off as unknown

    public Vision() { // create and initialize our vision object

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
        builder.setCameraResolution(Tunables.cameraResolution);
        builder.enableLiveView(Tunables.enableLiveView);
        builder.setStreamFormat(Tunables.streamFormat);
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    public Pose tryGetVisionPose(Pose referencePose) {
        long now = System.currentTimeMillis(); // get what time it is
        if (now - lastQueryTime < Tunables.aprilTagUpdateInterval) { // don't update if we are outside of the interval
            return null;
        }
        lastQueryTime = now; // update our last query time

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections(); // get our detections from the processor

        if (detections == null) { // we didn't get any detections
            status = "no detections";
            lastDetectionCount = 0;
            return null;
        }

        lastDetectionCount = detections.size(); // get the amount of detections that we received
        List<VisionObservation> observations = new ArrayList<>(); // create our observations list
        for (AprilTagDetection detection : detections) {
            VisionObservation observation = createObservation(detection);
            if (observation != null) { // add all of our observations that are not null
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
        double distanceChange = comparisonPose.distanceFrom(bestObservation.pose);
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
        lastPositionJump = distanceChange; // update our last position jump from the pose we're going to return
        lastHeadingJump = headingChange; // update our last heading jump from the pose we're going to return
        return bestObservation.pose;
    }

    // methods for reading status
    public int getLastDetectionCount() {
        return lastDetectionCount;
    }

    public String getStatus() {
        return status;
    }

    public Pose getLastVisionPose() {
        return lastVisionPose;
    }
    public double getLastPositionJump() {return lastPositionJump;}
    public double getLastHeadingJump() {return lastHeadingJump;} // this should return radians
    public Robot.Pattern getPattern() {return pattern;}

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /** private methods not to be called by OpModes **/
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

        double disagreement = first.pose.distanceFrom(second.pose); // find our distance between the 2 poses
        if (disagreement <= Tunables.maxTagDisagreement) {
            Pose averaged = averagePose(first.pose, second.pose);
            double bestRange = Math.min(first.range, second.range);
            return new VisionObservation(averaged, bestRange);
        }

        if (referencePose != null) {
            double firstDelta = referencePose.distanceFrom(first.pose); // find the distance between our reference Pose and our
            double secondDelta = referencePose.distanceFrom(second.pose);
            return firstDelta <= secondDelta ? first : second; // return the pose that is closest to our reference pose
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

    private VisionObservation createObservation(AprilTagDetection detection) {
        if (detection.metadata == null || detection.robotPose == null) { // if this detection is missing data, don't use it
            return null;
        }
        // pattern tags
        if (detection.id == 21) {
            pattern = Robot.Pattern.GPP; // update our pattern
            return null; // don't use the obelisk for localization
        } else if (detection.id == 22) {
            pattern = Robot.Pattern.PGP; // update our pattern
            return null; // don't use the obelisk for localization
        } else if (detection.id == 23) {
            pattern = Robot.Pattern.PPG; // update our pattern
            return null; // don't use the obelisk for localization
        }
        // obelisk tags
        else if (detection.id == 20 || detection.id == 24) { // the only localizing april tags are id 20 and 24
            double x = convertPositionComponent(detection.robotPose.getPosition(), DistanceUnit.INCH, Axis.X);
            double y = convertPositionComponent(detection.robotPose.getPosition(), DistanceUnit.INCH, Axis.Y);
            double heading = Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            Pose pose = new Pose(x, y, heading);
            double range = detection.ftcPose != null ? detection.ftcPose.range : Double.MAX_VALUE;
            return new VisionObservation(pose, range);
        }
        // if the ID is anything else, don't use it
        else {
            return null;
        }
    }

    private double convertPositionComponent(Position position, DistanceUnit desiredUnit, Axis axis) { // convert position components from and to different units
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

    private enum Axis { // used for convertPositionComponent
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
