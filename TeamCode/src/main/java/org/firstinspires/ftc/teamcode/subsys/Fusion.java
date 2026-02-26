package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Tunables;

import java.util.Arrays;

public class Fusion {
    /*
    1. take in data from Pedro Pathing estimated position and Limelight position
    2. if vision data exists, check it for errors and even reject
    3. use Kalman filter to intelligently combine
    4. return result

    - we shouldn't need to update Pedro Pathing's position, since our TeleOp will just rely upon this and auto uses odo only
     */
    private KalmanFilter xFilter;
    private KalmanFilter yFilter;

    private double lastOdoX;
    private double lastOdoY;

    public Fusion(Pose startingPose) {
        // x/y noise should be identical
        KalmanFilterParameters xParams = new KalmanFilterParameters(Tunables.modelCovariance, Tunables.dataCovariance);
        KalmanFilterParameters yParams = new KalmanFilterParameters(Tunables.modelCovariance, Tunables.dataCovariance);

        xFilter = new KalmanFilter(xParams);
        yFilter = new KalmanFilter(yParams);

        resetPos(startingPose);
    }

    public void update(Pose odoPose, Pose visionPose) { // visionPose may be null
        double dx = odoPose.getX() - lastOdoX;
        double dy = odoPose.getY() - lastOdoY;

        lastOdoX = odoPose.getX();
        lastOdoY = odoPose.getY();

        if (!isReasonable(odoPose, visionPose)) {
            // our vision data is not reliable - base only off of odo
            xFilter.update(dx, xFilter.getState());
            yFilter.update(dy, yFilter.getState());
        } else {
            // vision data is reliable, update our filters
            xFilter.update(dx, visionPose.getX());
            yFilter.update(dy, visionPose.getY());
        }
    }

    private boolean isReasonable(Pose odoPose, Pose visionPose) {
        // make sure vision isn't null (bad reading)
        if (visionPose == null) { return false; }
        // check if vision is reasonably close to odo
        return getState().distanceFrom(visionPose) < Tunables.maxVisionVariance;
    }

    public void resetPos(Pose startingPose) {
        lastOdoX = startingPose.getX();
        lastOdoY = startingPose.getY();

        // start variance must not be = 0
        xFilter.reset(startingPose.getX(), 0.01, 0.0);
        yFilter.reset(startingPose.getY(), 0.01, 0.0);
    }

    public Pose getState() {
        return new Pose(xFilter.getState(), yFilter.getState());
    }

    public String getStatus() {
        return Arrays.toString(xFilter.output()) + Arrays.toString(yFilter.output());
    }
}
