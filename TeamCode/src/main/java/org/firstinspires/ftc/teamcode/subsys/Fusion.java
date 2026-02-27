package org.firstinspires.ftc.teamcode.subsys;

import android.annotation.SuppressLint;

import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

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

    private int visionRejectedCount = 0;

    public Fusion(Pose startingPose) {
        resetFilters(startingPose);
    }

    public void resetFilters(Pose startingPose) {
        xFilter = new KalmanFilter(startingPose.getX(), Tunables.fusionInitialVariance);
        yFilter = new KalmanFilter(startingPose.getY(), Tunables.fusionInitialVariance);

        lastOdoX = startingPose.getX();
        lastOdoY = startingPose.getY();
    }

    public void update(Pose odoPose, Pose visionPose) { // visionPose may be null
        double dx = odoPose.getX() - lastOdoX;
        double dy = odoPose.getY() - lastOdoY;

        lastOdoX = odoPose.getX();
        lastOdoY = odoPose.getY();

        /** predict step (must always happen **/
        // adjust noise with odometry distance traveled
        double Qx = Tunables.fusionProcessNoiseBase + Tunables.fusionProcessNoisePerInch * Math.abs(dx);

        double Qy = Tunables.fusionProcessNoiseBase + Tunables.fusionProcessNoisePerInch * Math.abs(dy);

        xFilter.predict(dx, Qx);
        yFilter.predict(dy, Qy);

        /** correct step (only if vision is accurate+valid **/
        if (visionPose != null) {
            if (visionPose.roughlyEquals(getState(), Tunables.fusionMaxVisionError)) {
                xFilter.correct(visionPose.getX(), Tunables.fusionVisionVariance);
                yFilter.correct(visionPose.getY(), Tunables.fusionVisionVariance);
            } else {
                visionRejectedCount++;
            }
        }
    }

    private boolean isReasonable(Pose odoPose, Pose visionPose, double reasonability) {
        // make sure vision isn't null (bad reading)
        if (visionPose == null) { return false; }
        // check if vision is reasonably close to odo
        return getState().distanceFrom(visionPose) < reasonability;
    }

    public Pose getState() {
        return new Pose(xFilter.getState(), yFilter.getState());
    }

    public String getStatus() {
        return "x filter: " + xFilter.getDebugString() +
                "\ny filter:" + yFilter.getDebugString() +
                "\nvision rejected count: " + visionRejectedCount;
    }

    private class KalmanFilter {
        private double x;   // state
        private double P;   // covariance

        // Debug values
        private double lastK;
        private double lastInnovation;
        private double lastQ;
        private double lastR;

        public KalmanFilter(double initialState, double initialVariance) {
            this.x = initialState;
            this.P = initialVariance;
        }

        public void predict(double u, double Q) {
            x += u;
            P += Q;

            P *= 1.001; // prevent slow collapse

            // clamp P to prevent covariance collapse
            P = Range.clip(P, Tunables.fusionMinVariance, Tunables.fusionMaxVariance);

            lastQ = Q;
        }

        public void correct(double z, double R) {
            double y = z - x;          // innovation
            double S = P + R;
            double K = P / S;

            x += K * y;
            P *= (1 - K);

            // store debug values
            lastInnovation = y;
            lastK = K;
            lastR = R;
        }

        public double getState() { return x; }
        public double getVariance() { return P; }

        @SuppressLint("DefaultLocale")
        public String getDebugString() {
            return String.format(
                    "x=%.2f P=%.3f K=%.3f innov=%.2f Q=%.4f R=%.2f",
                    x, P, lastK, lastInnovation, lastQ, lastR
            );
        }
    }}
