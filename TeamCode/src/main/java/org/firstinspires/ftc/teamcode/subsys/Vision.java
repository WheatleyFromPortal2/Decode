/** this class holds all of our code that interfaces with our Limelight **/
package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Tunables;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class Vision {
    public static Limelight3A limelight;
    public enum Pipeline { // expression of our limelight pipelines; order and elements must match exactly with pipeline indices on camera
        // we're going to keep legacy pipelines 1/2 for now, since we want the rest of our branches to stay functional until full positioning is proven reliable
        OBELISK, // pipeline index: 0
        BLUE_TEAM, // pipeline index: 1
        RED_TEAM, // pipeline index: 2
        FULL_POS // pipeline index: 3
    }

    public enum Triplet {
        GPP, // green-purple-purple; ID: 21
        PGP, // purple-green-purple; ID: 22
        PPG, // purple-purple-green; ID: 23
        UNKNOWN // pattern has yet to be determined
    }

    private boolean started = false;
    private Timer staleTimer;
    private Pose lastBotPose;
    private Triplet lastTriplet = Triplet.UNKNOWN;


    public Vision(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask our Limelight for data (100 times per second)

        staleTimer = new Timer();
    }

    public void startPipeline(Pipeline pipeline) {
        limelight.start();
        setPipeline(pipeline);
        started = true;
    }

    public void checkPipelineReadiness(Pipeline pipeline) { // check to make sure we have started and are ready to use our desired pipeline
        if (!started) { // make sure we have started
            throw new RuntimeException("make sure to call vision.start()!");
        }
        if (getPipeline() != pipeline) {
            String error = "you tried to update using pipeline: " + getPipeline() + ", but pipeline: " + pipeline + " was expected!";
            throw new RuntimeException(error);
        }
    }


   public Pose updateFullPos(double odoHeadingRadians, double turretHeadingRadians) { // update our vision
        checkPipelineReadiness(Pipeline.FULL_POS);
        //seedLimelightHeading(odoHeadingRadians, turretHeadingRadians);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && !Double.isNaN(result.getTa())) { // if our result is good
            staleTimer.resetTimer(); // reset our staleness timer
            lastBotPose = translateLLPoseToField(result.getBotpose(), odoHeadingRadians, turretHeadingRadians);
            return lastBotPose;
        } else { return null; } // we haven't gotten any new results
    }

    public void updateObelisk() { // super simple update, only checks for pattern IDs; doesn't worry about full robot position
        /* return values:
            - false: we don't have a valid obelisk ID yet
            - true: we have received a valid obelisk ID
         */
        checkPipelineReadiness(Pipeline.OBELISK);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) { // if our result is good
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                // loop through every result, checking its ID
                for (int i = 0; i < fiducialResults.size(); i++) {
                    int id = fiducialResults.get(i).getFiducialId();

                    switch (id) {
                        case 21:
                            lastTriplet = Triplet.GPP;
                            break;
                        case 22:
                            lastTriplet = Triplet.PGP;
                            break;
                        case 23:
                            lastTriplet = Triplet.PPG;
                            break;
                        default: // any other ID; ID filter on limelight should prevent this, but stuff breaks (tm)
                            break; // we haven't received a valid ID yet
                    }
                }
            }
        }
    }

    private double getGoalDistance(double ta) { // returns goal distance in inches
        return 62.0787 * Math.pow(ta, -0.550239); // from Desmos data: 1-30-26 (fixed)
        // R^2 = 0.9965 using power regression (log mode = false)
    }

    private int getPipelineIndex() { return getStatus().getPipelineIndex(); } // Limelight pipelines are zero indexed

    public void setPipeline(Pipeline pipeline) {
        if (getPipeline() != pipeline) { // make sure our pipeline is not already correct
            limelight.pipelineSwitch(pipeline.ordinal()); // convert from Pipeline enum to ordinal/index
        }
    }

    private void seedLimelightHeading(double odoHeadingRadians, double turretHeadingRadians) {
        // seed our limelight's MT2 algorithm with a good estimate of where its heading is
        double absoluteLimelightHeading = getLimelightAbsoluteHeading(odoHeadingRadians, turretHeadingRadians);
        limelight.updateRobotOrientation(Math.toDegrees(absoluteLimelightHeading)); // assume limelight uses degrees for orientation
    }

    private double getLimelightAbsoluteHeading(double odoHeadingRadians, double turretHeadingRadians) {
        return odoHeadingRadians + turretHeadingRadians - Math.toDegrees(90);
    }

    /* take in raw pose of the limelight and convert it to a field pose.
    to do this we use inverse kinematics through applying:
    - field heading from odometry
    - turret heading from encoder
    */
    private Pose translateLLPoseToField(Pose3D rawPose3D, double odoHeadingRadians, double turretHeadingRadians) {
        // TODO: add translation from turret not being centered with drivetrain center (which is used by Pedro Pathing)
        double rawHeadingRadians = rawPose3D.getOrientation().getYaw(AngleUnit.RADIANS);
        double limelightHeadingRadians = getLimelightAbsoluteHeading(odoHeadingRadians, turretHeadingRadians);
        Position rawPos = rawPose3D.getPosition().toUnit(DistanceUnit.INCH); // ensure we are using inches
        double rawX = rawPos.y + 72;
        double rawY = 72 - rawPos.x;
        // we don't care about z
        // first let's check our heading. it should have already been seeded well, so if it drifts from our actual heading by too much,
        // then it is likely that this measurement is outdated, and it should therefor be discarded

        /*
        if (Math.abs(AngleUnit.normalizeRadians(rawHeadingRadians - limelightHeadingRadians)) > Tunables.maxLimelightHeadingError) {
            return null; // we don't have a good reading, don't rely off of this data
        }
        */

        double centeredX = rawX + Tunables.turretOffsetX;
        double centeredY = rawY + Tunables.turretOffsetY;

        /* calc our offset from the turret center using trigonometric functions:
        1. we draw a circle in a cartesian plane with a radius of Tunables.limelightTurretRadius
        2. we draw a right triangle, with:
            a. leg1 with a length of x inches
            b. leg2 with a length of y inches
            c. hypotenuse with:
                1. one point at the center of the circle
                2. one point on the circle
                3. a length of the radius of the circle (Tunables.limelightTurretRadius)
        3. we use:
            a. the angle of the turret (a / limelightHeadingRadians)
            b. the length of the hypotenuse (Tunables.limelightTurretRadius)
            c. sohcahtoa
           in order to figure out the length of the legs (x and y)
            a. cos(a) = y/r
            b. sin(a) = y/r
           so:
            x: r * cos(a)
            y: r * sin(a)
        */

        double camOffsetX = Tunables.limelightTurretRadius * Math.sin(limelightHeadingRadians);
        double camOffsetY = Tunables.limelightTurretRadius * Math.cos(limelightHeadingRadians);

        double fieldX = camOffsetX + centeredX;
        double fieldY = camOffsetY + centeredY;

        return new Pose(fieldX, fieldY, odoHeadingRadians);

        // use our pinpoint's compass for heading, it is good enough until Kalman Filter
        //return new Pose(fieldX, fieldY, odoHeadingRadians);

        // why did we have to use 3 different coordinate and orientation systems?
    }


    /** getter methods **/

    public Pose getLastBotPose() { return lastBotPose; }
    public Triplet getLastTriplet() { return lastTriplet; }
    public boolean isStale() { return staleTimer.getElapsedTime() >= Tunables.maxVisionStaleness; }
    public double getStaleness() { return staleTimer.getElapsedTime(); }
    public LLStatus getStatus() { return limelight.getStatus(); }
    public Pipeline getPipeline() { return Pipeline.values()[getPipelineIndex()]; } // convert from limelight pipeline ordinal/index to Pipeline enum
}
