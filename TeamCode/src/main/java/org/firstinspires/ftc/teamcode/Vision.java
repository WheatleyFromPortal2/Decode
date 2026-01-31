/** this class holds all of our code that interfaces with our Limelight **/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.util.Timer;

public class Vision {
    public static Limelight3A limelight;
    double lastGoalTx, lastGoalDistance, lastGoalTa;
    boolean isBlue; // whether we are blue or red team
    private boolean started = false;
    private Timer staleTimer;

    public Vision(HardwareMap hw, boolean isBlueTeam) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

        staleTimer = new Timer();

        isBlue = isBlueTeam;
    }

    public void start() {
        limelight.start();

        if (isBlue) limelight.pipelineSwitch(1);
        else limelight.pipelineSwitch(2);

        started = true;
    }

    public boolean update() { // update our vision

        if (!started) { // make sure we have started
            throw new RuntimeException("make sure to call vision.start()!");
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) { // if our result is good
            staleTimer.resetTimer(); // reset our staleness timer

            lastGoalTx = result.getTx();
            lastGoalTa = result.getTa();
            lastGoalDistance = getGoalDistance(lastGoalTa);
            return true; // we actually have a good result
        } else { return false; } // we haven't gotten any new results
    }

    private double getGoalDistance(double ta) { // returns goal distance in inches
        double d = 62.43976 * Math.pow(ta, -0.542823); // from Desmos data: 1-30-26
        // R^2 = 0.8912 using power regression (log mode = false)
        return d;
    }

    public double getLastGoalTx() { return lastGoalTx; }
    public double getLastGoalDistance() { return lastGoalDistance; }
    public double getLastGoalTa() { return lastGoalTa; }
    public boolean isStale() { return staleTimer.getElapsedTime() >= Tunables.maxVisionStaleness; }
    public LLStatus getStatus() { return limelight.getStatus(); }
}
