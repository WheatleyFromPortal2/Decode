/** this class holds all of our code that interfaces with our Limelight **/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Vision {
    public static Limelight3A limelight;
    double lastGoalTx, lastGoalDistance, lastGoalTa;
    boolean isBlue; // whether we are blue or red team
    PID turnController;
    private boolean started = false;

    public Vision(HardwareMap hw, boolean isBlueTeam) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

        if (isBlueTeam) limelight.pipelineSwitch(1);
        else limelight.pipelineSwitch(2);

        turnController = new PID(Tunables.turnP, Tunables.turnI, Tunables.turnD);
        isBlue = isBlueTeam;
    }

    public void start() {
        limelight.start();
        started = true;
    }

    public boolean update() { // update our vision

        if (!started) { // make sure we have started
            throw new RuntimeException("make sure to call vision.start()!");
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            lastGoalTx = result.getTx();
            lastGoalTa = result.getTa();
            lastGoalDistance = getGoalDistance(lastGoalTa);
            return true; // we actually have a good result
        } else { return false; } // we haven't gotten any new results
    }

    private double getGoalDistance(double ta) { // returns goal distance in inches
        double d = 18.86428*ta*ta - 91.66931*ta + 138.66433; // from Desmos
        return d + Tunables.goalOffset;
    }

    public double getLastGoalTx() { return lastGoalTx; }
    public double getLastGoalDistance() { return lastGoalDistance; }
    public double getLastGoalTa() { return lastGoalTa; }
    public boolean isStale() { return limelight.getTimeSinceLastUpdate() >= Tunables.maxVisionStaleness; }
    public LLStatus getStatus() { return limelight.getStatus(); }

    public double getGoalTurn() { // returns a double (-1)<->(1) that specifies how much Pedro should turn by to point towards the goal
        turnController.set(Tunables.turnP, Tunables.turnI, Tunables.turnD); // update our coefficients (they may have been changed with Configurables)
        return turnController.calc(lastGoalTx); // return our correction
    }

}
