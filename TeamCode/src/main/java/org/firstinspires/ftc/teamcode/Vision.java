/** this class holds all of our code that interfaces with our Limelight **/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Vision {
    private static Limelight3A limelight;
    Pose lastBotpose;
    double lastGoalTx;
    Robot.Pattern lastPattern = Robot.Pattern.UNKNOWN;
    PID turnController;

    public Vision(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        turnController = new PID(Tunables.turnP, Tunables.turnI, Tunables.turnD);
    }

    public void start() {
        limelight.start();
    }

    public boolean update() { // update our vision
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            updatePosition(result.getBotpose()); // let the conversion be handled by another function
            updatePatternAndTx(result.getFiducialResults()); // pass lists of tags to detect ID
            return true; // we actually have a good result
        } else { return false; } // we haven't gotten any new results
    }

    public void updatePosition(Pose3D botpose3D) { // this handles all the conversion from Pose3D->Pedro Pose and updates lastBotpose
        Position botPosition = botpose3D.getPosition();
        YawPitchRollAngles botOrientation = botpose3D.getOrientation();
        lastBotpose = new Pose(botPosition.x, botPosition.y, botOrientation.getYaw(AngleUnit.RADIANS)); // convert to pedro, prob need to apply some translations
    }

    public void updatePatternAndTx(List<FiducialResult> fiducialResults) { // update what pattern we have and update our goal tx
        if (fiducialResults.isEmpty()) { return; } // if we don't have any results, don't both
        for (int i = 0; i <= fiducialResults.size(); i++) { // loop through all results
            FiducialResult result = fiducialResults.get(i);
            int id = result.getFiducialId();

            switch (id) { // set our pattern based on the ID, if we don't have a pattern ID, don't change it
                // goal IDs are 20, 24
                // ideally, we should know whether we are on Blue/Red team to disregard the other, but right now the driver must first point at the correct goal
                // TODO: add logic to disregard goal of opposite team
                case 20:
                    lastGoalTx = result.getTargetXDegrees();
                    break;
                case 24:
                    lastGoalTx = result.getTargetXDegrees();
                    break;
                // pattern IDs are 21, 22, 23
                case 21:
                    lastPattern = Robot.Pattern.GPP;
                    break;
                case 22:
                    lastPattern = Robot.Pattern.PGP;
                    break;
                case 23:
                    lastPattern = Robot.Pattern.PPG;
                    break;
            }
        }
    }

    public Pose getLastBotpose() { return lastBotpose; }
    public double getLastGoalTx() { return lastGoalTx; }

    public double getGoalTurn() { // returns a double (-1)<->(1) that specifies how much Pedro should turn by to point towards the goal
        turnController.set(Tunables.turnP, Tunables.turnI, Tunables.turnD); // update our coefficients (they may have been changed with Configurables)
        return turnController.calc(lastGoalTx); // return our correction
    }

}
