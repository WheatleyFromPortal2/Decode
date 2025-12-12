/** this class holds all of our code that interfaces with our Limelight **/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision {
    private static Limelight3A limelight;

    public Vision(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    public boolean update() { // update our vision
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            result.getTx();

        }
        return false;
    }
}
