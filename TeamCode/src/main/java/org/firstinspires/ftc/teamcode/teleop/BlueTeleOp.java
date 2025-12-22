package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlueTeleOp", group="TeleOp")
public class BlueTeleOp extends BozoTeleOp {
    @Override // how our x-stick input should be modified to reflect the driver's position
    public double flipControl() { return -1; } // don't switch the control

    @Override
    public Pose getGoalPose() {
        // because the buildConfig() function is protected, we cannot call it here - possibly replace separate variables
        return new Pose (12, 138); // taken from BlueAuto.java
    }

    @Override
    public boolean isBlueTeam() {
        return true;
    }
}
