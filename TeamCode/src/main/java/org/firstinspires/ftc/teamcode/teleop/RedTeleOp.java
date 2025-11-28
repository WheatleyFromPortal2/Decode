package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.RedAuto;


@TeleOp(name="RedTeleOp", group="TeleOp")
public class RedTeleOp extends BozoTeleOp {
    @Override // how the heading of our switchover pose should be adjusted to reflect the driver position
    public Pose flipPose(Pose switchoverPose) {
        return switchoverPose;
    }

    @Override
    public Pose getGoalPose() { // return our goal pose
        // because the buildConfig() function is protected, we cannot call it here - possibly replace separate variables
        return new Pose (132, 138); // taken from RedAuto.java
    }
}
