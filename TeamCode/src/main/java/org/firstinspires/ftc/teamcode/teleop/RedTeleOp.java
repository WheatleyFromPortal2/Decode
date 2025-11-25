package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="RedTeleOp", group="TeleOp")
public class RedTeleOp extends BozoTeleOp {
    @Override
    public Pose flipPose(Pose switchoverPose) {
        return switchoverPose;
    }

    @Override
    public Pose getGoalPose() {
        return new Pose (132, 138); // taken from RedAuto.java

    }
}
