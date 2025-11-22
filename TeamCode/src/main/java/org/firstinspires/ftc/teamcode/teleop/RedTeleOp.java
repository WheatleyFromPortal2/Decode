package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="RedTeleOp", group="TeleOp")
public class RedTeleOp extends BozoTeleOp {
    @Override
    public Pose flipPose(Pose switchoverPose) {
        return switchoverPose;
    }
}
