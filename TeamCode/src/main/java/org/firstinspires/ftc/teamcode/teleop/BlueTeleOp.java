package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlueTeleOp", group="TeleOp")
public class BlueTeleOp extends BozoTeleOp {
    @Override
    public Pose flipPose(Pose switchoverPose) {
        return switchoverPose.setHeading(switchoverPose.getHeading() + Math.toRadians(180));
    }

    @Override
    public Pose getGoalPose() {
        return new Pose (12, 138); // taken from BlueAuto.java

    }
}
