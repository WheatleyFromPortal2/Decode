package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlueTeleOp", group="TeleOp")
public class BlueTeleOp extends BozoTeleOp {
    @Override // how the heading of our switchover pose should be adjusted to reflect the driver position
    public Pose flipPose(Pose switchoverPose) {
        return switchoverPose.setHeading(switchoverPose.getHeading() + Math.toRadians(180));
    }

    @Override
    public Pose getGoalPose() {
        // because the buildConfig() function is protected, we cannot call it here - possibly replace separate variables
        return new Pose (12, 138); // taken from BlueAuto.java
    }
}
