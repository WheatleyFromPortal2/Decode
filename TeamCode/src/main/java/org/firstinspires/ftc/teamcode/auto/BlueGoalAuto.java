package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueGoalAuto", group = "Blue", preselectTeleOp = "BozoTeleOp")
public class BlueGoalAuto extends BlueAuto {
    @Override
    public Pose getStartPose() {
        return new Pose(23.913978494623656, 125.93548387096774, Math.toRadians(-36)); // Start Pose of our robot.
    }
}
