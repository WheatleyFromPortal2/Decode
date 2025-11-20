package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueGoalAuto", group = "Blue", preselectTeleOp = "BozoTeleOp")
public class BlueGoalAuto extends BlueAuto {
    @Override
    public Pose getStartPose() {
        return new Pose(53.826193021038385, 155.51140732652559, Math.toRadians(270)); // Start Pose of our robot.
    }
}
