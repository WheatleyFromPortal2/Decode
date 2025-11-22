package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueGoalAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueGoalAuto extends BlueAuto {
    @Override
    public Pose getStartPose() {
        return new Pose(56.03347280334728, 155.51140732652559, Math.toRadians(270)); // Start Pose of our robot.
    }
}
