package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "RedGoalAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedGoalAuto extends RedAuto {
    public Pose getStartPose() {
        return new Pose(112, 153.51, Math.toRadians(270)); // Start Pose of our robot.
    }
}
