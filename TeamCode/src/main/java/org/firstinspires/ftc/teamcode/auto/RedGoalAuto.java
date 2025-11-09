package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "RedGoalAuto", group = "Auto")
public class RedGoalAuto extends RedAuto {
    public Pose getStartPose() {
        return new Pose(120.08602150537634, 126.10752688172043, Math.toRadians(-143)); // Start Pose of our robot.
    }
}
