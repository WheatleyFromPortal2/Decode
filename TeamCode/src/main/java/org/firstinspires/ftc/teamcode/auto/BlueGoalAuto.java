package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueGoalAuto", group = "Blue", preselectTeleOp = "BozoTeleOp")
public class BlueGoalAuto extends BlueAuto {
    @Override
    public Pose getStartPose() {
        return new Pose(56.03347280334728, 136.01673640167365, Math.toRadians(270)); // Start Pose of our robot.
    }
}
