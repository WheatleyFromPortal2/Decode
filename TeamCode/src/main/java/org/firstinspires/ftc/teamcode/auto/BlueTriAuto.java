package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueTriAuto", group = "Auto")
public class BlueTriAuto extends BlueAuto{
    @Override
    public Pose getStartPose() {
        return new Pose(56.60215053763441, 8.602150537634405, Math.toRadians(90)); // Start Pose of our robot.
    }
}
