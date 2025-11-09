package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "RedTriAuto", group = "Auto")
public class RedTriAuto extends RedAuto{
    @Override
    public Pose getStartPose() {
        return new Pose(86.53763440860214, 9.462365591397848, Math.toRadians(90)); // Start Pose of our robot.
    }
}