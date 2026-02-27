package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedFarAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedFarAuto extends FarAuto {
    @Override
    protected Pose getStartPose() {
        return new Pose(96, 8.5, Math.toRadians(0));
    }

    @Override
    protected Pose getPickupPose() {
        return new Pose(138, 8.5, Math.toRadians(0));
    }
}
