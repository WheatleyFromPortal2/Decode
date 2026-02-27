package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueFarAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueFarAuto extends FarAuto {
    @Override
    protected Pose getStartPose() {
        return new Pose(48, 8.5, Math.toRadians(180));
    }

    @Override
    protected Pose getPickupPose() {
        return new Pose(6, 8.5, Math.toRadians(180));
    }
}
