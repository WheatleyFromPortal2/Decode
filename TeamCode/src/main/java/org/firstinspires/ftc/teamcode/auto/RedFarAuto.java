package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous(name = "RedFarAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedFarAuto extends FarAuto {
    public static Pose startPose = new Pose(96, 8.5, Math.toRadians(0));
    public static Pose pickupPose = new Pose(138, 8.5, Math.toRadians(0));
    public static double turretPos = Math.toRadians(68);

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected Pose getPickupPose() {
        return pickupPose;
    }

    @Override
    protected double getTurretPos() {
        return turretPos;
    }
}
