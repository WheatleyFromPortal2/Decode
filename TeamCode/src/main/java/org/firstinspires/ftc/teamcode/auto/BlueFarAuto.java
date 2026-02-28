package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous(name = "BlueFarAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueFarAuto extends FarAuto {
    public static Pose startPose = new Pose(48, 8.5, Math.toRadians(180));
    public static Pose pickupPose = new Pose(6, 8.5, Math.toRadians(180));
    public static double turretPos = Math.toRadians(105); // absolute based off of field

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
