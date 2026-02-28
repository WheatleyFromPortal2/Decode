package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class HandoffState {
    // all of these should start off at defaults, so if we test TeleOp, it will assume defaults
    public static Pose pose = new Pose(72, 72, Math.toRadians(90)); // pose to hand off from auto->TeleOp

    public static double turretPos = 0;
}
