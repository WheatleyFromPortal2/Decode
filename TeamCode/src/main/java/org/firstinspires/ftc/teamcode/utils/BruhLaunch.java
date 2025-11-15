package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name="BruhLaunch", group="Util")
public class BruhLaunch extends LinearOpMode {
    public DcMotorEx launch;
    @Override
    public void runOpMode() {
        launch = hardwareMap.get(DcMotorEx.class, "launch");

        waitForStart();
        while (opModeIsActive()) {
            launch.setPower(1);

            telemetry.addData("launch TPS", launch.getVelocity());
            telemetry.update();
            idle();
        }
    }

}
