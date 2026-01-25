package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tunables;

@TeleOp(name="ToggleCalibrationMode", group="TeleOp")
public class ToggleCalibrationMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        if (Tunables.isCalibrationMode) {
            telemetry.addLine("calibration mode is ENABLED");
        } else {
            telemetry.addLine("calibration mode is DISABLED");
        }

        telemetry.addLine("to toggle calibration mode: press Start");
        telemetry.addLine("if you do not wish to do so: press Stop");
        telemetry.addLine("calibration mode may also be toggled in Panels at any time");
        telemetry.update();

        waitForStart();

        Tunables.isCalibrationMode = !Tunables.isCalibrationMode; // toggle calibration mode

        requestOpModeStop();
    }
}
