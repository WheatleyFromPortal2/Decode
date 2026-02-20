package org.firstinspires.ftc.teamcode.tuner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="Brushland Analog Tuner", group = "Tuner")
public class BrushlandAnalogTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput pin0 = hardwareMap.analogInput.get("analog0");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("voltage", pin0.getVoltage());
            telemetry.update();
        }
    }
}
