package org.firstinspires.ftc.teamcode.tuner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Brushland Tuner", group = "Tuner")
public class BrushlandTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel pin0 = hardwareMap.digitalChannel.get("purplePin");
        DigitalChannel pin1 = hardwareMap.digitalChannel.get("greenPin");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("digital 0", pin0.getState());
            telemetry.addData("digital 1", pin1.getState());
            telemetry.update();
        }
    }
}
