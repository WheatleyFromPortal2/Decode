/** the purpose of OpMode is to test our vision code */

package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision;

@TeleOp(name="VisionTest", group="Util")
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, true); // let's just say that we are blue team
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        waitForStart();

        while (opModeIsActive()) {
            vision.update();
            telemetryM.addData("last goal distance", vision.getLastGoalDistance());
            telemetryM.addData("last goal tx", vision.getLastGoalTx());
            telemetryM.addData("last goal ta", vision.getLastGoalTa());
            telemetryM.update(telemetry); // update our telemetry
            idle();
        }
    }
}