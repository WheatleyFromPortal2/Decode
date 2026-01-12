/** the purpose of OpMode is to test our vision code */

package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision;

@TeleOp(name="VisionTest", group="Util")
public class VisionTest extends LinearOpMode {
    public boolean isBlueTeam = true;

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, isBlueTeam); // let's just say that we are blue team
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
        vision.start();

        waitForStart();

        while (opModeIsActive()) {
            vision.update();
            telemetryM.debug("isBlueTeam: " + isBlueTeam);

            telemetryM.addData("last goal distance", vision.getLastGoalDistance());
            telemetryM.addData("last goal tx", vision.getLastGoalTx());
            telemetryM.addData("last goal ta", vision.getLastGoalTa());
            telemetryM.addData("limelight status", vision.getStatus());
            telemetryM.update(telemetry); // update our telemetry
            idle();
        }
    }
}