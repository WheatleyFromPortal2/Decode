/** the purpose of OpMode is to test servo endpoints and max flywheel speed
 * MAKE SURE BOTH SERVOS ARE NOT GOING TO RUN INTO ANYTHING!!! **/

package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

@TeleOp(name="ServoTuner", group="Util")
public class ServoTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap); // create our robot class

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        boolean testingTransferServos = false;

        telemetry.addLine("MAKE SURE ALL SERVOS ARE NOT GOING TO RUN INTO ANYTHING!!!"); // we're still prob gonna break some servos :(

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double lowerTransferAmount = (gamepad1.left_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)
            double upperTransferAmount = (gamepad1.right_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)

            if (gamepad1.xWasReleased()) testingTransferServos = !testingTransferServos;
            if (gamepad1.yWasReleased()) robot.setHoodPosition(0); // force going to minimum

            if (testingTransferServos) {
                robot.lowerTransfer.setPosition(lowerTransferAmount);
                robot.upperTransfer.setPosition(upperTransferAmount);
            } else { // set to default positions to avoid breaking servos when not testing them
                robot.lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit);
                robot.upperTransfer.setPosition(Tunables.upperTransferOpen);
            }

            telemetryM.debug("testing transfer servos?: "  + testingTransferServos);
            telemetryM.addLine("toggle testingTransferServos with X");
            telemetryM.addData("lowerTransfer:", robot.lowerTransfer.getPosition());
            telemetryM.addData("upperTransfer:", robot.upperTransfer.getPosition());
            telemetryM.addData("hood has been set to minimum:", Tunables.hoodMinimum);
            telemetryM.addLine("you may adjust the hoodMinimum with Panels");
            telemetryM.addLine("set hood to minimum by pressing Y");
            telemetryM.addLine("control lowerTransfer with left stick Y");
            telemetryM.addLine("control upperTransfer with right stick Y");

            telemetryM.update(telemetry);
            idle();
        }
    }
}