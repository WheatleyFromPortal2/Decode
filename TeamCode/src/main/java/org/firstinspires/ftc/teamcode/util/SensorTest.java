/** the purpose of OpMode is to test distance sensor endpoints */

package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

@TeleOp(name="SensorTest", group="Util")
public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap); // create our robot class
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
        Timer functionTimer = new Timer();

        waitForStart();

        while (opModeIsActive()) {
            telemetryM.debug("---all distances are in millimeters---");

            telemetryM.debug("---sensor values---");
            telemetryM.addData("intakeSensor", robot.intakeSensor.getDistance(DistanceUnit.MM));
            telemetryM.addData("lowerTransferSensor", robot.lowerTransferSensor.getDistance(DistanceUnit.MM));
            telemetryM.addData("upperTransferSensor", robot.upperTransferSensor.getDistance(DistanceUnit.MM));

            telemetryM.addLine(""); // spacing
            telemetryM.debug("---sensor endpoints---");
            telemetryM.addData("intakeSensorOpen", Tunables.intakeSensorOpen);
            telemetryM.addData("lowerTransferSensorOpen", Tunables.lowerTransferSensorOpen);
            telemetryM.addData("upperTransferSensorOpen", Tunables.upperTransferSensorOpen);

            telemetryM.addLine(""); // spacing
            telemetryM.debug("---function outputs---");
            functionTimer.resetTimer();
            telemetryM.addData("isBallInIntake()", robot.isBallInIntake());
            telemetryM.addData("isBallInLowerTransfer()", robot.isBallInLowerTransfer());
            telemetryM.addData("isBallInUpperTransfer()", robot.isBallInUpperTransfer());
            telemetryM.addData("total sensor check time (millis)", functionTimer.getElapsedTime());

            telemetryM.update(telemetry); // update our telemetry
            idle();
        }
    }
}
