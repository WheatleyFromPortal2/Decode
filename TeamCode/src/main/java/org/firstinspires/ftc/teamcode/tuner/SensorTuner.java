/** the purpose of OpMode is to test distance sensors and tune endpoints */

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.launch.Intake;
import org.firstinspires.ftc.teamcode.subsys.launch.Transfer;

@TeleOp(name="SensorTuner", group="Tuner")
public class SensorTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
        Timer functionTimer = new Timer();

        waitForStart();

        while (opModeIsActive()) {
            telemetryM.debug("---sensor endpoints---");
            telemetryM.addData("intakeSensorOpen", Tunables.intakeSensorOpen);

            telemetryM.addLine(""); // spacing
            telemetryM.debug("---function outputs---");
            functionTimer.resetTimer();
            telemetryM.addData("is ball in intake", intake.isBallInIntake());
            telemetryM.addData("is ball in lower transfer", transfer.isBallInLower());
            telemetryM.addData("is ball in upper transfer", transfer.isBallInUpper());
            telemetryM.addLine("");

            telemetryM.addData("total sensor check time (millis)", functionTimer.getElapsedTime());

            telemetryM.update(telemetry); // update our telemetry
            idle();
        }
    }
}
