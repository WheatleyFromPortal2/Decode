// the purpose of OpMode is to test servo endpoints and max flywheel speed
// MAKE SURE BOTH SERVOS ARE NOT GOING TO RUN INTO ANYTHING!!!

package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

@TeleOp(name="FlywheelTuner", group="Util")
public class FlywheelTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap); // create our robot class
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
        boolean isGradualControl = true; // if we're using the right trigger to control speed or just doing full power

        telemetryM.debug("use the right trigger to control launch speed");
        telemetryM.debug("use the B button to go between gradual control/full power");

        waitForStart();

        while (opModeIsActive()) {
            double launchRPM = ((gamepad1.right_trigger) * (6000 * Tunables.launchRatio)); // calculates max motor speed and multiplies it by the float of the right trigger

            if (gamepad1.bWasReleased()) { // switch modes we press the B button
                isGradualControl = !isGradualControl;
            }

            if (isGradualControl) {
                robot.launch.setVelocity(robot.RPMToTPS(launchRPM)); // set our desired velocity to our desired RPM
                telemetryM.addData("desired RPM", launchRPM);
                telemetryM.debug("in GRADUAL CONTROL");
            } else {
                robot.launch.setPower(1); // BRRRRRR
                telemetryM.addData("desired RPM", 6000 * Tunables.launchRatio);
                telemetryM.debug("in FULL POWER");
            }
            telemetryM.addData("actual RPM", robot.getLaunchRPM());
            telemetryM.addData("raw launch TPS", robot.launch.getVelocity());
            telemetryM.addData("raw launch RPM", robot.launch.getVelocity() * 60 / 28); // this should work
            telemetryM.addData("launch current", robot.getLaunchCurrent());
            telemetryM.addData("launch ratio", Tunables.launchRatio);
            telemetryM.update(telemetry); // update our telemetry
            idle();
        }
    }
}