/** the purpose of OpMode is to test flywheel max speed and tune flywheel PIDF **/

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
                robot.setLaunchVelocity(robot.RPMToTPS(launchRPM)); // set our desired velocity from converting our desired RPM
                telemetryM.addData("desiredRPM", launchRPM);
                telemetryM.debug("in GRADUAL CONTROL");
                robot.calcPIDF();
            } else {
                // test launch full power speed
                robot.launchLeft.setPower(1); // BRRRRRR
                robot.launchRight.setPower(1); // BRRRRRR
                telemetryM.addData("desired RPM", 6000 * Tunables.launchRatio);
                telemetryM.debug("in FULL POWER");
            }
            telemetryM.debug("PIDF: " + Tunables.launchP + ", " + Tunables.launchI + ", " + Tunables.launchD + ", " + Tunables.launchF);
            telemetryM.debug("launchRatio: " + Tunables.launchRatio);
            telemetryM.addData("launchCorrection", robot.launchCorrection);
            telemetryM.addData("actualRPM", robot.getLaunchRPM());
            telemetryM.addData("desiredTPS", robot.RPMToTPS(robot.getDesiredLaunchRPM()));
            telemetryM.addData("leftLaunchTPS", robot.launchLeft.getVelocity());
            telemetryM.addData("rightLaunchTPS", robot.launchRight.getVelocity());
            telemetryM.addData("rawLaunchRPM", robot.getLaunchRPM());
            telemetryM.addData("launchCurrent", robot.getLaunchCurrent());
            telemetryM.update(telemetry); // update our telemetry
            idle();
        }
    }
}