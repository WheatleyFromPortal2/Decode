// the purpose of OpMode is to test servo endpoints and max flywheel speed
// MAKE SURE BOTH SERVOS ARE NOT GOING TO RUN INTO ANYTHING!!!

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="FlywheelTest", group="Util")
public class FlywheelTest extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap); // create our robot class
        telemetry.update();
        boolean isGradualControl = true; // if we're using the right trigger to control speed or just doing full power

        telemetry.addLine("use the right trigger to control launch speed");
        telemetry.addLine("use the B button to go between gradual control/full power");

        waitForStart();

        while (opModeIsActive()) {
            double launchRPM = ((gamepad1.right_trigger) * (6000 * Robot.launchRatio)); // calculates max motor speed and multiplies it by the float of the right trigger

            if (gamepad1.bWasReleased()) { // switch modes we press the B button
                isGradualControl = !isGradualControl;
            }

            if (isGradualControl) {
                robot.launch.setVelocity(robot.RPMToTPS(launchRPM)); // set our desired velocity to our desired RPM
                telemetry.addData("desired RPM: ", launchRPM);
                telemetry.addLine("in GRADUAL CONTROL");
            } else {
                robot.launch.setPower(1); // BRRRRRR
                telemetry.addData("desired RPM", 6000 * Robot.launchRatio);
                telemetry.addLine("in FULL POWER");
            }
            telemetry.addData("actual RPM", robot.getLaunchRPM());
            telemetry.addData("raw launch TPS", robot.launch.getVelocity());
            telemetry.addData("raw launch RPM", robot.launch.getVelocity() * 60 / 28); // this should work
            telemetry.addData("launch current", robot.getLaunchCurrent());
            telemetry.addData("launch ratio", Robot.launchRatio);
            telemetry.update();
            idle();
        }
    }
}