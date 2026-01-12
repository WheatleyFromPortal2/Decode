/** the purpose of OpMode is to test servo endpoints and max flywheel speed
 * MAKE SURE BOTH SERVOS ARE NOT GOING TO RUN INTO ANYTHING!!! **/

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="ServoTest", group="Util")
public class ServoTest extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap); // create our robot class

        telemetry.addLine("MAKE SURE BOTH SERVOS ARE NOT GOING TO RUN INTO ANYTHING!!!"); // we're still prob gonna break some servos :(

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double lowerTransferAmount = (gamepad1.left_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)
            double upperTransferAmount = (gamepad1.right_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)

            robot.lowerTransfer.setPosition(lowerTransferAmount);
            robot.upperTransfer.setPosition(upperTransferAmount);

            telemetry.addData("lowerTransfer:", robot.lowerTransfer.getPosition());
            telemetry.addData("upperTransfer:", robot.upperTransfer.getPosition());

            telemetry.update();
            idle();
        }
    }
}