// the purpose of OpMode is to just test and record servo endpoints
// MAKE SURE BOTH SERVOS ARE NOT GOING TO RUN INTO ANYTHING!!!

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="ServoTest", group="TeleOp")
public class ServoTest extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap); // create our robot class

        telemetry.update();

        while (opModeIsActive()) {
            double lowerTransferAmount = (gamepad1.left_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)
            double upperTransferAmount = (gamepad1.right_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)

            robot.lowerTransfer.setPosition(lowerTransferAmount);
            robot.lowerTransfer.setPosition(upperTransferAmount);

            telemetry.addData("lowerTransfer:", robot.lowerTransfer.getPosition());
            telemetry.addData("upperTransfer:", robot.upperTransfer.getPosition());

            telemetry.update();

            idle();
        }
    }
}