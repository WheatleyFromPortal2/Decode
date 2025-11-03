package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.IMU; // import IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // import IMU orientation
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles; // import angles for IMU orientation
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; // import angle units for easy conversion

@TeleOp(name="BozoTeleOp", group="TeleOp")
public class BozoTeleOp extends LinearOpMode {
    private Robot robot;
    private boolean isIntakePowered = false;
    private boolean isLaunchPowered = false;
    private static final double debounceTime = 1; // wait for half a second before reading new button inputs

    private double time = getRuntime();
    private double oldTime = getRuntime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap); // create our robot class

        // Drive motor directions

        telemetry.update();
        //lowerTransfer.setPosition(0);
        robot.upperTransfer.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            time = getRuntime();
            // Read raw joystick inputs
            double x = gamepad1.left_stick_y; // forward (idk why this is cooked)
            double y = -gamepad1.left_stick_x; // strafe (idk why this is cooked)
            double rx = -gamepad1.right_stick_x; // rotation (idk why this is cooked)
            double ry = gamepad1.right_stick_y; // launch power (temporary until algorithm)

            if (gamepad1.a && oldTime + debounceTime < time) {
                isIntakePowered = !isIntakePowered;
                oldTime = getRuntime();
            }
            if (gamepad1.b && oldTime + debounceTime < time) {
                isLaunchPowered = !isLaunchPowered;
            }

            // Read IMU heading (radians)

            double botHeading = robot.odo.getHeading(AngleUnit.RADIANS); // get our heading in radians
            telemetry.addData("Heading (rad)", botHeading);

            // Field-centric transform
            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1.0);
            double flPower = (rotatedY + rotatedX + rx) / denominator;
            double blPower = (rotatedY - rotatedX - rx) / denominator;
            double frPower = (rotatedY - rotatedX - rx) / denominator;
            double brPower = (rotatedY + rotatedX - rx) / denominator;

            if (gamepad1.left_bumper) { // Slow mode
                flPower *= 0.5;
                blPower *= 0.5;
                frPower *= 0.5;
                brPower *= 0.5;
            }

            robot.frontLeft.setPower(flPower);
            robot.backLeft.setPower(blPower);
            robot.frontRight.setPower(frPower);
            robot.backRight.setPower(brPower);


            double launchRPM = ((-ry + 1) * (((double) 6000) / Robot.launchRatio) / 2); // calculates max motor speed and multiplies it by the float of the joystick y value

            if (isIntakePowered) robot.intake.setPower(1);
            else robot.intake.setPower(0);

            if (isLaunchPowered) robot.launch.setVelocity((launchRPM / 60) * Robot.TICKS_PER_REV);
            else {
                robot.launch.setPower(0);
                launchRPM = 0; // indicate that launch isn't powered
            }

            //intake.setPower(1); // permanently set intake to 100% BRRRRRRR
            //lowerTransfer.setPosition(ry);
            if (gamepad1.y) {
                robot.lowerTransfer.setPosition(Robot.lowerTransferUpperLimit);
            } else {
                robot.lowerTransfer.setPosition(Robot.lowerTransferLowerLimit);
            }

            telemetry.addData("odo status", robot.odo.getDeviceStatus());
            telemetry.addData("desired launch RPM", launchRPM);
            telemetry.addData("desired launch TPS", (launchRPM / 60) * Robot.TICKS_PER_REV);
            telemetry.addData("launch RPM", (robot.launch.getVelocity() / Robot.TICKS_PER_REV ) * 60); // convert from ticks/sec to rev/min
            telemetry.addData("launch velocity", robot.launch.getVelocity());
            telemetry.addData("launch current", robot.getLaunchCurrent()); // display current
            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.addData("ry", ry);
            telemetry.addData("lowerTransfer", robot.lowerTransfer.getPosition());
            telemetry.update();

            idle();
        }

    }
}
