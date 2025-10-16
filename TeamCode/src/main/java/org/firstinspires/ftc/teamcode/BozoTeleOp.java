package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.IMU; // import IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // import IMU orientation

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles; // import angles for IMU orientation
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; // import angle units for easy conversion

@TeleOp(name="BozoTeleOp", group="TeleOp")
public class BozoTeleOp extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    //private DcMotor intake, launch;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        //intake = hardwareMap.dcMotor.get("intake");
        //launch = hardwareMap.dcMotor.get("launch");

        // IMU initialization
        IMU imu;
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters IMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // control hub sticker faces to the right
                        RevHubOrientationOnRobot.UsbFacingDirection.UP // control hub usb ports face up
                )
        );

        imu.initialize(IMUParameters); // initialize IMU with our params
        imu.resetYaw(); // set IMU yaw to 0deg

        // Drive motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time

        waitForStart();

        while (opModeIsActive()) {
            // Read raw joystick inputs
            double y = -gamepad1.left_stick_y; // forward
            double x = gamepad1.left_stick_x * 1.1; // strafe
            double rx = gamepad1.right_stick_x; // rotation

            // Read IMU heading (radians)
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double botHeading = orientation.getYaw(AngleUnit.RADIANS); //

            // Field-centric transform
            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1.0);
            double flPower = (rotatedY + rotatedX + rx) / denominator;
            double blPower = (rotatedY - rotatedX + rx) / denominator;
            double frPower = (rotatedY - rotatedX - rx) / denominator;
            double brPower = (rotatedY + rotatedX - rx) / denominator;

            // Slow mode
            if (gamepad1.left_bumper) {
                flPower *= 0.5;
                blPower *= 0.5;
                frPower *= 0.5;
                brPower *= 0.5;
            }

            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

            //intake.setPower(1);

            // launch motor control
            /*if (gamepad1.a)
                launch.setPower(1);
            else if (gamepad1.b)
                launch.setPower(0.6);
            else if (gamepad1.x)
                launch.setPower(0);*/

            telemetry.addData("Heading (rad)", botHeading);
            telemetry.update();

            idle();
        }

    }
}
