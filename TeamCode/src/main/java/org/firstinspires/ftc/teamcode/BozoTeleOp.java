package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.hardware.IMU; // import IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // import IMU orientation

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles; // import angles for IMU orientation
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; // import angle units for easy conversion

@TeleOp(name="BozoTeleOp", group="TeleOp")
public class BozoTeleOp extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    private DcMotorEx launch; // we need current monitoring and PIDF control

    public static final double launchP = 2.5;
    public static final double launchI = 0.1;
    public static final double launchD = 0.2;
    public static final double launchF = 0.5;

    public static final double launchRatio = 1; // this is correct because 5202-0002-0001's gearbox ratio is 1:1, but if we change to any other motor, we need to update this
    //public int launchRPM = 3000; // max RPM with 5202-0002-0001 is 6000, so this should be good for tuning


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        intake = hardwareMap.dcMotor.get("intake");
        launch = (DcMotorEx) hardwareMap.dcMotor.get("launch");

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
        intake.setDirection(DcMotorSimple.Direction.FORWARD); // will prob need to be changed
        launch.setDirection(DcMotorSimple.Direction.FORWARD); // will prob need to be changed

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder

        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        PIDFCoefficients pidfOrig = launch.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(launchP, launchI, launchD, launchF);
        launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);

        waitForStart();

        while (opModeIsActive()) {
            // Read raw joystick inputs
            double y = -gamepad1.left_stick_y; // forward
            double x = gamepad1.left_stick_x * 1.1; // strafe
            double rx = gamepad1.right_stick_x; // rotation
            double ry = gamepad1.right_stick_y; // launch power (temporary until algorithm)

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
            intake.setPower(1); // permanently set intake to 100% BRRRRRRR

            int launchRPM = (int) (ry * 600); // TODO: fix this to adapt to diff gear ratios
            double launchTPS = ((double) launchRPM / 60) * launchRatio; // calculate the desired TPS (ticks per second) of our launch motor
            launch.setVelocity(launchTPS);
            telemetry.addData("desired launch RPM", "%.04f", launchTPS * 60);
            telemetry.addData("launch RPM", "%.04f", launch.getVelocity() * 60);

            telemetry.addData("Heading (rad)", botHeading);
            telemetry.update();

            idle();
        }

    }
}
