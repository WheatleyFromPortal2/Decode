package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.odometry.odometry;

@TeleOp(name="BozoTeleOp", group="TeleOp")
public class BozoTeleOp extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor arm, intake, shooter;
    private Servo claw;

    // Optional: if you want odometry + IMU combo
    private DcMotor verticalLeft, verticalRight, horizontal;
    private odometry odo;
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        arm = hardwareMap.dcMotor.get("arm");
        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");
        claw = hardwareMap.servo.get("claw");

        // Odometry wheels (if you have them)
        verticalLeft = hardwareMap.dcMotor.get("verticalLeft");
        verticalRight = hardwareMap.dcMotor.get("verticalRight");
        horizontal = hardwareMap.dcMotor.get("horizontal");

        // IMU initialization
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // Drive motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start odometry thread (optional)
        odo = new odometry(verticalLeft, verticalRight, horizontal, 1000.0, 75);
        Thread odometryThread = new Thread(odo);
        odometryThread.start();

        waitForStart();

        double clawPos = 0.5;

        while (opModeIsActive()) {
            // Read raw joystick inputs
            double y = -gamepad1.left_stick_y; // forward
            double x = gamepad1.left_stick_x * 1.1; // strafe
            double rx = gamepad1.right_stick_x; // rotation

            // Read IMU heading (radians)
            double botHeading = imu.getAngularOrientation().firstAngle;

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

            // Arm control
            if (Math.abs(gamepad2.left_stick_y) > 0.05)
                arm.setPower(-gamepad2.left_stick_y * 0.6);
            else
                arm.setPower(0);

            // Intake control
            if (gamepad2.right_bumper)
                intake.setPower(1);
            else if (gamepad2.left_bumper)
                intake.setPower(-1);
            else
                intake.setPower(0);

            // Shooter control
            if (gamepad2.a)
                shooter.setPower(1);
            else if (gamepad2.b)
                shooter.setPower(0.6);
            else if (gamepad2.x)
                shooter.setPower(0);

            // Claw control
            if (gamepad1.a) clawPos = 1.0;
            if (gamepad1.b) clawPos = 0.0;
            if (gamepad1.y) clawPos = 0.5;
            claw.setPosition(clawPos);

            telemetry.addData("Heading (rad)", botHeading);
            telemetry.addData("Odometry X", odo.returnXCoordinate());
            telemetry.addData("Odometry Y", odo.returnYCoordinate());
            telemetry.update();

            idle();
        }

        odo.stop();
    }
}
