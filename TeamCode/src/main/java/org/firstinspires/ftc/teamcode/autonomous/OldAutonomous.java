package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.odometry.odometry;

@Autonomous(name = "AutoDriveToGoal", group = "Competition")
public class OldAutonomous extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight, verticalLeft, verticalRight, horizontal;
    odometry globalPositionUpdate;

    static final double COUNTS_PER_INCH = 637.1;
    static final double TARGET_X = 120.0;
    static final double TARGET_Y = 18.0;

    static final double ANGLE = 45.0;
    static final double DRIVE_SPEED = 0.25;
    static final double ALLOWED_ERROR = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        verticalLeft = hardwareMap.dcMotor.get("verticalLeft");
        verticalRight = hardwareMap.dcMotor.get("verticalRight");
        horizontal = hardwareMap.dcMotor.get("horizontal");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        globalPositionUpdate = new odometry(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseRightEncoder();

        waitForStart();

        goToPosition(TARGET_X, TARGET_Y, DRIVE_SPEED);

        stopAllMotors();
        globalPositionUpdate.stop();
    }

    public void goToPosition(double targetX, double targetY, double speed) {
        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        while (opModeIsActive() && distance > ALLOWED_ERROR) {
            double x = targetX - globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
            double y = targetY - globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;
            double angle = Math.atan2(y, x);
            double powerY = Math.cos(angle);
            double powerX = Math.sin(angle);
            frontLeft.setPower((powerY + powerX) * speed);
            frontRight.setPower((powerY - powerX) * speed);
            backLeft.setPower((powerY - powerX) * speed);
            backRight.setPower((powerY + powerX) * speed);
            distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        }
    }

    public void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
