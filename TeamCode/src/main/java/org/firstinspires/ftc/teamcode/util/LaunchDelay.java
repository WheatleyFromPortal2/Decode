/** this OpMode is used to tune launch times **/
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="LaunchDelay", group="Util")
public class LaunchDelay extends LinearOpMode {
    private Robot robot;
    private boolean intakeOn = true;

    // let's start with our delays from Robot.java
    private int openDelay = Robot.openDelay; // time to wait for upperTransfer to open (in millis)
    private int pushDelay = Robot.pushDelay; // time to wait for lowerTransfer to move (in millis)
    private int firstInterLaunchWait = Robot.firstInterLaunchWait; // check Robot.java
    private int lastInterLaunchWait = Robot.lastInterLaunchWait; // check Robot.java

    private final static int  manualChangeAmount = 10; // amount to increment/decrement when d-pad is pressed

    @Override
    public void runOpMode() throws InterruptedException {
        robot = Robot.getInstance(hardwareMap);


        waitForStart();
        while (opModeIsActive()) {
            robot.setLaunchVelocity(robot.RPMToTPS(2400)); // RPM doesn't really matter

            if (gamepad1.aWasReleased()) intakeOn = !intakeOn; // toggle intake
            if (intakeOn) robot.intake.setPower(1);
            else robot.intake.setPower(0);

            if (gamepad1.rightBumperWasReleased()) launchBall(); // launch the ball
            if (gamepad1.yWasReleased()) launch3Balls(); // launch 3 balls

            if (gamepad1.dpadUpWasReleased()) openDelay += manualChangeAmount; // increment openDelay
            if (gamepad1.dpadDownWasReleased()) openDelay -= manualChangeAmount; // decrement openDelay

            if (gamepad1.dpadRightWasReleased()) pushDelay += manualChangeAmount; // increment pushDelay
            if (gamepad1.dpadLeftWasReleased()) pushDelay -= manualChangeAmount; // decrement pushDelay

            if (gamepad1.xWasReleased()) firstInterLaunchWait += manualChangeAmount; // increment interLaunchWait
            if (gamepad1.bWasReleased()) firstInterLaunchWait -= manualChangeAmount; // decrement interLaunchWait


            telemetry.addLine("use d-pad up/down to modify openDelay (upper transfer opening)");
            telemetry.addLine("use d-pad left/right to modify pushDelay (lower transfer pushing");
            telemetry.addLine("use X/B to modify interLaunchWait (time between ball launches for macro)");

            telemetry.addData("openDelay (millis)", openDelay);
            telemetry.addData("pushDelay (millis)", pushDelay);
            telemetry.addData("firstInterLaunchWait (millis)", firstInterLaunchWait);
            telemetry.addData("lastInterLaunchWait (millis)", lastInterLaunchWait);

            telemetry.addData("1 ball launch time (millis)", openDelay + pushDelay); // calc time to shoot 1 ball
            telemetry.addData("3 ball launch time (millis)", (openDelay + pushDelay) * 3 + firstInterLaunchWait * 2); // calc time to shoot 3 balls
            telemetry.addData("launch RPM", robot.getLaunchRPM());

            telemetry.update();
            idle();
        }
    }
    private void launchBall() throws InterruptedException { // launch a ball
        // TODO: make this asynchronous (eliminate all the waits)
        robot.upperTransfer.setPosition(Robot.upperTransferOpen);
        sleep(openDelay); // allow time for upper transfer to move
        robot.lowerTransfer.setPosition(Robot.lowerTransferUpperLimit);
        sleep(pushDelay); // allow time for lower transfer to move
        // hopefully the ball has launched by now
        robot.upperTransfer.setPosition(Robot.upperTransferClosed); // close upper transfer
        robot.lowerTransfer.setPosition(Robot.lowerTransferLowerLimit); // set lower transfer to its lowest
    }

    private void launch3Balls() throws InterruptedException {  // launch 3 balls in succession
        launchBall(); // launch our first ball
        sleep(firstInterLaunchWait); // could rework this to also watch for velocity
        launchBall(); // launch our second ball
        sleep(lastInterLaunchWait);
        launchBall(); // launch our third ball
    }
}
