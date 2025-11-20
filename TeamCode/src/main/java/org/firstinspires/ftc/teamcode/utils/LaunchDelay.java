// used to calibrate launch times
package org.firstinspires.ftc.teamcode.utils;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="LaunchDelay", group="Util")
public class LaunchDelay extends LinearOpMode {
    Robot robot;

    // let's start with our delays from Robot.java
    private int openDelay = Robot.openDelay; // time to wait for upperTransfer to open (in millis)
    private int pushDelay = Robot.pushDelay; // time to wait for lowerTransfer to move (in millis)
    private int interLaunchWait = Robot.interLaunchWait; // time to wait between launches (in millis)

    private final static int  manualChangeAmount = 10; // amount to increment/decrement when d-pad is pressed

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = Robot.getInstance(hardwareMap);

        telemetry.addLine("use d-pad up/down to modify openDelay (upper transfer opening)");
        telemetry.addLine("use d-pad left/right to modify pushDelay (lower transfer pushing");
        telemetry.addLine("use Y/A to modify interLaunchWait (time between ball launches for macro)");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.rightBumperWasReleased()) launchBall(); // launch the ball

            if (gamepad1.dpadUpWasReleased()) openDelay += manualChangeAmount; // increment openDelay
            if (gamepad1.dpadDownWasReleased()) openDelay -= manualChangeAmount; // decrement openDelay

            if (gamepad1.dpadRightWasReleased()) pushDelay += manualChangeAmount; // increment pushDelay
            if (gamepad1.dpadLeftWasReleased()) pushDelay -= manualChangeAmount; // decrement pushDelay

            if (gamepad1.yWasReleased()) interLaunchWait += manualChangeAmount; // increment interLaunchWait
            if (gamepad1.aWasReleased()) interLaunchWait -= manualChangeAmount; // decrement interLaunchWait

            telemetry.addData("openDelay (millis)", openDelay);
            telemetry.addData("pushDelay (millis)", pushDelay);
            telemetry.addData("interLaunchWait (millis)", interLaunchWait);

            telemetry.addData("1 ball launch time (millis)", openDelay + pushDelay); // calc time to shoot 1 ball
            telemetry.addData("3 ball launch time (millis)", (openDelay + pushDelay) * 3 + interLaunchWait * 2); // calc time to shoot 3 balls

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
        robot.launchBall(); // launch our first ball
        sleep(interLaunchWait); // could rework this to also watch for velocity
        robot.launchBall(); // launch our second ball
        sleep(interLaunchWait);
        robot.launchBall(); // launch our third ball
        sleep(interLaunchWait); // make sure ball has fully exited robot
    }
}
