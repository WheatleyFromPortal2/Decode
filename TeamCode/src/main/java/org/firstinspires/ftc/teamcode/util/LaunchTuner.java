/** this OpMode is used to tune delays for our launch state machine **/

package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

@TeleOp(name="LaunchDelay", group="Util")
public class LaunchTuner extends LinearOpMode {
    private boolean intakeOn = true;

    private final static int  manualChangeAmount = 10; // amount of millis to increment/decrement when d-pad is pressed

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance(hardwareMap);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        waitForStart();
        while (opModeIsActive()) {
            robot.calcPIDF();
            robot.setLaunchVelocity(robot.RPMToTPS(2400)); // RPM doesn't really matter

            if (gamepad1.aWasReleased()) intakeOn = !intakeOn; // toggle intake
            if (intakeOn) robot.intake.setPower(1);
            else robot.intake.setPower(0);

            if (gamepad1.rightBumperWasReleased()) robot.launchBalls(1); // launch 1 ball
            if (gamepad1.yWasReleased()) robot.launchBalls(3); // launch 3 balls

            if (gamepad1.dpadUpWasReleased()) Tunables.openDelay += manualChangeAmount; // increment openDelay
            if (gamepad1.dpadDownWasReleased()) Tunables.openDelay -= manualChangeAmount; // decrement openDelay

            if (gamepad1.dpadRightWasReleased()) Tunables.maxPushDelay += manualChangeAmount; // increment pushDelay
            if (gamepad1.dpadLeftWasReleased()) Tunables.maxPushDelay -= manualChangeAmount; // decrement pushDelay

            if (gamepad1.xWasReleased()) Tunables.maxTransferDelay += manualChangeAmount; // increment interLaunchWait
            if (gamepad1.bWasReleased()) Tunables.maxTransferDelay -= manualChangeAmount; // decrement interLaunchWait


            telemetryM.addLine("use d-pad up/down to modify openDelay (upper transfer opening)");
            telemetryM.addLine("use d-pad left/right to modify pushDelay (lower transfer pushing");
            telemetryM.addLine("use X/B to modify interLaunchWait (time between ball launches for macro)");

            telemetryM.addData("openDelay (millis)", Tunables.openDelay);
            telemetryM.addData("maxTransferDelay (millis)", Tunables.maxTransferDelay);
            telemetryM.addData("maxPushDelay (millis)", Tunables.maxPushDelay);
            telemetryM.addData("last launch interval", robot.getLastLaunchInterval());
            telemetryM.addData("desired launch RPM", robot.getDesiredLaunchRPM());
            telemetryM.addData("launch RPM", robot.getLaunchRPM());

            telemetryM.addData("launching?: ", robot.updateLaunch()); // update what's happening in our launch and send it to driver
            telemetryM.update();
            idle();
        }
    }
}
