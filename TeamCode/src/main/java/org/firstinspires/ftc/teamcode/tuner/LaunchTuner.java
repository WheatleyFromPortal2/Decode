/** this OpMode is used to tune delays for our launch state machine **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;

@TeleOp(name="LaunchTuner", group="Tuner")
public class LaunchTuner extends OpMode {
    private final static int  manualChangeAmount = 10; // amount of millis to increment/decrement when d-pad is pressed
    private LaunchSetpoints launchState = new LaunchSetpoints(0, 0, 0);
    private final double FLYWHEEL_RPM = 3000;

    private Robot robot;
    TelemetryManager telemetryM;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        launchState.setRPM(FLYWHEEL_RPM);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
    }

    @Override
    public void loop() {

        if (gamepad1.aWasReleased()) robot.intake.toggle();

        if (gamepad1.rightBumperWasReleased()) robot.launchBalls(1); // launch 1 ball
        if (gamepad1.yWasReleased()) robot.launchBalls(3); // launch 3 balls

        if (gamepad1.dpadUpWasReleased()) Tunables.openDelay += manualChangeAmount; // increment openDelay
        if (gamepad1.dpadDownWasReleased()) Tunables.openDelay -= manualChangeAmount; // decrement openDelay

        if (gamepad1.dpadRightWasReleased()) Tunables.extraPushDelay += manualChangeAmount; // increment pushDelay
        if (gamepad1.dpadLeftWasReleased()) Tunables.extraPushDelay -= manualChangeAmount; // decrement pushDelay

        if (gamepad1.xWasReleased()) Tunables.transferDelay -= manualChangeAmount; // increment interLaunchWait
        if (gamepad1.bWasReleased()) Tunables.transferDelay += manualChangeAmount; // decrement interLaunchWait

        if (gamepad1.backWasReleased()) Tunables.lastTransferDelay -= manualChangeAmount;
        if (gamepad1.startWasReleased()) Tunables.lastTransferDelay += manualChangeAmount;

        robot.update();

        telemetryM.addLine("use d-pad up/down to modify openDelay (upper transfer opening)");
        telemetryM.addLine("use d-pad left/right to modify extraPushDelay (lower transfer pushing");
        telemetryM.addLine("use X/B to modify transferDelay (time waiting for transfer)");
        telemetryM.addLine("use back/start to modify lastTransferDelay");

        telemetryM.addLine("---all values are in millis---");

        telemetryM.addData("openDelay", Tunables.openDelay);
        telemetryM.addData("extraPushDelay", Tunables.extraPushDelay);
        telemetryM.addData("maxPushDelay", Tunables.transferDelay);
        telemetryM.addData("lastTransferDelay", Tunables.lastTransferDelay);

        telemetryM.addData("last launch interval (s)", robot.getLastLaunchInterval());

        telemetryM.addData("launching?: ", robot.isLaunching()); // update what's happening in our launch and send it to driver
        telemetryM.update(telemetry);
    }
}
