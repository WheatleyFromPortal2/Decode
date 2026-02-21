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
    private LaunchSetpoints launchState = new LaunchSetpoints(0, 0, 0);
    private final double FLYWHEEL_RPM = 3000;

    private Robot robot;
    TelemetryManager telemetryM;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        launchState.setRPM(FLYWHEEL_RPM);

        robot.setSetpoints(launchState);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
    }

    @Override
    public void loop() {
        robot.intake.forward();

        if (gamepad1.aWasReleased()) robot.intake.toggle();

        if (gamepad1.rightBumperWasReleased()) robot.launchBalls(1); // launch 1 ball
        if (gamepad1.yWasReleased()) robot.launchBalls(3); // launch 3 balls

        robot.update();

        telemetryM.addData("last launch interval (s)", robot.getLastLaunchInterval());

        telemetryM.addData("launching?: ", robot.isLaunching()); // update what's happening in our launch and send it to driver
        telemetryM.addData("RPM", robot.flywheel.getRPM());
        telemetryM.update(telemetry);
    }
}
