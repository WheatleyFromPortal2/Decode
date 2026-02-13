/** the purpose of OpMode is to test flywheel max speed and tune flywheel PIDF **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;
import org.firstinspires.ftc.teamcode.subsys.launch.Flywheel;

@TeleOp(name="FlywheelTuner", group="Tuner")
public class FlywheelTuner extends LinearOpMode {
    private enum TuneMode {
        NO_POWER,
        FULL_POWER,
        PIDF
    }

    private double MAX_SET_RPM = 6000;

    @Override
    public void runOpMode() {
        Flywheel flywheel = new Flywheel(hardwareMap);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        TuneMode mode = TuneMode.NO_POWER;

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];
        TuneMode mode2 = TuneMode.values()[2];

        waitForStart();

        while (opModeIsActive()) {
            telemetryM.addLine("select your tuning mode using the gamepad:");
            telemetryM.addLine("(A): " + mode0);
            telemetryM.addLine("(B): " + mode1);
            telemetryM.addLine("(X): " + mode2);

            if (gamepad1.aWasReleased()) { mode = mode0; }
            if (gamepad1.bWasReleased()) { mode = mode1; }
            if (gamepad1.xWasReleased()) { mode = mode2; }

            telemetryM.addLine("");
            telemetryM.addLine("current testing mode is: " + mode);

            switch (mode) {
                case NO_POWER:
                    flywheel.powerUpdate(0.0); // apply no power
                    break;
                case FULL_POWER:
                    flywheel.powerUpdate(1.0);
                    break;
                case PIDF:
                    LaunchSetpoints setState = new LaunchSetpoints(0, 0, 0); // use LaunchState for conversion

                    double desiredFlywheelRPM = (MAX_SET_RPM * gamepad1.right_trigger);

                    setState.setRPM(desiredFlywheelRPM);
                    flywheel.setpointUpdate(setState.getTPS());

                    telemetryM.addData("desired flywheel RPM", desiredFlywheelRPM);
                    telemetryM.addData("desired flywheel TPS", setState.getTPS());
                    break;
            }

            LaunchSetpoints currentState = new LaunchSetpoints(flywheel.getTPS(), 0, 0); // use LaunchState for conversion

            telemetryM.addData("flywheel RPM", currentState.getRPM());
            telemetryM.addData("flywheel TPS", currentState.getTPS());

            telemetryM.debug("PIDF: " + Tunables.flywheelP + ", " + Tunables.flywheelI + ", " + Tunables.flywheelD + ", " + Tunables.flywheelF);
            telemetryM.addData("flywheel power", flywheel.getPower());
            telemetryM.addData("flywheel current", flywheel.getCurrent());

            telemetryM.update(telemetry); // update our telemetry

            idle();
        }
    }
}