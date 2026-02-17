/** this OpMode provides several modes to test and tune servos **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.launch.Transfer;
import org.firstinspires.ftc.teamcode.subsys.launch.Hood;

@TeleOp(name="ServoTuner", group="Tuner")
public class ServoTuner extends OpMode {

    private enum TuneMode { // order matters for display
        LOWER_TRANSFER,
        UPPER_TRANSFER,
        HOOD_ABSOLUTE,
        HOOD_RADIANS,
        UNSELECTED
    }

    private TuneMode mode = TuneMode.UNSELECTED;

    private Hood hood;
    private Transfer transfer;

    private TelemetryManager telemetryM;

    @Override
    public void init() {
        hood = new Hood(hardwareMap);
        transfer = new Transfer(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        telemetryM.addLine("init complete");

        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];
        TuneMode mode2 = TuneMode.values()[2];
        TuneMode mode3 = TuneMode.values()[3];

        telemetryM.addLine("select your tuning mode using the gamepad:");
        // display servo test modes in order of TestMode enum
        telemetryM.addLine("(A): " + mode0);
        telemetryM.addLine("(B): " + mode1);
        telemetryM.addLine("(X): " + mode2);
        //telemetryM.addLine("(Y): " + mode3);

        if (gamepad1.aWasReleased()) { mode = mode0; }
        if (gamepad1.bWasReleased()) { mode = mode1; }
        if (gamepad1.xWasReleased()) { mode = mode2; }
        if (gamepad1.yWasReleased()) { mode = mode3; }

        telemetryM.addLine("current testing mode is: " + mode);

        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        double stickAmount = (gamepad1.right_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)

        telemetryM.addLine("current tuning mode is: " + mode);
        telemetryM.addLine("you may change the tuning mode by restarting this OpMode");
        telemetryM.addLine("use the right stick to tune positions");

        switch (mode) {
            case LOWER_TRANSFER:
                transfer.setRawLower(stickAmount);

                telemetryM.addData("lower transfer pos", stickAmount);
                break;
            case UPPER_TRANSFER:
                transfer.setRawUpper(stickAmount);

                telemetryM.addData("lower transfer pos", stickAmount);
                break;
            case HOOD_ABSOLUTE:
                /*telemetryM.addLine("this tuner will still map your stick input between:");
                telemetryM.addLine("Tunables.hoodMinimum: " + Tunables.hoodMinimum);
                telemetryM.addLine("Tunables.hoodMaximum" + Tunables.hoodMaximum);
                telemetryM.addLine("both of which can be adjusted with Panels"); */
                telemetryM.addLine("setting raw hood position");

                hood.setRawPos(stickAmount);
                telemetryM.addLine("");
                telemetryM.addData("hood position (absolute)", hood.getAbsolutePos());
                break;
            case HOOD_RADIANS:
                hood.setPos(Range.scale(stickAmount, 0, 1, Tunables.hoodMinimumPos, Tunables.hoodMaximumPos));

                telemetryM.addLine("hood set to: " + hood.getRadians() + "rad");
                telemetryM.addData("degrees", Math.toDegrees(hood.getRadians()));
                break;
            case UNSELECTED:
                throw new RuntimeException("make sure to select a tuning mode before starting!");
        }

        telemetryM.update(telemetry);
    }
}