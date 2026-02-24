/** this OpMode provides several modes to test and tune servos **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.Indexer;
import org.firstinspires.ftc.teamcode.subsys.launch.Transfer;
import org.firstinspires.ftc.teamcode.subsys.launch.Hood;

@TeleOp(name="ServoTuner", group="Tuner")
public class ServoTuner extends OpMode {

    private enum TuneMode { // order matters for display
        TRANSFER,
        INDEXER,
        HOOD_ABSOLUTE,
        HOOD_RADIANS,
        UNSELECTED
    }

    private TuneMode mode = TuneMode.UNSELECTED;

    private Hood hood;
    private Transfer transfer;
    private Indexer indexer;

    private TelemetryManager telemetryM;

    @Override
    public void init() {
        hood = new Hood(hardwareMap);
        transfer = new Transfer(hardwareMap);
        indexer = new Indexer(hardwareMap);

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
        TuneMode mode4 = TuneMode.values()[4];

        telemetryM.addLine("select your tuning mode using the gamepad:");
        // display servo test modes in order of TestMode enum
        telemetryM.addLine("(A): " + mode0);
        telemetryM.addLine("(B): " + mode1);
        telemetryM.addLine("(X): " + mode2);
        telemetryM.addLine("(Y): " + mode3);
        telemetryM.addLine("(start): " + mode4);

        if (gamepad1.aWasReleased()) { mode = mode0; }
        if (gamepad1.bWasReleased()) { mode = mode1; }
        if (gamepad1.xWasReleased()) { mode = mode2; }
        if (gamepad1.yWasReleased()) { mode = mode3; }
        if (gamepad1.startWasReleased()) { mode = mode4; }

        telemetryM.addLine("current testing mode is: " + mode);

        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        double rightStickAmount = (gamepad1.right_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)
        double leftStickAmount = (gamepad1.left_stick_y+ 1) / 2; // map from (-1)<->(1) to (0)<->(1)

        telemetryM.addLine("current tuning mode is: " + mode);
        telemetryM.addLine("you may change the tuning mode by restarting this OpMode");

        switch (mode) {
            case TRANSFER:
                telemetryM.addLine("use the right stick to tune transfer positions");
                transfer.setServoRaw(rightStickAmount);

                telemetryM.addData("transfer pos", rightStickAmount);
                break;
            case INDEXER:
                telemetryM.addLine("use the left stick to tune indexer kicker positions");
                indexer.setKickerPosFast(rightStickAmount);
                telemetryM.addData("kicker pos", rightStickAmount);
                telemetryM.addLine(""); // padding
                telemetryM.addLine("use the right stick to tune indexer blocker positions");
                indexer.setBlockerPosFast(leftStickAmount);
                telemetryM.addData("blocker pos", leftStickAmount);

                break;
            case HOOD_ABSOLUTE:
                telemetryM.addLine("use the right stick to tune hood raw positions");
                /*telemetryM.addLine("this tuner will still map your stick input between:");
                telemetryM.addLine("Tunables.hoodMinimum: " + Tunables.hoodMinimum);
                telemetryM.addLine("Tunables.hoodMaximum" + Tunables.hoodMaximum);
                telemetryM.addLine("both of which can be adjusted with Panels"); */

                hood.setRawPos(rightStickAmount);
                telemetryM.addLine("");
                telemetryM.addData("hood position (absolute)", hood.getAbsolutePos());
                break;
            case HOOD_RADIANS:
                telemetryM.addLine("use the right stick to tune radian hood position");
                hood.setPos(Range.scale(rightStickAmount, 0, 1, Tunables.hoodMinimumPos, Tunables.hoodMaximumPos));

                telemetryM.addLine("hood set to: " + hood.getRadians() + "rad");
                telemetryM.addData("degrees", Math.toDegrees(hood.getRadians()));
                break;
            case UNSELECTED:
                throw new RuntimeException("make sure to select a tuning mode before starting!");
        }

        telemetryM.update(telemetry);
    }
}