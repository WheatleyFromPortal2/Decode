/** this OpMode is used to test/tune the turret **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.launch.Turret;

@TeleOp(name="TurretTuner", group="Tuner")
public class TurretTuner extends OpMode {
    private int MIN_MOVE_TIME = 100; // minimum millis to wait for turret to move

    private enum TuneMode { // order matters for display
        CALIBRATE,
        TEST,
        UNSELECTED
    }

    private enum CalibrationState {
        START,
        ZERO_CENTER, // zero before measuring center
        MOVE_LEFT_MAX,
        RECENTER,
        MOVE_RIGHT_MAX,
        RESET,
        END
    }

    private Turret turret;
    private TelemetryManager telemetryM;

    /** only vars that should change **/

    private TuneMode mode = TuneMode.UNSELECTED;
    private CalibrationState calibrationState = CalibrationState.START;
    private Timer moveTimer = new Timer();

    /** end vars that change **/

    @Override
    public void init() {
        turret = new Turret(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        telemetryM.addLine("init complete");

        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];

        telemetryM.addLine("select your tuning mode using the gamepad:");
        // display servo test modes in order of TestMode enum
        telemetryM.addLine("(A): " + mode0);
        telemetryM.addLine("(B): " + mode1);

        if (gamepad1.aWasReleased()) { mode = mode0; }
        if (gamepad1.bWasReleased()) { mode = mode1; }

        telemetryM.addLine("");
        telemetryM.addLine("current testing mode is: " + mode);

        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        telemetryM.addLine("current tuning mode is: " + mode);
        telemetryM.addLine("you may change the tuning mode by restarting this OpMode");
        telemetryM.addLine("");

        switch (mode) {
            case CALIBRATE:
                telemetryM.addData("calibration state", calibrationState);
                switch (calibrationState) {
                    case START:
                        // turret should already be off
                        telemetryM.addLine("move the turret to exactly forward, and then press (A) to calibrate center");
                        if (gamepad1.aWasReleased()) {
                            turret.zero();
                            calibrationState = CalibrationState.ZERO_CENTER;
                        }
                        break;
                    case ZERO_CENTER:
                        Tunables.turretCenterOffset = turret.getPos();

                        // test turret left max
                        turret.setRawServoPositions(1);
                        startMoving();
                        calibrationState = CalibrationState.MOVE_LEFT_MAX;

                        break;
                    case MOVE_LEFT_MAX:
                        telemetryM.addLine("calibrating turret left max range...");
                        if (isTurretStopped()) { // wait for turret to stop moving
                            Tunables.turretMaxLeft = turret.getPos() - Tunables.turretCenterOffset; // account for offset

                            // recenter
                            turret.setRawServoPositions(0.5);
                            startMoving();
                            calibrationState = CalibrationState.RECENTER;
                        }
                        break;
                    case RECENTER:
                        telemetryM.addLine("centering turret");
                        if (isTurretStopped()) {
                            // test turret left max
                            turret.setRawServoPositions(0);
                            startMoving();
                            calibrationState = CalibrationState.MOVE_RIGHT_MAX;
                        }
                        break;
                    case MOVE_RIGHT_MAX:
                        telemetryM.addLine("calibrating turret right max range");
                        if (isTurretStopped()) { // wait for turret to stop moving
                            Tunables.turretMaxRight = Tunables.turretCenterOffset - turret.getPos();

                            calibrationState = CalibrationState.RESET;
                        }
                        break;
                    case RESET:
                        turret.setDesiredPos(0); // reset to normal, now perfectly centered position
                        //turret.on();
                        turret.update();
                        calibrationState = CalibrationState.END;
                        break;
                    case END:
                        telemetryM.addLine("turret calibration complete!");
                        telemetryM.addData("turret center offset", Tunables.turretCenterOffset);
                        telemetryM.addData("turret max left", Tunables.turretMaxLeft);
                        telemetryM.addData("turret max right", Tunables.turretMaxRight);

                        telemetryM.addLine("");
                        telemetryM.addLine("if these values are reliable after testing, then save them in Tunables.java!");
                        testTurret();
                        break;
                }
                break;
            case TEST:
                testTurret();
                break;
            case UNSELECTED:
                throw new RuntimeException("make sure to select a tuning mode before starting!");
        }

        telemetryM.addLine("");
        telemetryM.addData("turret position", turret.getPos());
        telemetryM.addData("turret velocity", turret.getVelocity());

        telemetryM.addData("turret offset", Tunables.turretCenterOffset);

        telemetryM.update(telemetry);
    }


    private void testTurret() { // test turret movement using gamepad
        telemetryM.addLine("use the right stick to tune positions");
        telemetryM.addLine("you may also hold down the dpad to move the turret");

        double inputPos = Range.scale(gamepad1.right_stick_y, -1, 1, -2 * Math.PI, 2 * Math.PI);

        if (gamepad1.dpad_down) inputPos = Math.toRadians(180);
        if (gamepad1.dpad_left) inputPos = Math.toRadians(-90);
        if (gamepad1.dpad_up) inputPos = Math.toRadians(0);
        if (gamepad1.dpad_right) inputPos = Math.toRadians(90);

        turret.setDesiredPos(inputPos);

        telemetryM.addData("desired turret position", turret.getDesiredPos());

        turret.update();
    }

    private void startMoving() {
        moveTimer.resetTimer();

    }
    private boolean isTurretStopped() {
        if (moveTimer.getElapsedTime() <= MIN_MOVE_TIME) {
            return false; // wait for turret to start moving
        } else if (moveTimer.getElapsedTime() >= MIN_MOVE_TIME) {
            return turret.getVelocity() == 0; // wait for turret to  stop moving
        } else {
            return false; // still waiting for turret to stop moving
        }
    }
}
