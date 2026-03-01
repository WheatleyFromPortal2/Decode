/** this OpMode is used to test/tune the turret **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.launch.Intake;
import org.firstinspires.ftc.teamcode.subsys.launch.Turret;

@TeleOp(name="TurretTuner", group="Tuner")
public class TurretTuner extends OpMode {
    private int START_MOVE_MILLIS = 100; // minimum millis to wait for turret to move
    private int SETTLE_MILLIS = 2000; // time to wait for turret to settle

    private enum TuneMode { // order matters for display
        CALIBRATE,
        TEST,
        TEST_RAW,
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
    private boolean startedMoving = false;

    /** end vars that change **/

    @Override
    public void init() {
        Intake intake = new Intake(hardwareMap); // need to do this to get encoder
        turret = new Turret(hardwareMap, intake.getMotor());

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        telemetryM.addLine("init complete");

        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];
        TuneMode mode2 = TuneMode.values()[2];

        telemetryM.addLine("select your tuning mode using the gamepad:");

        telemetryM.addLine("");
        telemetryM.addLine("to re-center and re-mesh turret servos:");
        telemetryM.addLine("select " + TuneMode.TEST_RAW + " and leave the sticks untouched while keeping the Opmode running");

        telemetryM.addLine("");
        // display servo test modes in order of TestMode enum
        telemetryM.addLine("(A): " + mode0);
        telemetryM.addLine("(B): " + mode1);
        telemetryM.addLine("(X): " + mode2);

        if (gamepad1.aWasReleased()) { mode = mode0; }
        if (gamepad1.bWasReleased()) { mode = mode1; }
        if (gamepad1.xWasReleased()) { mode = mode2; }

        telemetryM.addLine("");
        telemetryM.addLine("current testing mode is: " + mode);

        telemetryM.addData("turret max left", Tunables.turretMaxLeft);
        telemetryM.addData("turret max right", Tunables.turretMaxRight);

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
                            turret.setRawServoPositions(0.5);
                            startMoving();
                            calibrationState = CalibrationState.ZERO_CENTER;
                        }
                        break;
                    case ZERO_CENTER:
                        if (isTurretStopped()) {

                            // test turret left max
                            turret.setRawServoPositions(Tunables.turretLimitLeft);
                            startMoving();
                            calibrationState = CalibrationState.MOVE_LEFT_MAX;
                        }
                        break;
                    case MOVE_LEFT_MAX:
                        telemetryM.addLine("calibrating turret left max range...");
                        if (isTurretStopped()) { // wait for turret to stop moving
                            Tunables.turretMaxLeft = turret.getPos();

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
                            turret.setRawServoPositions(Tunables.turretLimitRight);
                            startMoving();
                            calibrationState = CalibrationState.MOVE_RIGHT_MAX;
                        }
                        break;
                    case MOVE_RIGHT_MAX:
                        telemetryM.addLine("calibrating turret right max range");
                        if (isTurretStopped()) { // wait for turret to stop moving
                            Tunables.turretMaxRight = turret.getPos();

                            calibrationState = CalibrationState.RESET;
                        }
                        break;
                    case RESET:
                        turret.setRawServoPositions(0.5);
                        //turret.setDesiredPos(0); // reset to normal, now perfectly centered position
                        //turret.on();
                        //turret.update();
                        calibrationState = CalibrationState.END;
                        break;
                    case END:
                        telemetryM.addLine("turret calibration complete!");

                        telemetryM.addLine("");
                        telemetryM.addLine("if these values are reliable after testing, then save them in Tunables.java!");
                        break;
                }
                break;
            case TEST:
                testTurret();
                break;
            case TEST_RAW:
                double rawPos = Range.scale(gamepad1.right_stick_x, -1.0, 1.0, 0.0, 1.0);

                turret.setRawServoPositions(rawPos);

                telemetryM.addData("raw pos", rawPos);
                break;
            case UNSELECTED:
                throw new RuntimeException("make sure to select a tuning mode before starting!");
        }

        telemetryM.addLine("");
        telemetryM.addData("turret position", turret.getPos());
        telemetryM.addData("turret velocity", turret.getVelocity());

        telemetryM.addData("turret max left", Tunables.turretMaxLeft);
        telemetryM.addData("turret max right", Tunables.turretMaxRight);

        telemetryM.update(telemetry);
    }


    private void testTurret() { // test turret movement using gamepad
        telemetryM.addLine("use the right stick to tune positions");
        telemetryM.addLine("you may also hold down the dpad to move the turret");

        double inputPos = Range.scale(gamepad1.right_stick_y, -1.0, 1.0, Tunables.turretMaxLeft, Tunables.turretMaxRight);

        if (gamepad1.dpad_down) inputPos = Math.toRadians(180);
        if (gamepad1.dpad_left) inputPos = Math.toRadians(-90);
        if (gamepad1.dpad_up) inputPos = Math.toRadians(0);
        if (gamepad1.dpad_right) inputPos = Math.toRadians(90);

        turret.update();

        if (gamepad1.aWasReleased()) turret.zero();

        turret.setDesiredPos(inputPos);

        telemetryM.addData("desired turret position", turret.getDesiredPos());
        telemetryM.addData("turret error (deg", Math.toDegrees(turret.getPos() - inputPos));
    }

    private void startMoving() {
        moveTimer.resetTimer();
        startedMoving = false;
    }
    private boolean isTurretStopped() {
        if (!startedMoving) {
            if (moveTimer.getElapsedTime() <= START_MOVE_MILLIS) {
                return false; // wait for turret to start moving
            } else if (turret.getVelocity() == 0) {
                startedMoving = true;
                moveTimer.resetTimer();
            }
        } else {
            if (moveTimer.getElapsedTime() <= SETTLE_MILLIS || turret.getVelocity() != 0) {
                return false;
            } else {
                return true;
            }
        }
        return false;
    }
}
