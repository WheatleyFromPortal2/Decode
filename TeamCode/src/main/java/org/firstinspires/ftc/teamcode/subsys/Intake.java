package org.firstinspires.ftc.teamcode.subsys;

import android.health.connect.datatypes.units.Power;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Tunables;

import com.pedropathing.util.Timer;

public class Intake {
    private enum State {
        OFF, // intake completely unpowered
        ON, // intake full power
        POWER_SAVE, // saving intake power
        HOLD, // intake partially powered, just enough to keep balls in but save power
        REVERSE, // intake full power reverse
    }

    public enum PowerSaveTrigger {
        CURRENT,
        VELOCITY,
        NOT_TRIGGERED
    }

    private State state = State.OFF; // start unpowered

    private Timer powerSaveWaitTimer;
    private Timer powerSaveCheckTimer;
    private PowerSaveTrigger powerSaveTrigger = PowerSaveTrigger.NOT_TRIGGERED;
    double lastIntakeHoldPower = Tunables.intakeHoldPower;

    private DcMotorEx motor;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "intake");

        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder

        powerSaveWaitTimer = new Timer();
        powerSaveCheckTimer = new Timer();
    }

    public void update() { // need to continuously update for power save function and caching
        switch (state) {
            case OFF:
                break;
            case ON:
                if (Tunables.intakeUsePowerSave) {
                    if (powerSaveCheckTimer.getElapsedTime() >= Tunables.intakePowerSaveCheckInterval) {
                        // now we check which of our power save triggers hit
                        // we should ideally do this in order of what reads are the quickest / most accurate
                        if (isCurrentTrigger()) {
                            powerSaveTrigger = PowerSaveTrigger.CURRENT;
                            enablePowerSave();
                            return;
                        } else if (isVelocityTrigger()) {
                            powerSaveTrigger = PowerSaveTrigger.VELOCITY;
                            enablePowerSave();
                            return;
                        } else {
                            powerSaveTrigger = PowerSaveTrigger.NOT_TRIGGERED;
                            powerSaveCheckTimer.resetTimer(); // wait Tunables.intakePowerSaveUpdateInterval before next reading
                            return;
                        }
                    }
                    // else: don't waste loop time
                }
                break;
            case POWER_SAVE:
                // waiting with intake off and then after a while going back to on
                if (powerSaveWaitTimer.getElapsedTime() >= Tunables.intakePowerSaveWaitInterval) {
                    // we've waiting long enough, turn intake back on
                    on();
                }
                break;
            case HOLD:
                if (Tunables.intakeHoldPower != lastIntakeHoldPower) {
                    motor.setPower(Tunables.intakeHoldPower);
                    lastIntakeHoldPower = Tunables.intakeHoldPower;
                }
                break;
            case REVERSE:
                break;
        }
    }

    // we check states to prevent unnecessary writes

    public void off() {
        if (state != State.OFF) {
            motor.setPower(0);
            state = State.OFF;
        }
    }

    public void on() {
        if (state != State.ON) {
            motor.setPower(1);
            powerSaveTrigger = PowerSaveTrigger.NOT_TRIGGERED;
            state = State.ON;
            powerSaveCheckTimer.resetTimer();
        }
    }

    public void toggle() {
        switch (state) {
            case OFF:
                on();
            case ON:
                off();
            case REVERSE:
                on();
            case POWER_SAVE:
                on();
            case HOLD:
                on();
        }
    }

    public void hold() {
        if (state != State.HOLD) {
            motor.setPower(Tunables.intakeHoldPower);
            state = State.HOLD;
        }
    }

    public void reverse() {
        if (state != State.REVERSE) {
            motor.setPower(-1);
            state = State.REVERSE;
        }
    }

    public boolean isCurrentTrigger() { // whether our current amps is over our trigger amps
        return getCurrent() >= Tunables.intakePowerSaveTriggerAmps;
    }
    public boolean isVelocityTrigger() { // whether our current velocity is under our trigger velocity
        return getVelocity() <= Tunables.intakePowerSaveTriggerVelocity;
    }
    private void enablePowerSave() {
        state = State.POWER_SAVE;
        powerSaveWaitTimer.resetTimer();
        motor.setPower(0); // save power
    }

    /** getter methods **/
    public double getCurrent() { return motor.getCurrent(CurrentUnit.AMPS); } // return intake current in amps
    public double getVelocity() { return motor.getVelocity(); } // get velocity in tps
    public PowerSaveTrigger getPowerSaveTrigger() { return powerSaveTrigger; }
}
