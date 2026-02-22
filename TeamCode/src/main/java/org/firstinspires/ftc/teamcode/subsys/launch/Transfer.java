package org.firstinspires.ftc.teamcode.subsys.launch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tunables;

public class Transfer {
    private enum State {
        OFF,
        FORWARD,
        REVERSE
    }

    private Servo servo;
    private DcMotorEx motor;
    private DigitalChannel lowerTransferSensor, upperTransferSensor;

    /** only variables that should change through run time **/

    private boolean isUpperOpen;
    private boolean wasBallInUpperTransfer = false; // was a ball in lower transfer last time
    private State state = State.OFF;

    /** end vars that change **/

    public Transfer(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "transferMotor");
        servo = hw.get(Servo.class, "transferServo");

        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        close();

        lowerTransferSensor = hw.get(DigitalChannel.class, "lowerTransferSensor");
        upperTransferSensor = hw.get(DigitalChannel.class, "upperTransferSensor");

        lowerTransferSensor.setMode(DigitalChannel.Mode.INPUT);
        upperTransferSensor.setMode(DigitalChannel.Mode.INPUT);

        reset();
    }

    /** getter methods **/
    public boolean isOff() {
        return state == State.OFF;
    }

    public boolean isForward() {
        return state == State.FORWARD;
    }

    public boolean isReverse() {
        return state == State.REVERSE;
    }

    public boolean isUpperOpen() {
        return isUpperOpen;
    }

    public boolean isBallInLower() { // return true if there is a ball reducing our measured distance
        return !lowerTransferSensor.getState(); // invert
        // if this sensor disconnects, it reports true (which is inverted to false), so the launch state machine will just fallback to the fixed delay
    }
    public boolean isBallInUpper() { // return true if there is a ball reducing our measured distance
        return !upperTransferSensor.getState(); // invert
        // if this sensor disconnects, it reports true (which is inverted to false), so the launch state machine will just fallback to the fixed delay
    }

    public boolean wasBallLaunched() {
        boolean ballInUpper = isBallInUpper();
        boolean launched;

        if (!wasBallInUpperTransfer && ballInUpper) {
            // ball has launched
            launched = true;
        } else {
            launched = false;
        }

        wasBallInUpperTransfer = ballInUpper;
        return launched;
    }

    /** setter methods **/

    public void off() {
        if (state != State.OFF) {
            motor.setPower(0);
            state = State.OFF;
        }
    }

    public void forward() {
        if (state != State.FORWARD) {
            motor.setPower(Tunables.transferMotorForwardPower);
            state = State.FORWARD;
        }
    }

    public void reverse() {
        if (state != State.REVERSE) {
            close();
            motor.setPower(Tunables.transferMotorReversePower);
            state = State.REVERSE;
        }
    }

    public void open() {
        if (!isUpperOpen) {
            servo.setPosition(Tunables.transferServoOpen);
            isUpperOpen = true;
        }
    }

    public void close() {
        if (isUpperOpen) {
            servo.setPosition(Tunables.transferServoClosed);
            isUpperOpen = false;
        }
    }

    public void setServoRaw(double pos) {
        // this will cause caching errors, so only use for testing!
        servo.setPosition(pos);
    }

    public void reset() { // reset everything
        servo.setPosition(Tunables.transferServoClosed);
        motor.setPower(0);

        state = State.OFF;
        isUpperOpen = false;
    }
}
