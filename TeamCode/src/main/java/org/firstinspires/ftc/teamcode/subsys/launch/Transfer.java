package org.firstinspires.ftc.teamcode.subsys.launch;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tunables;

public class Transfer {
    private Servo lower, upper;
    private DigitalChannel lowerTransferSensor, upperTransferSensor;

    /** only variables that should change through run time **/

    private boolean isLowerRaised;
    private boolean isUpperOpen;

    /** end vars that change **/

    public Transfer(HardwareMap hw) {
        lower = hw.get(Servo.class, "lowerTransfer");
        upper = hw.get(Servo.class, "upperTransfer");

        lowerTransferSensor = hw.get(DigitalChannel.class, "lowerTransferSensor");
        upperTransferSensor = hw.get(DigitalChannel.class, "upperTransferSensor");

        lowerTransferSensor.setMode(DigitalChannel.Mode.INPUT);
        upperTransferSensor.setMode(DigitalChannel.Mode.INPUT);

        reset();
    }

    /** getter methods **/

    public boolean isLowerRaised() {
        return isLowerRaised;
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

    /** setter methods **/

    public void raiseLower() {
        if (!isLowerRaised) {
            lower.setPosition(Tunables.lowerTransferUpperLimit);
            isLowerRaised = true;
        }
    }

    public void lowerLower() { // goofy name
        if (isLowerRaised) {
            lower.setPosition(Tunables.lowerTransferLowerLimit);
            isLowerRaised = false;
        }
    }

    public void openUpper() {
        if (!isUpperOpen) {
            upper.setPosition(Tunables.upperTransferOpen);
            isUpperOpen = true;
        }
    }

    public void closeUpper() {
        if (isUpperOpen) {
            upper.setPosition(Tunables.upperTransferClosed);
            isUpperOpen = false;
        }
    }

    public void setRawLower(double pos) {
        // this will cause caching errors, so only use for testing!
        lower.setPosition(pos);
    }

    public void setRawUpper(double pos) {
        // this will cause caching errors, so only use for testing!
        upper.setPosition(pos);
    }

    public void reset() { // reset servo positions
        lower.setPosition(Tunables.lowerTransferLowerLimit); // make sure lower transfer is not getting in the way
        upper.setPosition(Tunables.upperTransferClosed); // make sure balls cannot launch

        isLowerRaised = false;
        isUpperOpen = false;
    }
}
