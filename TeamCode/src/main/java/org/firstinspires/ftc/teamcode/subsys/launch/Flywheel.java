package org.firstinspires.ftc.teamcode.subsys.launch;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;
import org.firstinspires.ftc.teamcode.subsys.PIDF;

public class Flywheel {
    // if the TPS of either motor encoder is lower than this, assume that it is dysfunctional and fall back to the other
    private double FALLBACK_MIN_TPS = 5;

    /** hardware **/
    private DcMotorEx launchLeft, launchRight;

    /** stuff that changes **/
    private PIDF pidf;
    private boolean isLeftEncoderDisconnected = false;
    private double lastSetpointTPS = 0;

    public Flywheel(HardwareMap hw) {
        // launch motors (all are DcMotorEx for current monitoring)
        launchLeft = hw.get(DcMotorEx.class, "launchLeft"); // left flywheel motor, connected with a 1:1 ratio
        launchRight = hw.get(DcMotorEx.class, "launchRight"); // right flywheel motor, connected with a 1:1 ratio

        // set up hardware
        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);
        launchRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor

        // set up PIDF
        pidf = new PIDF(Tunables.flywheelP, Tunables.flywheelI, Tunables.flywheelD, Tunables.flywheelF); // create our PIDF controller for our launch motors
    }

    public void setpointUpdate(double setpointTPS) {
        pidf.updateTerms(Tunables.flywheelP, Tunables.flywheelI, Tunables.flywheelD, Tunables.flywheelF);
        double currentTPS = getTPS();
        double newPower;

        if (setpointTPS == 0) {
            newPower = 0.0;
        } else if (Math.abs(currentTPS - setpointTPS) < Tunables.flywheelQuickStartThreshold) {
            // small difference - use PIDF
            newPower = pidf.calc(setpointTPS, currentTPS); // calc new motor power
        } else {
            // big difference - full power
            newPower = 1.0;
        }

        double clippedPower = Range.clip(newPower, -Tunables.maxFlywheelBreaking, 1.0);

        launchLeft.setPower(clippedPower);
        launchRight.setPower(clippedPower);

        lastSetpointTPS = setpointTPS;
    }

    public void powerUpdate(double newPower) {
        double clippedPower = Range.clip(newPower, 0.0, 1.0);

        launchLeft.setPower(clippedPower);
        launchRight.setPower(clippedPower);
    }

    /** internal methods **/

    public double getTPS() {
        // get TPS of flywheel in the fastest way while also being able to fall back between encoders
        if (!isLeftEncoderDisconnected) {
            double leftVelocity = Math.abs(launchLeft.getVelocity()); // account for encoder plugged into wrong port

            if (leftVelocity <= FALLBACK_MIN_TPS) { // if our velocity is negative, fall back!
                isLeftEncoderDisconnected = true;
                return Math.abs(launchRight.getVelocity());
            } else {
                return leftVelocity;
            }
        } else {
            double rightVelocity = Math.abs(launchRight.getVelocity()); // account for encoder plugged into wrong port
            if (rightVelocity <= FALLBACK_MIN_TPS) { // if our velocity is negative, fall back!
                isLeftEncoderDisconnected = false;
                return Math.abs(launchLeft.getVelocity());
            } else { return rightVelocity; }
        }
    }

    /** getter methods **/

    public double getRPM() {
        LaunchSetpoints current = new LaunchSetpoints(getTPS(), 0, 0);
        return current.getRPM();
    }

    public boolean isWithinMargin() {
        if (lastSetpointTPS == 0 || getTPS() == 0) { return false; }
        return Math.abs(lastSetpointTPS - getTPS()) < Tunables.flywheelMargin;
    }

    // these should only be used for tuning
    public double getPower() {
        return launchLeft.getPower(); // doesn't mater which one we query for power - we're both setting them the same
    }

    public double getCurrent() { // return sum current in amps
        return launchLeft.getCurrent(CurrentUnit.AMPS) + launchRight.getCurrent(CurrentUnit.AMPS);
    }
}
