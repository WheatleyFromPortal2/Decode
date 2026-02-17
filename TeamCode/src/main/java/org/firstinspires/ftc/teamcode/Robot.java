/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// hardware imports
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

// unit imports
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsys.Fusion;
import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;
import org.firstinspires.ftc.teamcode.subsys.Vision;
import org.firstinspires.ftc.teamcode.subsys.launch.Flywheel;
import org.firstinspires.ftc.teamcode.subsys.launch.Hood;
import org.firstinspires.ftc.teamcode.subsys.launch.Intake;
import org.firstinspires.ftc.teamcode.subsys.launch.Transfer;
import org.firstinspires.ftc.teamcode.subsys.launch.Turret;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

// Pedro Pathing imports
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;


public class Robot { // create our global class for our robot
    public enum LaunchState { // these are the possible states our launch state machine can be in
        START,
        OPEN_UPPER_TRANSFER,
        RAISE_LOWER_TRANSFER,
        WAIT_FOR_SENSOR_HIT,
        WAIT_FOR_EXIT,
        WAIT_FOR_LOWER,
        WAIT_FOR_TRANSFER
    }

    public LynxModule controlHub;
    public LynxModule expansionHub;

    /** subsystems **/

    public Flywheel flywheel;
    public Hood hood;
    public Turret turret;
    public Intake intake;
    public Transfer transfer;

    public Fusion fusion;
    public Vision vision;


    /** only these variables should change during runtime **/

    private LaunchSetpoints setpoints;
    private LaunchState state;
    private Timer launchStateTimer;
    private Timer launchIntervalTimer;
    private int ballsRemaining = 0;
    private double lastLaunchInterval = 0; // measure last amount of time it took to launch balls in seconds

    /** end vars that change **/

    public Robot(HardwareMap hw) { // create all of our hardware and initialize our class
        // initialize subsystems
        flywheel = new Flywheel(hw);
        hood = new Hood(hw);
        turret = new Turret(hw);
        intake = new Intake(hw);
        transfer = new Transfer(hw);

        vision = new Vision(hw);
        fusion = new Fusion(HandoffState.pose);

        setpoints = new LaunchSetpoints(0, 0, 0);

        controlHub = hw.get(LynxModule.class, "Control Hub");
        expansionHub = hw.get(LynxModule.class, "Expansion Hub 2"); // I believe this starts at 2

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        launchStateTimer = new Timer(); // set up timer for the launch state machine
        launchIntervalTimer = new Timer();
    }

    public boolean update() {
        // update static
        flywheel.setpointUpdate(setpoints.getTPS());
        hood.update(setpoints.getHoodPos());

        // turret is more complex, so we need to set and then update
        turret.setDesiredPos(setpoints.getTurretPos());
        turret.update();

        if (ballsRemaining == 0) { // if we are done with balls or our launch isn't running fast enough
            cancelLaunch();
            return true; // we're done with launching balls
        } else { // balls remaining > 0 && we are launching
            switch (state) {
                case START:
                    intake.forward();
                    launchStateTimer.resetTimer();
                    //if (ballsRemaining > 1) intake.setPower(Tunables.launchingIntakePower); // hopefully allow lowerTransfer to go down
                    if (!transfer.isUpperOpen()) {
                        transfer.openUpper();
                        setState(LaunchState.OPEN_UPPER_TRANSFER);
                    } else {
                        setState(LaunchState.RAISE_LOWER_TRANSFER);
                    }
                    break;
                case OPEN_UPPER_TRANSFER:
                    if (launchStateTimer.getElapsedTime() >= Tunables.openDelay) {
                        setState(LaunchState.RAISE_LOWER_TRANSFER);
                    }
                    break;
                case RAISE_LOWER_TRANSFER:
                    transfer.raiseLower();

                    intake.hold();
                    launchStateTimer.resetTimer();
                    setState(LaunchState.WAIT_FOR_SENSOR_HIT);
                    break;
                case WAIT_FOR_SENSOR_HIT:
                    if (transfer.isBallInUpper() // wait until we detect a ball in upper transfer (ball has been launched)
                            || launchStateTimer.getElapsedTime() >= Tunables.maxPushDelay) { // or if that hasn't happened in a while, just go to the next launch
                        launchStateTimer.resetTimer();
                        setState(LaunchState.WAIT_FOR_EXIT);
                    }
                    break;
                case WAIT_FOR_EXIT:
                    if (launchStateTimer.getElapsedTime() >= Tunables.extraPushDelay) {
                        transfer.lowerLower();
                        ballsRemaining -= 1; // we've launched a ball
                        if (ballsRemaining == 0) { // we're done with launching
                            setState(LaunchState.START);
                            lastLaunchInterval = launchIntervalTimer.getElapsedTimeSeconds();
                        }
                        else {
                            setState(LaunchState.WAIT_FOR_LOWER);
                        }
                        launchStateTimer.resetTimer(); // reset our timer
                    }
                    break;
                case WAIT_FOR_LOWER:
                    if (launchStateTimer.getElapsedTime() >= Tunables.lowerDelay) {
                        setState(LaunchState.WAIT_FOR_TRANSFER);
                        intake.forward();
                        launchStateTimer.resetTimer();
                    }
                    break;
                case WAIT_FOR_TRANSFER:
                    if (ballsRemaining == 1) {
                        if (launchStateTimer.getElapsedTime() < Tunables.lastTransferDelay && !transfer.isBallInLower()) {
                            break;
                        }
                    } else {
                        if (launchStateTimer.getElapsedTime() < Tunables.transferDelay && !transfer.isBallInLower()) {
                            break;
                        }
                    }
                    launchStateTimer.resetTimer();
                    setState(LaunchState.START);
                    break;
            }
        }
        return false;
    }

    private void setState(LaunchState newState) {
        if (newState == LaunchState.START) { intake.forward(); }
        state = newState;
        launchStateTimer.resetTimer();
    }

    public double getSystemVoltage() { // return system current in volts
        return (controlHub.getInputVoltage(VoltageUnit.VOLTS) + expansionHub.getInputVoltage(VoltageUnit.VOLTS)) / 2; // average values for more accuracy
    }

    public double getSystemCurrent() { // return system current in amps
        return (controlHub.getCurrent(CurrentUnit.AMPS) + expansionHub.getCurrent(CurrentUnit.AMPS));
    }

    public double getGoalDst(Pose currentPosition, Pose goalPose) { // get our distance from the goal in inches
        return currentPosition.distanceFrom(goalPose) + Tunables.goalOffset; // use poses to find our distance easily :)
    }

    public double getTurretGoalHeading(@NonNull Pose currentPosition, @NonNull Pose goalPose) { // return turret heading to point towards goal in radians
        double xDst = goalPose.getX() - currentPosition.getX();
        double yDst = goalPose.getY() - currentPosition.getY();
        double desiredAbsoluteHeading = Math.atan2(yDst, xDst);
        return desiredAbsoluteHeading - currentPosition.getHeading() + Tunables.magicOffset;
        //return desiredAbsoluteHeading - currentPosition.getHeading();
    }

    public void setAutomatedLaunchSetpoints(double d) { // given positions, use our functions to set our launch speed and hood position
        if (!isLaunching()) { // don't update if we're launching or if
            double RPM = 0;
            double hoodPos = 0;
            if (d < Tunables.farZoneDataStart) { // use close zone data
                RPM = -0.00943691 * Math.pow(d, 2) +12.47649 * d + 1772.38054 + Tunables.closeAutoRPMOffset;
                // from Desmos data: 2-5-26 (removing outliers)
                // R^2 = 0.973 using linear regression

                hoodPos = 0.00000256939 * Math.pow(d, 3) - 0.000531448 * Math.pow(d, 2) + 0.0367024 * d - 0.675026;
                // from Desmos data: 2-5-26 (removing outliers)
                // R^2 = 0.6066 using cubic regression
            } else { // use far zone data
                RPM = 0.0498139 * Math.pow(d, 2) - 1.32898 * d + 2370.65436 + Tunables.farAutoRPMOffset;
                // from Desmos data: 2-5-26 (removing outliers)
                // R^2 = 0.9822 using quadratic regression

                hoodPos = 0.22;
                // from Desmos data: 2-5-26 (removing outliers)
                // at this distance, we should always be using the lowest angle
            }

            setpoints.setRPM(RPM);
            setpoints.setHoodPos(hoodPos);
        }
    }

    /** ball launching methods **/

    public void launchBalls(int balls) { // sets to launch this many balls
        ballsRemaining += balls;
        if (ballsRemaining > 3) ballsRemaining = 3;
        launchStateTimer.resetTimer(); // reset launch state timer (it may be off if cancelled)
        launchIntervalTimer.resetTimer();
        state = LaunchState.START; // reset our state machine to the start
    }

    public void cancelLaunch() {
        state = LaunchState.START;
        ballsRemaining = 0;
        transfer.reset();
    }

    public int getBallsRemaining() { return ballsRemaining; }

    public double getLastLaunchInterval() { return lastLaunchInterval; }

    public boolean isLaunching() {
        return ballsRemaining > 0;
    }

    /** getter methods **/

    public LaunchSetpoints getSetpoints() {
        return setpoints;
    }

    /** setter methods **/

    public void setSetpoints(LaunchSetpoints newState) {
        setpoints = newState;
    }
}
