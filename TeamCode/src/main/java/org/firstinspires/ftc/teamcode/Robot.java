/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// hardware imports
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

// unit imports
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsys.Indexer;
import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;
import org.firstinspires.ftc.teamcode.subsys.Light;
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
    public LynxModule controlHub;
    public LynxModule expansionHub;

    /** subsystems **/

    public Flywheel flywheel;
    public Hood hood;
    public Turret turret;
    public Intake intake;
    public Transfer transfer;
    public Indexer indexer;
    public Light light;

    /** only these variables should change during runtime **/

    private LaunchSetpoints setpoints;
    private Timer launchStateTimer;
    private Timer launchIntervalTimer;
    private int ballsRemaining = 0;
    private boolean launching = false;
    private double lastLaunchInterval = 0; // measure last amount of time it took to launch balls in seconds

    /** end vars that change **/

    public Robot(HardwareMap hw) { // create all of our hardware and initialize our class
        // initialize subsystems
        intake = new Intake(hw);
        flywheel = new Flywheel(hw);
        hood = new Hood(hw);
        turret = new Turret(hw, intake.getMotor()); // pass in intake motor to use for turret encoder
        transfer = new Transfer(hw);
        light = new Light(hw);
        indexer = new Indexer(hw);

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

        // light overrides
        if (launching) { light.orange(); }
        if (intake.isReverse()) { light.red(); }

        light.update();

        // turret is more complex, so we need to set and then update
        turret.setDesiredPos(setpoints.getTurretPos());
        turret.update();

        if (!transfer.isBallInLower()) { transfer.forward(); }
        else { transfer.off(); }

        if (launching) {
            if (launchIntervalTimer.getElapsedTime() <= Tunables.maxLaunchTime) {
                intake.forward();
                transfer.forward();
                transfer.open();

                if (transfer.wasBallLaunched()) {
                    ballsRemaining--;
                }

                if (ballsRemaining <= 0) {
                    light.white();
                    launchIntervalTimer.resetTimer();
                    return true;
                } else {
                    return false;
                }
            } else { // we have exceeded our max launch time
                cancelLaunch();
            }
        } else {
            return true; // done launching
        }
        // code will never be reached but IDE gets mad
        return false;
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
        launching = true;
    }

    public void cancelLaunch() {
        ballsRemaining = 0;
        transfer.reset();
        launching = false;
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
