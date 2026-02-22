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
    private Timer launchTimer;
    private boolean launching = false;
    private boolean hasLaunchedFirst = false;
    private double firstShotDelay = Tunables.staticShotDelay; // start with static

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

        launchTimer = new Timer();
    }

    public boolean update() {
        // update static
        flywheel.setpointUpdate(setpoints.getTPS());
        hood.setRadians(setpoints.getHoodRadians());

        // light overrides
        if (launching) { light.orange(); }
        if (intake.isReverse()) { light.red(); }

        // turret is more complex, so we need to set and then update
        turret.setDesiredPos(setpoints.getTurretPos());
        turret.update();

        if (!transfer.isBallInLower()) { transfer.forward(); }
        else { transfer.off(); }

        if (launching) {
            light.orange();
            if (launchTimer.getElapsedTime() >= Tunables.maxLaunchTime) {
                endLaunch();
                return true;
            } else {
                if (!hasLaunchedFirst && transfer.wasBallLaunched()) {
                    firstShotDelay = launchTimer.getElapsedTimeSeconds();
                    hasLaunchedFirst = true;
                }
                transfer.forward();
                transfer.open();
                intake.forward();
                return false;
            }
        } else {
            light.blue();
            return true;
        }
    }

    public double getSystemVoltage() { // return system current in volts
        return (controlHub.getInputVoltage(VoltageUnit.VOLTS) + expansionHub.getInputVoltage(VoltageUnit.VOLTS)) / 2; // average values for more accuracy
    }

    public double getSystemCurrent() { // return system current in amps
        return (controlHub.getCurrent(CurrentUnit.AMPS) + expansionHub.getCurrent(CurrentUnit.AMPS));
    }

    /** ball launching methods **/

    public void launch() {
        launching = true;
        hasLaunchedFirst = false;
        launchTimer.resetTimer();
    }

    public void endLaunch() {
        launching = false;
        transfer.close();
    }

    /** getter methods **/

    public LaunchSetpoints getSetpoints() {
        return setpoints;
    }

    public boolean isLaunching() {
        return launching;
    }

    public double getFirstShotDelay() {
        return firstShotDelay;
    }

    /** setter methods **/

    public void setSetpoints(LaunchSetpoints newState) {
        setpoints = newState;
    }
}
