package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Tunables { // this should hold all of our constants
    /** Robot tunables (used in Robot.java) **/

    // this needs to be calculated+changed every time you modify the launch ratio
    public static double launchRatio = (double) 16 / 20; // this is correct because 5202-0002-0001's gearbox ratio is 1:1, and we go from a 16tooth -> 20tooth pulley
    // if intake has a velocity that is less than intakeStallVelocity or a current greater than intakeOvercurrent then we consider it stalled
    public static int intakeStallVelocity = 5; // ~5 RPM, velocity lower than this means we think we have a stalled intake
    public static double intakeOvercurrent = 6; // amount of amps that we think means a stalled intake
    public static double magicNumber = (double) 1 / 15; // magic number for auto RPM

    // PIDF coefficients
    public static double launchP = 100; // the P is too high when on full-charge batteries but 300 is about right for slightly discharged batteries
    public static double launchI = 0.1; // orig 0.1
    public static double launchD = 0; // orig 0.2
    public static double launchF = 20; // ChatGPT was onto nothing with this

    // servo open/close points (don't find these with the backplate on!)
    public static double lowerTransferLowerLimit = 0.28;
    public static double lowerTransferUpperLimit = 0.49;
    public static double upperTransferClosed = 0.36; // servo position where upper transfer prevents balls from passing into launch
    public static double upperTransferOpen = 0.70; // servo position where upper transfer allows balls to pass into launch

    // distance sensor limits (better to undershoot rather than to overshoot)
    // these are all in millimeters
    // if we have less than either of these, then we have a ball
    // TODO: tune these

    public static double intakeSensorOpen = 210; // amount of mm's the intake sensor should report if there is no ball
    public static double lowerTransferSensorOpen = 20; // needs to be different than upper transfer because color sensor distance measurements don't exceed 30mm :(
    public static double upperTransferSensorOpen = 140; // amount of mm's that upper transfer sensor  should report if there is no ball

    // delays
    public static int openDelay = 150; // time to wait for upperTransfer to open (in millis)
    public static int maxPushDelay = 250; // maximum time to wait for lowerTransfer to move (in millis)
    public static int firstInterLaunchWait = 75; // time to wait between 1st and 2nd launches
    public static int lastInterLaunchWait = 200; // time to wait between the 2nd and last launch
    public static double scoreMargin = 100; // margin of 100TPS; TODO: tune this
    public static int intakePollingRate = 250; // how many millis to check intake is full

    /** TeleOp tunables (used in BozoTeleOp.java) **/

    public static double turnRateMultiplier = 0.75; // always have our turns 75% speed
    public static int adjustRPM = 50; // driver increments/decrements by adjustRPM
    public static double initialLaunchRPM = 2300; // maybe 2500; from crease
    public static boolean useBrakes = true; // whether to use brakes in TeleOp
    public static boolean holdEnd = true; // whether to hold end while shooting
    public static double launchTurnMargin = Math.toRadians(5); // margin we want to get our turn to for launch

    /** Auto tunables (used in BozoAuto.java) **/

    public static double scoreRPM = 2400; // RPM to set for launching (stolen from teleop)
    public static double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static int beginningLaunchDelay =  100; // time to wait before launching first ball
    public static double launchDistanceMargin = 2; // must be within this amount of inches to shoot
}
