package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

@Configurable
public class Tunables { // this should hold all of our constants
    /** Robot tunables (used in Robot.java) **/

    // this needs to be calculated+changed every time you modify the launch ratio
    public static double launchRatio = 1; // both of our launch motors are 1:1
    // if intake has a velocity that is less than intakeStallVelocity or a current greater than intakeOvercurrent then we consider it stalled
    public static double intakeOvercurrent = 6; // amount of amps that we think means a stalled intake
    public static double magicNumber = -50; // add this many RPMs to auto rpm

    // TODO: tune these
    // launch PIDF coefficients
    // updated 1-23-26
    public static double launchP = 0.012;
    public static double launchI = 0;
    public static double launchD = 0;
    public static double launchF = 0.0005;
    public static double launchMaxPowerThreshold = 1000; // if RPM diff is greater than this, bypass PIDF and go full power

    // TODO: tune these
    // turret PIDF coefficients
    public static double turretDoubleP = 0.4; // recalibrated 1-25-26
    // 0.25 works pretty well for 2servos
    // 0.6 works well for 1servo
    public static double turretDoubleI = 0;
    public static double turretDoubleD = 0.03; // recalibrated 1-25-26

    public static double turretSingleMargin = Math.toRadians(10); // margin for us control using just one servo, because two has too much power
    public static double turretSingleP = 4;
    public static double turretSingleI = 0;
    // I = 2 works well but is super slow
    public static double turretSingleD = 0.05;

    // servo open/close points (don't find these with the backplate on!)
    public static double lowerTransferLowerLimit = 0.6871; // recalibrated 1-16-25
    public static double lowerTransferUpperLimit = 0.87; // recalibrated 1-27-26
    // servo position where upper transfer prevents balls from passing into launch
    public static double upperTransferClosed = 0.00; // recalibrated 1-27-26
    // servo position where upper transfer allows balls to pass into launch
    public static double upperTransferOpen = 0.28; // recalibrated 1-16-25

    public static double hoodMinimum = 0.32; // calibrated 1-22-26
    public static double hoodMaximum = 0.54; // calibrated 1-22-26
    public static int hoodHomingTime = 1500; // millis to wait for hood to reach maximum position

    // distance sensor limits (better to undershoot rather than to overshoot)
    public static double intakeSensorOpen = 210; // amount of mm's the intake sensor should report if there is no ball
    public static double lowerTransferSensorOpen = 140; // needs to be different than upper transfer because color sensor distance measurements don't exceed 30mm :(

    // delays
    public static int openDelay = 150; // time to wait for upperTransfer to open (in millis)
    public static int lastOpenDelay = 350;
    public static int maxTransferDelay = 200; // maximum time to wait for ball to enter lower transfer
    public static int maxPushDelay = 250; // maximum time to wait for lowerTransfer to move (in millis)

    public static double scoreMargin = 100; // margin of 100TPS; TODO: tune this
    public static int intakeOvercurrentDelay = 250; // if intake has been overcurrent for more than this many millis, we consider it full
    public static double launchingIntakePower = 0.1;

    // rumble effects
    private static int delay1 = 200; // ms delay for 1ball
    private static int delay2 = 100; // ms delay for 2balls
    private static int delay3 = 10; // ms delay for 3balls
    public static Gamepad.RumbleEffect rumble1 = new Gamepad.RumbleEffect.Builder() // rumble for when we have 1ball
            .addStep(0.0, 1.0, delay1) // rumble left motor 100% for delay1
            .addStep(1.0, 0.0, delay1) // rumble right motor 100% for delay1
            .addStep(0.0, 0.0, delay1) // pause for delay1
            .build();
    public static Gamepad.RumbleEffect rumble2 = new Gamepad.RumbleEffect.Builder() // rumble for when we have 2balls
            .addStep(0.0, 1.0, delay2) // rumble left motor 100% for delay2
            .addStep(1.0, 0.0, delay2) // rumble right motor 100% for delay2
            .addStep(0.0, 0.0, delay2) // pause for delay2
            .build();
    public static Gamepad.RumbleEffect rumble3 = new Gamepad.RumbleEffect.Builder() // rumble for when we have 3balls (needs to be differentiated)
            .addStep(1.0, 1.0, delay3) // rumble left motor 100% for delay3
            .addStep(0.0, 0.0, delay3) // rumble right motor 100% for delay3
            // no pause (more intensity)
            .build();

    /** TeleOp tunables (used in BozoTeleOp.java) **/

    public static boolean isCalibrationMode = false; // whether out teleop should allow manual control for calibration
    public static double turnRateMultiplier = 0.75; // always have our turns 75% speed
    public static int adjustRPM = 50; // driver increments/decrements by adjustRPM
    public static double initialManualLaunchRPM = 2350; // 2400 is a little too much
    public static boolean useBrakes = true; // whether to use brakes in TeleOp
    public static boolean holdEnd = true; // whether to hold end while shooting
    public static double launchTurnMargin = Math.toRadians(5); // margin we want to get our turn to for launch

    /** Auto tunables (used in BozoAuto.java) **/

    public static double scoreRPM = 2300; // RPM to set for launching (stolen from teleop)
    public static double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double clearEndTime = 0.1; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static int beginningLaunchDelay =  200; // time to wait before launching first ball
    public static double clearTime = 000; // amount of ms to wait for clear
    public static double launchDistanceMargin = 2; // must be within this amount of inches to shoot

    /** Vision tunables (used in Vision.java) **/
    public static double turnP = 1;
    public static double turnI = 0;
    public static double turnD = 0;

    public static long maxVisionStaleness = 20; // amount of millis without a reading where vision becomes stale
    public static double goalOffset = 0; // how many inches less our vision reports the goal to be at than it actually is
}
