package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

@Configurable
public class Tunables { // this should hold all of our constants
    public static boolean isDebugging = false;

    /** Robot tunables (used in Robot.java) **/

    // this needs to be calculated+changed every time you modify the launch ratio
    public static double launchRatio = 1; // both of our launch motors are 1:1
    // if intake has a velocity that is less than intakeStallVelocity or a current greater than intakeOvercurrent then we consider it stalled
    public static double autoRPMOffset = 0; // add this many RPMs to auto rpm
    // calibrated 2-6-26; accurate +/-1deg
    // this offset is in radians
    public static double turretOffset = -0.11; // 0 turret position as read in ServoTuner OpMode (make sure to start turret straight forward so our encoder is accurate)
    // you can ensure this value is correct by pressing left/right dpad on TurretTuner and observing the turret error

    // TODO: tune these
    // launch PIDF coefficients
    // updated 1-23-26
    public static double launchP = 0.02;
    public static double launchI = 0;
    public static double launchD = 0.0001;
    public static double launchF = 0.0005;
    public static double launchMaxPowerThreshold = 1000; // if RPM diff is greater than this, bypass PIDF and go full power

    // turret PIDF coefficients
    public static double turretDoubleMinPower = 0.07;
    public static double turretDoubleP = 0.1; // recalibrated 1-28-26
    // don't use I, if there is a steady error because of the dead zone for a while, it will crazily overshoot the next time
    public static double turretDoubleI = 0.0; // recalibrated 1-28-26 - don't use I ^
    public static double turretDoubleD = 0.02; // recalibrated 1-28-26

    public static double turretSingleMargin = Math.toRadians(10); // margin for us control using just one servo, because two has too much power
    public static double turretSinglePosMinPower = 0.02;
    public static double turretSingleNegMinPower = 0.04; // recalibrated 1-28-26
    public static double turretSingleP = 0.3; // recalibrated 1-28-26
    public static double turretSingleI = 0.3; // recalibrated 1-28-26
    // I = 2 works well but is super slow
    public static double turretSingleD = 0.04; // recalibrated 1-28-26
    public static double turretTxReduction = 6; // amount to divide turret tx by before applying to new desired turret position
    public static double turretMaxVelocityForVision = Math.toRadians(10); // max amount of error to apply vision offset

    // servo open/close points (don't find these with the backplate on!)
    public static double lowerTransferLowerLimit = 0.154; // recalibrated 2-7-26
    public static double lowerTransferUpperLimit = 0.45; // recalibrated 1-27-26
    // servo position where upper transfer prevents balls from passing into launch
    public static double upperTransferClosed = 0.00; // recalibrated 1-27-26
    // servo position where upper transfer allows balls to pass into launch
    public static double upperTransferOpen = 0.28; // recalibrated 1-16-25

    public static double hoodMinimum = 0.32; // calibrated 1-22-26
    public static double hoodMaximum = 0.54; // calibrated 1-22-26

    // distance sensor limits (better to undershoot rather than to overshoot)
    public static double intakeSensorOpen = 210; // amount of mm's the intake sensor should report if there is no ball
    public static double lowerTransferSensorOpen = 140; // needs to be different than upper transfer because color sensor distance measurements don't exceed 30mm :(

    // delays
    public static int openDelay = 0; // time to wait for upperTransfer to open (in millis)
    public static int lowerDelay = 40; // time to wait for lower transfer to lower
    public static int transferDelay = 115; // time to wait for ball to enter lower transfer
    public static int lastTransferDelay = 300; // time to wait for last ball to enter lower transfer
    public static int extraPushDelay = 20; // extra time to wait for exit after ball triggers upperTransferSensor
    public static int maxPushDelay = 250; // maximum time to wait for lowerTransfer to move (in millis)

    public static double scoreMargin = 100; // margin of 100TPS; TODO: tune this
    public static double launchingIntakePower = 0; // just enough power to keep balls in, without moving them

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

    public static double turnRateMultiplier = 0.75; // always have our turns 75% speed
    public static int adjustRPM = 50; // driver increments/decrements by adjustRPM
    public static double initialManualLaunchRPM = 2350; // 2400 is a little too much
    public static boolean useBrakes = true; // whether to use brakes in TeleOp
    public static double maxTurretLockMillis = 200;
    public static double farZoneDataStart = 95; // if d > this, use far zone data

    /** Auto tunables (used in BozoAuto.java) **/

    public static double scoreRPM = 2250; // RPM to set for launching (stolen from teleop)
    public static double scoreHoodPos = 0.171; // hood position for launching in auto
    public static double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double clearEndTime = 0.1; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double clearTime = 1000; // amount of ms to wait for clear
    public static double launchDistanceMargin = 2; // must be within this amount of inches to shoot
    public static double maxGrabVelocity = 50; // max velocity while grabbing balls in inches/second
    public static double clearVelocity = 10;
    public static double clearMaxPower = 0.8;
    public static double farAutoRPMOffset = 90; // manually calibrated 2-7-26
    public static double magicOffset = 0.12;

    /** Vision tunables (used in Vision.java) **/
    public static long maxVisionStaleness = 50; // amount of millis without a reading where vision becomes stale
    public static double goalOffset = -30; // TODO:

    // offsets for translating from limelight->field position
    // TODO: calibrate these
    //public static double limelightOffsetZ = 0;
    public static double maxLimelightHeadingError = Math.toRadians(5); // if our limelight reports a heading that differs by more than this, discard that measurement
    public static double turretOffsetX = 0;
    public static double turretOffsetY = 0;
    public static double limelightTurretRadius = 0; // how far limelight is mounted from center of turret in inches
}
