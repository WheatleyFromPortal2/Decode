/** class that holds all of our constants that we want to be able to tune quickly
 * all values in are milliseconds unless otherwise stated
 */

package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

@Configurable
public class Tunables {
    public static boolean isDebugging = true;

    /** Robot tunables (used in Robot.java) **/

    // launch delays
    public static int maxLaunchTime = 500; // max amount of time to complete one launch cycle


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

    /** intake tunables (used in Intake.java) **/

    public static boolean intakeUsePowerSave = false;
    public static double intakeSensorOpen = 210; // amount of mm's the intake sensor should report if there is no ball
    public static int intakePowerSaveWaitInterval = 30; // how often to check for a stalled intake while running
    public static int intakePowerSaveCheckInterval = 30; // how often to update intake power/velocity check in millis
    public static double intakePowerSaveTriggerAmps = 8; // if we are drawing more amsp than this, trigger power save
    public static double intakePowerSaveTriggerVelocity = 0; // if we have a tps less than this, trigger power save
    public static double intakeHoldPower = 0.75; // minimum power for balls to not fall out of intake

    /** launch tunables (used in Launch.java) **/

    // launch PIDF coefficients
    // updated 1-23-26
    public static double flywheelP = 0.02;
    public static double flywheelI = 0;
    public static double flywheelD = 0.0001;
    public static double flywheelF = 0.0005;

    public static double maxFlywheelBreaking = 0.05; // max amount of breaking (negative power) to apply to launch motors
    public static double flywheelQuickStartThreshold = 500; // if TPS diff is greater than this, set full power (~1000rpm)
    public static double flywheelMargin = 100; // margin in TPS for flywheel to be considered "within range" of shooting

    /** hood tunables (used in Hood.java) **/

    public static double hoodMinimumPos = 0.35; // calibrated 2-23-26
    public static double hoodMinimumRadians = Math.toRadians(48); // calibrated 2-23-26
    public static double hoodMaximumPos = 0.55; // calibrated 2-23-26
    public static double hoodMaximumRadians = Math.toRadians(72); // calibrated 2-23-26
    public static double hoodWriteMargin = 0.01; // if our hood position change is less than this, don't waste time with a write
    // angles: 0.22 = 50deg; 0.00917 = 19deg
    // just use a map and save lowest/highest deg vals

    /** turret tunables (used in Turret.java) **/
    public static double turretCenterOffset = -0.11; // 0 turret position as read in ServoTuner OpMode (make sure to start turret straight forward so our encoder is accurate)
    // calibrated 2-24-26
    public static double turretMaxLeft = -3.5122; // max range that the turret can go left or right from servo center
    // 1.837
    public static double turretMaxRight = 3.57668;

    //these are in radians
    public static double turretLimitLeft = -2;
    public static double turretLimitRight = 2;

    /** transfer tunables (used in Transfer.java) **/

    public static double transferMotorForwardPower = 1;
    public static double transferMotorReversePower = -1;

    // servo position where upper transfer prevents balls from passing into launch
    // put this at just enough to stop balls but not enough that it gets stuck on launch
    public static double transferServoClosed = 0.00; // recalibrated 2-20-26
    // servo position where upper transfer allows balls to pass into launch
    public static double transferServoOpen = 0.32; // recalibrated 2-17-25

    /** indexer tunables (used in Indexer.java) **/

    public static double indexerKickerIntake = 0.99; // get balls from intake (down)
    public static double indexerKickerUp = 0.858; // move balls to chute
    public static double indexerKickerChute = 0.674; // get balls from chute

    public static double indexerBlockerClosed = 0.5;
    public static double indexerBlockerOpen = 0.0;


    /** TeleOp tunables (used in BozoTeleOp.java) **/

    public static double fieldCentricTurnRateMultiplier = 0.75; // what to multiply turn input while doing field centric control
    public static double robotCentricTurnRateMultiplier = 0.4; // what to multiply turn input while doing robot centric control
    public static int adjustRPM = 50; // driver increments/decrements by adjustRPM
    public static double initialManualLaunchRPM = 2350; // 2400 is a little too much
    public static boolean useBrakes = true; // whether to use brakes in TeleOp
    public static double farZoneDataStart = 95; // if d > this, use far zone data
    public static boolean isDynamicPhysics = true;

    /** bozo auto tunables (used in BozoAuto.java) **/

    public static double bozoScoreRPM = 2100; // RPM to set for launching (stolen from teleop)
    public static double bozoScoreHoodRadians = Math.toRadians(60); // hood position for launching in auto
    public static double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double clearTime = 1000; // amount of ms to wait for clear
    public static double launchDistanceMargin = 2; // must be within this amount of inches to shoot
    public static double maxGrabVelocity = 50; // max velocity while grabbing balls in inches/second
    public static double clearMaxPower = 0.8;

    /** far auto tunables (used in FarAuto.java) **/

    public static double farScoreRPM = 3500;
    // hood position is always set to minimum angle since we're so far
    public static double farScoreHoodRadians = Math.toRadians(-60);

    // auto cycle and delay configuration
    public static int farCycles = 3; // how many times to cycle through (1 = only score preload)
    // array that holds all of our waits before launches
    // this array may be longer than farCycles but must never be shorter
    public static int[] farLaunchWaits = {
            0, // wait before 1st launch
            0, // wait before 2nd launch
            0, // wait before 3rd launch
            0  // wait before 4th launch
    };


    // delays (all in millis)
    public static double launchDelay1 = 0;
    public static double launchDelay2 = 0;
    public static double launchDelay3 = 0;

    /** vision tunables (used in Vision.java) **/
    public static long maxVisionStaleness = 50; // amount of millis without a reading where vision becomes stale
    public static double goalOffset = -30; // TODO:

    // offsets for translating from limelight->field position
    // TODO: calibrate these
    //public static double limelightOffsetZ = 0;
    public static double turretOffsetX = 0;
    public static double turretOffsetY = 0;
    public static double limelightTurretRadius = 0; // how far limelight is mounted from center of turret in inches

    /** fusion tunables (used in Fusion.java) **/

    public static double fusionInitialVariance = 5;
    public static double fusionMaxVisionError = 20; // don't use vision if it differs by more inches than this
    public static double fusionProcessNoiseBase = 0.005; // odo slips very little
    public static double fusionProcessNoisePerInch = 0.01; // pinpoint is pretty accurate
    public static double fusionVisionVariance = 4.0;

    // prevent variance collapse
    public static double fusionMinVariance = 0.5;
    public static double fusionMaxVariance = 30;

    /** physics tunables (used in Physics.java) **/

    // both of these must be in meters
    public static double shootZ = 0.4064; // calibrated 2-13-26
    public static double goalZ = 0.7874; // calibrated 2-13-26
    public static double physicsRPMOffset = 200;
    public static double staticShotDelay = 0.2;
    public static double angleCutoff = 120; // if inches more than this
}
