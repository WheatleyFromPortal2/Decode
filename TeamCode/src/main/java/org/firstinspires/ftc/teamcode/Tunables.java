/** class that holds all of our constants that we want to be able to tune quickly
 * all values in are milliseconds unless otherwise stated
 */

package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsys.Vision;

@Configurable
public class Tunables {
    public static boolean isDebugging = false;

    /** Robot tunables (used in Robot.java) **/

    // launch delays
    public static int maxLaunchTime = 500; // max amount of time to complete one launch cycle

    /** intake tunables (used in Intake.java) **/

    public static boolean intakeUsePowerSave = false;
    public static double intakeSensorOpen = 210; // amount of mm's the intake sensor should report if there is no ball
    public static int intakePowerSaveWaitInterval = 30; // how often to check for a stalled intake while running
    public static int intakePowerSaveCheckInterval = 30; // how often to update intake power/velocity check in millis
    public static double intakePowerSaveTriggerAmps = 8; // if we are drawing more amsp than this, trigger power save
    public static double intakePowerSaveTriggerVelocity = 0; // if we have a tps less than this, trigger power save
    public static double intakeHoldPower = 0.0; // minimum power for balls to not fall out of intake

    /** launch tunables (used in Launch.java) **/

    // launch PIDF coefficients
    // updated 1-23-26
    public static double flywheelP = 0.02;
    public static double flywheelI = 0;
    public static double flywheelD = 0.0001;
    public static double flywheelF = 0.0005;

    public static double maxFlywheelBreaking = 0.05; // max amount of breaking (negative power) to apply to launch motors
    public static double flywheelQuickStartThreshold = 500; // if TPS diff is greater than this, set full power (~1000rpm)
    public static double flywheelMargin = 10; // margin in TPS for flywheel to be considered "within range" of shooting

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

    public static double turretP = 0.12;
    public static double turretI = 0.00;
    public static double turretD = 0.01;
    public static double turretF = 0;
    public static double turretPIDFMargin = Math.toRadians(10);

    // calibrated 2-87-26
    public static double turretMaxLeft = 2;
    public static double turretMaxRight = -2;

    // max servo positions
    public static double turretLimitLeft = 0.25;
    public static double turretLimitRight = 0.75;

    /** transfer tunables (used in Transfer.java) **/

    public static double transferMotorForwardPower = 1;
    public static double transferMotorReversePower = -1;

    // servo position where upper transfer prevents balls from passing into launch
    // put this at just enough to stop balls but not enough that it gets stuck on launch
    public static double transferServoClosed = 0.00; // recalibrated 2-20-26
    // servo position where upper transfer allows balls to pass into launch
    public static double transferServoOpen = 0.32; // recalibrated 2-17-25

    /** indexer tunables (used in Indexer.java) **/

    public static double indexerKickerIntake = 0.98; // get balls from intake (down)
    public static double indexerKickerUp = 0.87; // move balls to chute
    public static double indexerKickerChute = 0.65; // get balls from chute

    public static double indexerBlockerClosed = 0.45;
    public static double indexerBlockerOpen = 0.75;
    public static double indexerBlockerUp = 0.11;

    public static float purpleBot = 100;
    public static float purpleTop = 100;

    public static int indexWait = 750;


    /** TeleOp tunables (used in BozoTeleOp.java) **/

    public static double fieldCentricTurnRateMultiplier = 0.75; // what to multiply turn input while doing field centric control
    public static double robotCentricTurnRateMultiplier = 0.4; // what to multiply turn input while doing robot centric control
    public static int adjustRPM = 50; // driver increments/decrements by adjustRPM
    public static double initialManualLaunchRPM = 2350; // 2400 is a little too much
    public static boolean useBrakes = true; // whether to use brakes in TeleOp
    public static double farZoneDataStart = 95; // if d > this, use far zone data
    public static boolean isDynamicPhysics = true;
    public static boolean usingKalman = false;

    /** bozo auto tunables (used in BozoAuto.java) **/

    public static double lastReverse = 300;
    public static double reverseTime = 200;
    public static double bozoScoreRPM = 2150; // RPM to set for launching (stolen from teleop)
    public static double bozoScoreHoodRadians = Math.toRadians(60); // hood position for launching in auto
    public static double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double clearTime = 1000; // amount of ms to wait for clear
    public static double launchDistanceMargin = 2; // must be within this amount of inches to shoot
    public static double grabPowerMultiplier = 0.34; // max velocity while grabbing balls in inches/second
    public static double maxSortVelocity = 5; //max velocity while sorting
    public static double sortTime = 0.75;
    public static double clearMaxPower = 0.8;

    public static double startLaunchDelay1 = 100;
    public static double startLaunchDelay2 = 100;

    /** far auto tunables (used in FarAuto.java) **/

    public static double farScoreRPM = 3200;
    // hood position is always set to minimum angle since we're so far
    public static double farScoreHoodRadians = Math.toRadians(-60);

    // auto cycle and delay configuration
    public static int farCycles = 10; // how many times to cycle through (1 = only score preload)
    // array that holds all of our waits before launches
    // this array may be longer than farCycles but must never be shorter
    public static int[] farLaunchWaits = {
            0, // wait before 1st launch
            0, // wait before 2nd launch
            0, // wait before 3rd launch
            0  // wait before 4th launch
    };


    /** vision tunables (used in Vision.java) **/
    public static Vision.Triplet tripletOverride = Vision.Triplet.UNKNOWN;
    public static long maxVisionStaleness = 50; // amount of millis without a reading where vision becomes stale

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
    public static double physicsRPMMultiplier = 1.1;
    public static double physicsRPMOffset = 50;
    public static double staticShotDelay = 0.2;
    public static double angleCutoff = 120; // if inches more than this

    /** pattern tunables **/
    public static boolean dumbPattern = true;
    public static String patternT = "C0";
}
