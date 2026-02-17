package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

@Configurable
public class Tunables { // this should hold all of our constants
    public static boolean isDebugging = false;

    /** Robot tunables (used in Robot.java) **/

    // automatic launch calc offsets
    public static double closeAutoRPMOffset = 0; // add this many RPMs to auto rpm
    public static double farAutoRPMOffset = 90; // manually calibrated 2-7-26

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

    public static boolean intakeUsePowerSave = true;
    public static double intakeSensorOpen = 210; // amount of mm's the intake sensor should report if there is no ball
    public static int intakePowerSaveWaitInterval = 100; // how often to check for a stalled intake while running
    public static int intakePowerSaveCheckInterval = 100; // how often to update intake power/velocity check in millis
    public static double intakePowerSaveTriggerAmps = 4; // if we are drawing more amsp than this, trigger power save
    public static double intakePowerSaveTriggerVelocity = 0; // if we have a tps less than this, trigger power save
    public static double intakeHoldPower = 0.05; // minimum power for balls to not fall out of intake

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

    public static double hoodMinimumPos = 0.32; // calibrated 1-22-26
    public static double hoodMinimumRadians = Math.toRadians(50); // calibrated 2-14-26
    public static double hoodMaximumPos = 0.54; // calibrated 1-22-26
    public static double hoodMaximumRadians = Math.toRadians(72); // calibrated 2-14-26
    public static double hoodWriteMargin = 0.01; // if our hood position change is less than this, don't waste time with a write

    /** turret tunables (used in Turret.java) **/
    public static double turretCenterOffset = -0.11; // 0 turret position as read in ServoTuner OpMode (make sure to start turret straight forward so our encoder is accurate)
    public static double turretMaxLeft = 1; // max range that the turret can go left or right from servo center
    public static double turretMaxRight = 1;
    // TODO: fix NaN in Turret.java

    /** transfer tunables (used in Transfer.java) **/

    public static double transferMotorForwardPower = 1;
    public static double transferMotorReversePower = -1;

    // servo position where upper transfer prevents balls from passing into launch
    public static double transferServoClosed = 0.00; // recalibrated 2-17-26
    // servo position where upper transfer allows balls to pass into launch
    public static double transferServoOpen = 0.34; // recalibrated 2-17-25


    /** TeleOp tunables (used in BozoTeleOp.java) **/

    public static double turnRateMultiplier = 0.75; // always have our turns 75% speed
    public static int adjustRPM = 50; // driver increments/decrements by adjustRPM
    public static double initialManualLaunchRPM = 2350; // 2400 is a little too much
    public static boolean useBrakes = true; // whether to use brakes in TeleOp
    public static double farZoneDataStart = 95; // if d > this, use far zone data

    /** auto tunables (used in BozoAuto.java) **/

    public static double scoreRPM = 2250; // RPM to set for launching (stolen from teleop)
    public static double scoreHoodPos = 0.171; // hood position for launching in auto
    public static double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double clearTime = 1000; // amount of ms to wait for clear
    public static double launchDistanceMargin = 2; // must be within this amount of inches to shoot
    public static double maxGrabVelocity = 50; // max velocity while grabbing balls in inches/second
    public static double clearMaxPower = 0.8;
    public static double magicOffset = 0.12;

    /** vision tunables (used in Vision.java) **/
    public static long maxVisionStaleness = 50; // amount of millis without a reading where vision becomes stale
    public static double goalOffset = -30; // TODO:

    // offsets for translating from limelight->field position
    // TODO: calibrate these
    //public static double limelightOffsetZ = 0;
    public static double maxLimelightHeadingError = Math.toRadians(5); // if our limelight reports a heading that differs by more than this, discard that measurement
    public static double turretOffsetX = 0;
    public static double turretOffsetY = 0;
    public static double limelightTurretRadius = 0; // how far limelight is mounted from center of turret in inches

    /** fusion tunables (used in Fusion.java) **/

    public static double modelCovariance = 0.05; // odo noise
    public static double dataCovariance = 0.01; // vision noise

    public static double maxVisionVariance = 10; // reject vision estimates with a euclidean distance difference greater than this from odo
}
