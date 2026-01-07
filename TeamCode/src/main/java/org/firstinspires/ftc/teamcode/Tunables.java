package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;

@Configurable
public class Tunables { // this should hold all of our constants
    /** Robot tunables (used in Robot.java) **/

    // this needs to be calculated+changed every time you modify the launch ratio
    public static double launchRatio = (double) 16 / 20; // this is correct because 5202-0002-0001's gearbox ratio is 1:1, and we go from a 16tooth -> 20tooth pulley
    // if intake has a velocity that is less than intakeStallVelocity or a current greater than intakeOvercurrent then we consider it stalled
    public static double intakeOvercurrent = 6; // amount of amps that we think means a stalled intake
    public static double magicNumber = (double) 3 / 8; // magic number for auto RPM

    // PIDF coefficients
    public static double launchP = 100; // the P is too high when on full-charge batteries but 300 is about right for slightly discharged batteries
    public static double launchI = 0.1; // orig 0.1
    public static double launchD = 0; // orig 0.2
    public static double launchF = 20; // ChatGPT was onto nothing with this

    // servo open/close points (don't find these with the backplate on!)
    public static double lowerTransferLowerLimit = 0.00; // recalibrated 1-6-25
    public static double lowerTransferUpperLimit = 0.30; // recalibrated 1-6-25
    // servo position where upper transfer prevents balls from passing into launch
    public static double upperTransferClosed = 0.38; // recalibrated 1-6-25
    // servo position where upper transfer allows balls to pass into launch
    public static double upperTransferOpen = 0.00; // recalibrated 1-6-25

    // distance sensor limits (better to undershoot rather than to overshoot)
    public static double intakeSensorOpen = 210; // amount of mm's the intake sensor should report if there is no ball
    public static double lowerTransferSensorOpen = 140; // needs to be different than upper transfer because color sensor distance measurements don't exceed 30mm :(
    public static double upperTransferSensorOpen = 140; // amount of mm's that upper transfer sensor  should report if there is no ball

    // delays
    public static int openDelay = 150; // time to wait for upperTransfer to open (in millis)
    public static int maxTransferDelay = 200; // maximum time to wait for ball to enter lower transfer
    public static int maxPushDelay = 250; // maximum time to wait for lowerTransfer to move (in millis)

    public static double scoreMargin = 100; // margin of 100TPS; TODO: tune this
    public static int intakeOvercurrentDelay = 250; // if intake has been overcurrent for more than this many millis, we consider it full

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
    public static double initialLaunchRPM = 2350; // 2400 is a little too much
    public static boolean useBrakes = true; // whether to use brakes in TeleOp
    public static boolean holdEnd = true; // whether to hold end while shooting
    public static double launchTurnMargin = Math.toRadians(5); // margin we want to get our turn to for launch

    /** Auto tunables (used in BozoAuto.java) **/

    public static double scoreRPM = 2300; // RPM to set for launching (stolen from teleop)
    public static double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static int beginningLaunchDelay =  200; // time to wait before launching first ball
    public static double launchDistanceMargin = 2; // must be within this amount of inches to shoot

    /** Vision tunables (used in Vision.java) **/
    public static double turnP = 1;
    public static double turnI = 0;
    public static double turnD = 0;

    public static long maxVisionStaleness = 20; // amount of millis without a reading where vision becomes stale
    public static double goalOffset = 0; // how many inches less our vision reports the goal to be at than it actually is
}
