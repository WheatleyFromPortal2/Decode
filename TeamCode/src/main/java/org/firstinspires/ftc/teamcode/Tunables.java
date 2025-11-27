package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;

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

    public static double intakeOvercurrent = 6; // amount of amps that we think means a stalled intake

    // PIDF coefficients
    public static   double launchP = 300; // the P is too high when on full-charge batteries but 300 is about right for slightly discharged batteries
    public static   double launchI = 0.1; // orig 0.1
    public static   double launchD = 0.2; // orig 0.2
    public static   double launchF = (double) 1 / 2800; // 6000 rpm motor; 2333.333333333333 ideal

    // servo open/close points (don't find these with the backplate on!)
    public static double lowerTransferLowerLimit = 0.28;
    public static double lowerTransferUpperLimit = 0.49;
    public static double upperTransferClosed = 0.36; // servo position where upper transfer prevents balls from passing into launch
    public static double upperTransferOpen = 0.70; // servo position where upper transfer allows balls to pass into launch

    // delays
    public static int openDelay = 150; // time to wait for upperTransfer to open (in millis)
    public static int pushDelay = 180; // time to wait for lowerTransfer to move (in millis)
    public static int firstInterLaunchWait = 75; // time to wait between 1st and 2nd launches
    public static int lastInterLaunchWait = 200; // time to wait between the 2nd and last launch
    public static double scoreMargin = 100; // margin of 100TPS; TODO: tune this

    /** TeleOp tunables (used in BozoTeleOp.java) **/

    public static double turnRateMultiplier = 0.75; // always have our turns 75% speed
    public static int adjustRPM = 50; // driver increments/decrements by adjustRPM
    public static double initialLaunchRPM = 2300; // maybe 2500; from crease
    public static int intakePollingRate = 50; // test if intake is stalled every 50millis
    public static boolean useBrakes = true; // whether to use brakes in TeleOp
    public static boolean holdEnd = true; // whether to hold end while shooting

    /** Auto tunables (used in BozoAuto.java) **/

    public static double scoreRPM = 2400; // RPM to set for launching (stolen from teleop)
    public static double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    public static int beginningLaunchDelay =  100; // time to wait before launching first ball
    public static double launchDistanceMargin = 2; // must be within this amount of inches to shoot

    /** Vision tunables (used in Vision.java) **/
    // processor variables
    public static Position cameraPosition = new Position(
            DistanceUnit.MM, // we use MM in cad, so this just make sense
            140, // taken from CAD
            0, // our camera should be centered
            233, // taken from CAD
            0 // this has no acquisition time
    );
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,
            10, // TODO: calibrate this
            0,
            0 // no acquisition time
    );
    public static int decimation = 2; // TODO: tune this

    // logic variables
    public static int visionUpdateInterval = 50; // update vision every x millis
    public static int aprilTagUpdateInterval = 20; // update apriltags from vision every x millis
    public static double maxPoseJumpDistance = 3.0; // only ever change our position this many inches based off of vision data
    public static double maxTagDisagreement = 2.0; // tags should only disagree by 2 inches
    public static double maxHeadingJump = Math.toRadians(15); // only ever change our heading by this many radians based off of vision data
    // left corner tag information

    // testing variables
    public static boolean drawAxis = true;
    public static boolean drawCubeProjections = true;
    public static boolean drawTagOutline = true;
    public static boolean drawTagID = true;
    public static boolean enableLiveView = true;
    public static Size cameraResolution = new Size(640, 480);
    public static VisionPortal.StreamFormat streamFormat = VisionPortal.StreamFormat.YUY2; // MPJEG is faster if we don't have enough bandwidth

}
