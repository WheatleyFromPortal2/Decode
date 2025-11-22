/*
this is basically our mega-class that holds all robot data that is shared between auto and teleop
 */
package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.pedropathing.geometry.Pose;

// Pedro Pathing imports


public class Robot { // create our global class for our robot
    public static Pose goalPose; // this must be initialized by the auto
    public static Pose switchoverPose; // this must be initialized by the auto and is used to persist our current position from auto->TeleOp
    private static Robot instance;
    public DcMotorEx intake, launch; // drive motors are handled by Pedro Pathing
    public Servo lowerTransfer, upperTransfer;

    private Timer launchStepTimer;
    private enum LaunchState {
        START,
        OPENING_UPPER_TRANSFER,
        PUSHING_LOWER_TRANSFER,
        WAITING_FOR_EXIT,
    }
    LaunchState launchState = LaunchState.START; // set our launch state to start

    public static final int TICKS_PER_REV = 28; // REV Robotics 5203/4 series motors have 28ticks/revolution
    public double neededLaunchVelocity; // this stores our needed launch velocity, used to check if we're in range

    // this needs to be calculated+changed every time you modify the launch ratio
    public static final double launchRatio = (double) 16 / 20; // this is correct because 5202-0002-0001's gearbox ratio is 1:1, and we go from a 16tooth -> 20tooth pulley

    // servo open/close points (don't find these with the backplate on!)
    public static final double upperTransferClosed = 0.36; // servo position where upper transfer prevents balls from passing into launch
    public static final double upperTransferOpen = 0.70; // servo position where upper transfer allows balls to pass into launch

    /** stuff to tune **/

    // PIDF coefficients
    public static final double launchP = 300; // the P is too high when on full-charge batteries but 300 is about right for slightly discharged batteries
    public static final double launchI = 0.1; // orig 0.1
    public static final double launchD = 0.2; // orig 0.2
    public static final double launchF = (double) 1 / 2800; // 6000 rpm motor; 2333.333333333333 ideal
    public static final double lowerTransferLowerLimit = 0.28;
    public static final double lowerTransferUpperLimit = 0.49;

    // delays
    public static final int openDelay = 150; // time to wait for upperTransfer to open (in millis)
    public static final int pushDelay = 180; // time to wait for lowerTransfer to move (in millis)
    public static final int firstInterLaunchWait = 75; // time to wait between 1st and 2nd launches
    public static final int lastInterLaunchWait = 200; // time to wait between the 2nd and last launch
    public final double scoreMargin = 100; // margin of 100TPS; TODO: tune this

    public Robot(HardwareMap hw) { // create all of our hardware
        // DC motors (all are DcMotorEx for current monitoring)
        intake = hw.get(DcMotorEx.class, "intake");
        launch = hw.get(DcMotorEx.class, "launch");

        // servos
        lowerTransfer = hw.get(Servo.class, "lowerTransfer");
        upperTransfer = hw.get(Servo.class, "upperTransfer");

        // sensors
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launch.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder
        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        //PIDFCoefficients pidfOrig = launch.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(launchP, launchI, launchD, launchF);
        launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // set up our timer for the launch state machine
        launchStepTimer = new Timer(); // tracks time since we started our last launch
    }

    public static Robot getInstance(HardwareMap hw) { // this allows us to preserve the Robot instance from auto->teleop
        if (instance == null) {
            instance = new Robot(hw);
        }
        return instance;
    }

    public double getDstFromGoal(Pose currentPosition) {
        double xDst = Math.abs(currentPosition.getX() - goalPose.getX());
        double yDst = Math.abs(currentPosition.getY() - goalPose.getY());
        return Math.pow(Math.pow(xDst, 2) + Math.pow(yDst, 2), 0.5); // use pythag to find dst from goal
    }

    public double getGoalHeading(Pose currentPosition) { // return bot heading to point towards goal in radians
        double xDst = goalPose.getX() - currentPosition.getX();
        double yDst = goalPose.getY() - currentPosition.getY();
        return Math.atan2(yDst, xDst);
    }

    public double getTangentialSpeed(Pose currentPosition) { // returns needed tangential speed to launch ball to the goal
        double d = getDstFromGoal(currentPosition);
        double numerator = 19.62 * Math.pow(d, 2);
        double denominator = (Math.pow(3, 0.5) * d) - 0.8;
        return Math.pow(numerator / denominator, 0.5); // thank u rahul
    }

    public double getNeededVelocity(double tangentialSpeed) { // input tangentialSpeed (in m/s) and set launch velocity to have ball shoot at that speed
        double TPS = 0;
        // this will be the hardest function to code
        // it basically needs to be a relation between the rotational speed of launch and the actual output speed of the ball
        // at the end, we will output the desired TPS of our motor to monitor once it reaches it
        return TPS;
    }

    public void setAutomatedLaunchVelocity(Pose currentPosition) {
        double neededTangentialSpeed = getTangentialSpeed(currentPosition);
        double neededVelocity = getNeededVelocity(neededTangentialSpeed);
        launch.setVelocity(neededVelocity);
    }

    public double TPSToRPM(double TPS) {
        return (TPS / TICKS_PER_REV) * 60 * launchRatio;
    }
    public double RPMToTPS(double RPM) { return (RPM * TICKS_PER_REV / 60) / launchRatio;}
    public double getLaunchRPM() { // return launch velocity in RPM
        return TPSToRPM(launch.getVelocity());
    }
    public double getLaunchRadians() { // return launch velocity in radians/second
        return ((launch.getVelocity() / TICKS_PER_REV) * 2 * Math.PI);
    }
    public double getLaunchCurrent() { // return launch current in amps
        return launch.getCurrent(CurrentUnit.AMPS);
    }
    public boolean isLaunchWithinMargin() {
        if (neededLaunchVelocity == 0) return true; // if our needed launch velocity is 0 (off) then we're within range
        return Math.abs(neededLaunchVelocity - launch.getVelocity()) < scoreMargin; // measure if our launch velocity is within our margin of error
    }
    public double getIntakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
    public void initServos() { // set servos to starting state
        upperTransfer.setPosition(Robot.upperTransferClosed); // make sure balls cannot launch
        lowerTransfer.setPosition(Robot.lowerTransferLowerLimit); // make sure lower transfer is not getting in the way
    }
    public void launchBall() throws InterruptedException { // launch a ball
        // TODO: make this asynchronous (eliminate all the waits)
        upperTransfer.setPosition(upperTransferOpen);
        sleep(openDelay); // allow time for upper transfer to move
        lowerTransfer.setPosition(lowerTransferUpperLimit);
        sleep(pushDelay); // allow time for lower transfer to move
        // hopefully the ball has launched by now
        upperTransfer.setPosition(upperTransferClosed); // close upper transfer
        lowerTransfer.setPosition(lowerTransferLowerLimit); // set lower transfer to its lowest
    }

    public void setLaunchVelocity(double velocity) { // velocity is in TPS
        neededLaunchVelocity = velocity; // update our desired launch velocity
        launch.setVelocity(velocity); // set our launch velocity
    }
    public void launchOff() { // turn launch off
        neededLaunchVelocity = 0; // we don't need any launch velocity
        launch.setPower(0);
        launch.setVelocity(0); // prob don't need this but ok
    }
    public boolean updateLaunch() { // outputs true/false whether we are done with launching
        switch (launchState) {
            case START:
                upperTransfer.setPosition(Robot.upperTransferOpen);
                launchStepTimer.resetTimer();
                launchState = LaunchState.OPENING_UPPER_TRANSFER;
                return false;
            case OPENING_UPPER_TRANSFER:
                if (launchStepTimer.getElapsedTime() >= Robot.openDelay) { // we've given it openDelay millis to open
                    lowerTransfer.setPosition(Robot.lowerTransferUpperLimit);
                    launchStepTimer.resetTimer();
                    launchState = LaunchState.PUSHING_LOWER_TRANSFER;
                }
                return false;
            case PUSHING_LOWER_TRANSFER:
                if (launchStepTimer.getElapsedTime() >= Robot.pushDelay) {
                    lowerTransfer.setPosition(Robot.lowerTransferLowerLimit);
                    upperTransfer.setPosition(Robot.upperTransferClosed);
                    launchStepTimer.resetTimer();
                    launchState = LaunchState.WAITING_FOR_EXIT;
                }
                return false;
            case WAITING_FOR_EXIT:
                if (launchStepTimer.getElapsedTime() >= Robot.firstInterLaunchWait) { // ideally with this we won't need to separate first/last inter launch delay
                    launchState = LaunchState.START; // get ready for next one
                    return true;
                }
                return false;
        }
        return false; // this code should never be reached, but the IDE freaks out if i don't have it
    }
}
