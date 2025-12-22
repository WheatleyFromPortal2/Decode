/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/
package org.firstinspires.ftc.teamcode;

// hardware imports

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// unit imports
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

// Pedro Pathing imports
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;


public class Robot { // create our global class for our robot
    public static final int TICKS_PER_REV = 28; // REV Robotics 5203/4 series motors have 28ticks/revolution
    // TODO: replace switchover pose with inverting controller inputs to avoid modifying robot heading - driver will need to point a different way for adjusting heading
    public static Pose switchoverPose; // this must be initialized by the auto and is used to persist our current position from auto->TeleOp
    private static Robot instance; // this stores our current instance of Robot, so we can transfer it from auto->TeleOp
    public DcMotorEx intake, launch; // drive motors are handled by Pedro Pathing
    public Servo lowerTransfer, upperTransfer; // servos
    public Rev2mDistanceSensor intakeSensor, lowerTransferSensor, upperTransferSensor; // all of our distance sensors for detecting balls

    private Timer launchStateTimer, // tracks time since we started our last launch state
            intakeTimer, // measures time since last intake measurement
            intakeOvercurrentTimer, // measures time intake has been overcurrent
            transferTimer, // measure time since ball was launched to see how long we need to wait for transfer
            launchIntervalTimer; // this timer measures the time between individual launches
    private enum LaunchState { // these are the possible states our launch state machine can be in
        START,
        OPENING_UPPER_TRANSFER,
        WAITING_FOR_EXIT,
    }

    /** only these variables should change during runtime **/
    LaunchState launchState = LaunchState.START; // set our launch state to start

    public double neededLaunchVelocity; // this stores our needed launch velocity, used to check if we're in range

    private boolean isLaunching = false; // since we are now using ballsRemaining to see how many balls we have, we need this to track when we actually want to launch
    private int ballsRemaining = 0; // tracks how many balls are in the robot
    private boolean wasBallInIntake = false; // this tracks whether we had a ball in intake last time we checked, use to calculate whether we have gathered all of our balls
    private double lastLaunchInterval; // stores the amount of time it took for our last launch

    public Robot(HardwareMap hw) { // create all of our hardware and initialize our class
        // DC motors (all are DcMotorEx for current monitoring)
        intake = hw.get(DcMotorEx.class, "intake"); // intake motor
        launch = hw.get(DcMotorEx.class, "launch"); // launch motor, connected with launchRatio

        // servos
        lowerTransfer = hw.get(Servo.class, "lowerTransfer");
        upperTransfer = hw.get(Servo.class, "upperTransfer");

        // sensors
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        launch.setDirection(DcMotorEx.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        intake.setCurrentAlert(Tunables.intakeOvercurrent, CurrentUnit.AMPS);
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor

        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder

        // distance sensors
        intakeSensor = hw.get(Rev2mDistanceSensor.class, "intakeSensor");
        lowerTransferSensor = hw.get(Rev2mDistanceSensor.class, "lowerTransferSensor");
        upperTransferSensor = hw.get(Rev2mDistanceSensor.class, "upperTransferSensor");

        // Change PIDF coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(Tunables.launchP, Tunables.launchI, Tunables.launchD, Tunables.launchF); // use our coefficients from Tunables.java
        launch.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew); // apply our coefficients to our motor


        launchStateTimer = new Timer(); // set up timer for the launch state machine
        intakeTimer = new Timer(); // set up timer to measure balls in intake
        intakeOvercurrentTimer = new Timer();
        launchIntervalTimer = new Timer();
        transferTimer = new Timer();
    }

    public static Robot getInstance(HardwareMap hw) { // this allows us to preserve the Robot instance from auto->teleop
        if (instance == null) { // we don't already have an instance
            instance = new Robot(hw); // create a new instance
        }
        return instance;
    }
    public void updatePIDF() {
        PIDFCoefficients pidfNew = new PIDFCoefficients(Tunables.launchP, Tunables.launchI, Tunables.launchD, Tunables.launchF);
        launch.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
    }
    public double getDstFromGoal(Pose currentPosition, Pose goalPose) { // get our distance from the goal in inches
        double inchesD = currentPosition.distanceFrom(goalPose); // use poses to find our distance easily :)
        return inchesD * 0.0254; // convert to meters
    }

    public double getGoalHeading(Pose currentPosition, Pose goalPose) { // return bot heading to point towards goal in radians
        // TODO: fix this only being able to turn counterclockwise
        double xDst = goalPose.getX() - currentPosition.getX();
        double yDst = goalPose.getY() - currentPosition.getY();
        double desiredHeading = Math.atan2(yDst, xDst); // need atan2 to account for negatives
        double currentHeading = normalizeRadians(currentPosition.getHeading());
        return normalizeRadians(desiredHeading - currentHeading) + currentHeading;
    }

    public double getTangentialSpeed(Pose currentPosition, Pose goalPose) { // returns needed tangential speed to launch ball to the goal
        double d = getDstFromGoal(currentPosition, goalPose);
        double fraction = (4.9)/((d * 1.73205) - 0.83);
        double beforeMagicNumber = Math.pow(fraction, 0.5)* 2 * d;
        return beforeMagicNumber * Tunables.magicNumber;
    }

    public void setAutomatedLaunchVelocity(Pose currentPosition, Pose goalPose) { // given positions, use our functions to set our launch speed
        double neededTangentialSpeed = getTangentialSpeed(currentPosition, goalPose);
        double neededVelocity = getNeededVelocity(neededTangentialSpeed);
        setLaunchVelocity(neededVelocity);
    }

    public double TPSToRPM(double TPS) { return (TPS / TICKS_PER_REV) * 60 * Tunables.launchRatio; }
    public double RPMToTPS(double RPM) { return (RPM * TICKS_PER_REV / 60) / Tunables.launchRatio;}

    /** hardware methods **/
    public void resetServos() { // set servos to starting state
        upperTransfer.setPosition(Tunables.upperTransferClosed); // make sure balls cannot launch
        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit); // make sure lower transfer is not getting in the way
    }
    public double getIntakeCurrent() { return intake.getCurrent(CurrentUnit.AMPS); } // return intake current in amps
    public boolean isFull() {
        return ballsRemaining >= 3;
    }
    public double getLaunchRPM() { return TPSToRPM(launch.getVelocity()); } // return launch velocity in RPM
    public double getLaunchCurrent() { return launch.getCurrent(CurrentUnit.AMPS); } // return launch current in amps
    public boolean isLaunchWithinMargin() {
        if (neededLaunchVelocity == 0) return true; // if our needed launch velocity is 0 (off) then we're within range
        return Math.abs(neededLaunchVelocity - launch.getVelocity()) < Tunables.scoreMargin; // measure if our launch velocity is within our margin of error
    }

    public void setLaunchVelocity(double velocity) { // velocity is in TPS
        neededLaunchVelocity = velocity; // update our desired launch velocity
        launch.setVelocity(velocity); // set our launch velocity
    }
    public double getNeededVelocity(double tangentialSpeed) { // input tangentialSpeed (in m/s) and set launch velocity to have ball shoot at that speed
        double numerator = tangentialSpeed - 0.269926;
        double RPM = (numerator/0.000636795);
        return RPMToTPS(RPM); // return our TPS
    }
    public boolean isBallInIntake() { // return true if there is a ball reducing our measured distance
        return intakeSensor.getDistance(DistanceUnit.MM) < Tunables.intakeSensorOpen;
    }
    public boolean isBallInLowerTransfer() { // return true if there is a ball reducing our measured distance
        return lowerTransferSensor.getDistance(DistanceUnit.MM) < Tunables.lowerTransferSensorOpen; // a hole in the ball could be allowing a sensor to report a false negative, so we need to check both
    }
    public boolean isBallInUpperTransfer() { // return true if there is a ball reducing our measured distance
        return upperTransferSensor.getDistance(DistanceUnit.MM) < Tunables.upperTransferSensorOpen; // a hole in the ball could be allowing a sensor to report a false negative, so we need to check both
    }

    /** ball launching methods **/
    public void launchBalls(int balls) { // sets to launch this many balls
        ballsRemaining = balls;
        isLaunching = true; // we are launching now
        launchStateTimer.resetTimer(); // reset launch state timer (it may be off if cancelled)
        launchState = LaunchState.START; // reset our state machine to the start
    }

    public void cancelLaunch() { // set servos to default position, this could break if activated at the right time
        isLaunching = false; // stop launching
        ballsRemaining = 0;
        upperTransfer.setPosition(Tunables.upperTransferClosed); // make sure balls can't accidentally be launched
        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit); // allow robot to store all balls
    }

    public boolean updateLaunch() { // outputs true/false whether we are done with launching
        if (ballsRemaining == 0) {
            isLaunching = false;
            return true; // we're done with launching balls
        } else if (isLaunching) { // balls remaining > 0 && we are launching
            switch (launchState) {
                case START:
                    upperTransfer.setPosition(Tunables.upperTransferOpen);
                    launchStateTimer.resetTimer();
                    launchState = LaunchState.OPENING_UPPER_TRANSFER;
                    launchIntervalTimer.resetTimer(); // start measuring our time for this launch
                    break;
                case OPENING_UPPER_TRANSFER:
                    if (!isBallInLowerTransfer() // if we don't have a ball in lower transfer
                            && !isBallInIntake() // AND we don't have a ball waiting in intake
                            && launchStateTimer.getElapsedTime() >= Tunables.maxTransferDelay ) { // AND we have waited our max time for transfer to happen, we don't have any balls, let's not waste our time
                        ballsRemaining = 0;
                        launchState = LaunchState.START;
                        isLaunching = false;
                        return true;
                    }
                    if (launchStateTimer.getElapsedTime() >= Tunables.openDelay) { // we've given it openDelay millis to open
                        lowerTransfer.setPosition(Tunables.lowerTransferUpperLimit);
                        launchStateTimer.resetTimer();
                        launchState = LaunchState.WAITING_FOR_EXIT;
                    }
                    break;
                case WAITING_FOR_EXIT:
                    if (isBallInUpperTransfer() // wait until we detect a ball in upper transfer (ball has been launched)
                            || launchStateTimer.getElapsedTime() >= Tunables.maxPushDelay) { // or if that hasn't happened in a while, just go to the next launch
                        resetServos(); // reset our servos
                        launchState = LaunchState.START; // get ready for next one
                        ballsRemaining -= 1; // we've launched a ball
                        launchStateTimer.resetTimer(); // reset our timer
                        lastLaunchInterval = launchIntervalTimer.getElapsedTime();
                        transferTimer.resetTimer(); // we are starting transfer for next ball
                    }
                    break;
            }
        }
        return false;
    }

    public void updateBalls() { // checks our intake sensor and updates our balls
        // we want to only read from our sensors once, because they take a long time to read from
        boolean ballInIntake = isBallInIntake();
        boolean ballInLowerTransfer = isBallInLowerTransfer();

        if (!wasBallInIntake && ballInIntake) ballsRemaining++; // if we previously didn't have a ball in intake, and we do now, then increment our remaining balls
        if (ballsRemaining > 3) { ballsRemaining = 3; } // shouldn't ever have >3balls
        if (ballsRemaining == 3 && !ballInIntake) { ballsRemaining = 2; }

        if (!ballInLowerTransfer // if we are missing a ball in lower transfer
                && !ballInIntake // AND we don't have a ball waiting in intake
                && transferTimer.getElapsedTime() >= Tunables.maxTransferDelay) { // AND we have exceeded our max time for transfer
            ballsRemaining = 0; // we can safely say we are out of balls
        }

        if (ballsRemaining == 0 && ballInLowerTransfer) { ballsRemaining = 1; } // if we have a ball in lower transfer, then we probably have at least 1 ball

        if (intake.getCurrent(CurrentUnit.AMPS) <= Tunables.intakeOvercurrent) { // our intake is not overcurrent
            intakeOvercurrentTimer.resetTimer(); // reset our overcurrent timer
        } else {
            if (intakeOvercurrentTimer.getElapsedTime() >= Tunables.intakeOvercurrentDelay) { // our intake has been full for intakeOvercurrentDelay
                ballsRemaining = 3; // our intake is full, so we have 3 balls
            }
        }
        intakeTimer.resetTimer(); // wait to check for a while
        wasBallInIntake = ballInIntake; // update our reading at the end
    }

    public int getBallsRemaining() { return ballsRemaining; }
    public double getLastLaunchInterval() { return lastLaunchInterval; }
    public boolean isLaunching() { return isLaunching; }
}
