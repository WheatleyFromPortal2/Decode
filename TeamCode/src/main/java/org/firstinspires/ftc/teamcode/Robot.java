/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/
package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.geometry.Pose;


public class Robot { // create our global class for our robot
    public static final int TICKS_PER_REV = 28; // REV Robotics 5203/4 series motors have 28ticks/revolution
    public static Pose switchoverPose; // this must be initialized by the auto and is used to persist our current position from auto->TeleOp
    private static Robot instance;
    public DcMotorEx intake, launch; // drive motors are handled by Pedro Pathing, thus we only need our intake and launch motors
    public Servo lowerTransfer, upperTransfer;
    public Rev2mDistanceSensor intakeSensor, lowerTransferSensor1, lowerTransferSensor2, upperTransferSensor1, upperTransferSensor2; // all of our distance sensors for detecting balls

    private Timer launchStateTimer; // this timer measures the time between states in launch

    private enum LaunchState {
        START,
        OPENING_UPPER_TRANSFER,
        PUSHING_LOWER_TRANSFER,
        WAITING_FOR_EXIT,
    }

    /** only these variables should change during runtime **/
    LaunchState launchState = LaunchState.START; // set our launch state to start

    public double neededLaunchVelocity; // this stores our needed launch velocity, used to check if we're in range

    private boolean isLaunching = false; // since we are now using ballsRemaining to see how many balls we have, we need this to track when we actually want to launch
    private int ballsRemaining = 0; // tracks how many balls are in the robot
    private boolean wasBallInIntake = false; // this tracks whether we had a ball in intake last time we checked, use to calculate whether we have gathered all of our balls

    public Robot(HardwareMap hw) { // create all of our hardware and initialize our class
        // DC motors (all are DcMotorEx for current monitoring)
        intake = hw.get(DcMotorEx.class, "intake");
        launch = hw.get(DcMotorEx.class, "launch");

        intake.setDirection(DcMotorEx.Direction.FORWARD);
        launch.setDirection(DcMotorEx.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor

        intake.setCurrentAlert(Tunables.intakeOvercurrent, CurrentUnit.AMPS);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder

        // servos
        lowerTransfer = hw.get(Servo.class, "lowerTransfer");
        upperTransfer = hw.get(Servo.class, "upperTransfer");

        // sensors
        intakeSensor = hw.get(Rev2mDistanceSensor.class, "intakeSensor");
        lowerTransferSensor1 = hw.get(Rev2mDistanceSensor.class, "lowerTransferSensor1");
        lowerTransferSensor2 = hw.get(Rev2mDistanceSensor.class, "lowerTransferSensor2");
        upperTransferSensor1 = hw.get(Rev2mDistanceSensor.class, "upperTransferSensor1");
        upperTransferSensor2 = hw.get(Rev2mDistanceSensor.class, "upperTransferSensor2");

        // Change PIDF coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(Tunables.launchP, Tunables.launchI, Tunables.launchD, Tunables.launchF); // use our coefficients from Tunables.java
        launch.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew); // apply our coefficients to our motor

        // set up our timers for the launch state machine
        launchStateTimer = new Timer(); // tracks time since we started our last launch state
    }

    public static Robot getInstance(HardwareMap hw) { // this allows us to preserve the Robot instance from auto->teleop
        if (instance == null) {
            instance = new Robot(hw);
        }
        return instance;
    }

    /** math methods **/
    public double getDstFromGoal(Pose currentPosition, Pose goalPose) {
        double xDst = Math.abs(currentPosition.getX() - goalPose.getX());
        double yDst = Math.abs(currentPosition.getY() - goalPose.getY());
        return Math.pow(Math.pow(xDst, 2) + Math.pow(yDst, 2), 0.5); // use pythag to find dst from goal
    }

    public double getGoalHeading(Pose currentPosition, Pose goalPose) { // return bot heading to point towards goal in radians
        double xDst = goalPose.getX() - currentPosition.getX();
        double yDst = goalPose.getY() - currentPosition.getY();
        return Math.atan2(yDst, xDst);
    }

    public double getTangentialSpeed(Pose currentPosition, Pose goalPose) { // returns needed tangential speed to launch ball to the goal
        double d = getDstFromGoal(currentPosition, goalPose);
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

    public void setAutomatedLaunchVelocity(Pose currentPosition, Pose goalPose) {
        double neededTangentialSpeed = getTangentialSpeed(currentPosition, goalPose);
        double neededVelocity = getNeededVelocity(neededTangentialSpeed);
        launch.setVelocity(neededVelocity);
    }

    public double TPSToRPM(double TPS) { return (TPS / TICKS_PER_REV) * 60 * Tunables.launchRatio; }
    public double RPMToTPS(double RPM) { return (RPM * TICKS_PER_REV / 60) / Tunables.launchRatio;}

    /** hardware methods **/
    public void initServos() { // set servos to starting state
        upperTransfer.setPosition(Tunables.upperTransferClosed); // make sure balls cannot launch
        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit); // make sure lower transfer is not getting in the way
    }
    public double getIntakeCurrent() { return intake.getCurrent(CurrentUnit.AMPS); } // return intake current in amps
    public boolean isIntakeOvercurrent() {
        return intake.getCurrent(CurrentUnit.AMPS) >= Tunables.intakeOvercurrent;
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
    public void launchOff() { // turn launch off
        neededLaunchVelocity = 0; // we don't need any launch velocity
        launch.setPower(0);
        launch.setVelocity(0); // prob don't need this but ok
    }
    public boolean isBallInIntake() { // return true if there is a ball reducing our measured distance
        return intakeSensor.getDistance(DistanceUnit.MM) < Tunables.intakeOpen;
    }
    public boolean isBallInLowerTransfer() { // return true if there is a ball reducing our measured distance
        return lowerTransferSensor1.getDistance(DistanceUnit.MM) < Tunables.transferOpen // a hole in the ball could be allowing a sensor to report a false negative, so we need to check both
                || lowerTransferSensor2.getDistance(DistanceUnit.MM) < Tunables.transferOpen;
    }
    public boolean isBallInUpperTransfer() { // return true if there is a ball reducing our measured distance
        return upperTransferSensor1.getDistance(DistanceUnit.MM) < Tunables.transferOpen // a hole in the ball could be allowing a sensor to report a false negative, so we need to check both
                || upperTransferSensor2.getDistance(DistanceUnit.MM) < Tunables.transferOpen;
    }

    /** ball launching methods **/
    public void launchBalls() { // sets to launch this many balls
        isLaunching = true;
        launchStateTimer.resetTimer(); // reset launch state timer (it may be off if cancelled)
        launchState = LaunchState.START;
    }

    public void cancelLaunch() { // set servos to default position, this could break if activated at the right time
        isLaunching = false; // stop launching
        ballsRemaining = 0;
        upperTransfer.setPosition(Tunables.upperTransferClosed); // make sure balls can't accidentally be launched
        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit); // allow robot to store all balls
    }

    public boolean updateLaunch() { // outputs true/false whether we are done with launching
        if (!isBallInLowerTransfer()) { ballsRemaining = 0; } // if we don't have a ball in lower transfer, we don't have any balls, let's not waste our time

        if (ballsRemaining == 0) {
            isLaunching = false;
            return true; // we're done with launching balls
        } else if (isLaunching) { // balls remaining > 0 && we are launching
            switch (launchState) {
                case START:
                    upperTransfer.setPosition(Tunables.upperTransferOpen);
                    launchStateTimer.resetTimer();
                    launchState = LaunchState.OPENING_UPPER_TRANSFER;
                case OPENING_UPPER_TRANSFER:
                    if (launchStateTimer.getElapsedTime() >= Tunables.openDelay) { // we've given it openDelay millis to open
                        lowerTransfer.setPosition(Tunables.lowerTransferUpperLimit);
                        launchStateTimer.resetTimer();
                        launchState = LaunchState.PUSHING_LOWER_TRANSFER;
                    }
                case PUSHING_LOWER_TRANSFER: // TODO: merge PUSHING_LOWER_TRANSFER and WAITING_FOR_EXIT to waste as little time as possible
                    if (launchStateTimer.getElapsedTime() >= Tunables.pushDelay) {
                        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit);
                        upperTransfer.setPosition(Tunables.upperTransferClosed);
                        launchStateTimer.resetTimer();
                        launchState = LaunchState.WAITING_FOR_EXIT;
                    }
                case WAITING_FOR_EXIT:
                    if (isBallInUpperTransfer()) { // wait until we detect a ball in upper transfer (ball has been launched)
                        launchState = LaunchState.START; // get ready for next one
                        ballsRemaining -= 1; // we've launched a ball
                        isLaunching = false; // we are no longer launching
                    }
            }
        }
        return false; // we're still working on launching balls
    }

    public void updateBalls() { // checks our intake sensor and updates our balls
        boolean ballInIntake = isBallInIntake();
        if (!wasBallInIntake && ballInIntake) ballsRemaining++; // if we previously didn't have a ball in intake, and we do now, then increment our remaining balls
        wasBallInIntake = ballInIntake; // update our reading
    }

    public int getBallsRemaining() { return ballsRemaining; }
    public boolean isLaunching() { return isLaunching; }
}
