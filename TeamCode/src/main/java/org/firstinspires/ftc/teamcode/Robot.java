/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/
package org.firstinspires.ftc.teamcode;

// hardware imports
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

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

    private Timer launchStateTimer, // this timer measures the time between states in launch
            launchIntervalTimer; // this timer measures the time between individual launches
    private enum LaunchState { // these are the possible states our launch state machine can be in
        START,
        OPENING_UPPER_TRANSFER, // waiting for upper transfer to be open so we can push a ball through
        PUSHING_LOWER_TRANSFER, // waiting for lower transfer to push up fully
        WAITING_FOR_EXIT, // waiting for the ball to exit
    }

    /** only these variables should change during runtime **/
    LaunchState launchState = LaunchState.START; // set our launch state to start

    public double neededLaunchVelocity; // this stores our needed launch velocity, used to check if we're in range

    private int ballsRemaining = 3;

    public Robot(HardwareMap hw) { // create all of our hardware
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

        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // we are using the encoder for stall detection
        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        //PIDFCoefficients pidfOrig = launch.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(Tunables.launchP, Tunables.launchI, Tunables.launchD, Tunables.launchF);
        launch.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        // set up our timer for the launch state machine
        launchStateTimer = new Timer(); // tracks time since we started our last launch state
        launchIntervalTimer = new Timer(); // tracks time since we launched our last ball
    }

    public static Robot getInstance(HardwareMap hw) { // this allows us to preserve the Robot instance from auto->teleop
        if (instance == null) { // we don't already have an instance
            instance = new Robot(hw); // create a new instance
        }
        return instance;
    }

    public double getDstFromGoal(Pose currentPosition, Pose goalPose) { // get our distance from the goal in inches
        return currentPosition.distanceFrom(goalPose); // use poses to find our distance easily :)
    }

    public double getGoalHeading(Pose currentPosition, Pose goalPose) { // return bot heading to point towards goal in radians
        double xDst = goalPose.getX() - currentPosition.getX();
        double yDst = goalPose.getY() - currentPosition.getY();
        return Math.atan2(yDst, xDst); // need atan2 to account for negatives
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
        setLaunchVelocity(neededVelocity);
    }

    public double TPSToRPM(double TPS) { return (TPS / TICKS_PER_REV) * 60 * Tunables.launchRatio; }
    public double RPMToTPS(double RPM) { return (RPM * TICKS_PER_REV / 60) / Tunables.launchRatio;}
    public double getLaunchRPM() { return TPSToRPM(launch.getVelocity()); } // return launch velocity in RPM
    public double getLaunchCurrent() { return launch.getCurrent(CurrentUnit.AMPS); } // return launch current in amps
    public double getIntakeCurrent() { return intake.getCurrent(CurrentUnit.AMPS); } // return intake current in amps
    public boolean isLaunchWithinMargin() {
        if (neededLaunchVelocity == 0) return true; // if our needed launch velocity is 0 (off) then we're within range
        return Math.abs(neededLaunchVelocity - launch.getVelocity()) < Tunables.scoreMargin; // measure if our launch velocity is within our margin of error
    }
    public boolean isIntakeStalled() {
        return intake.getCurrent(CurrentUnit.AMPS) >= Tunables.intakeOvercurrent
                || intake.getVelocity() < Tunables.intakeStallVelocity;
    }
    public void initServos() { // set servos to starting state
        upperTransfer.setPosition(Tunables.upperTransferClosed); // make sure balls cannot launch
        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit); // make sure lower transfer is not getting in the way
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

    public void launchBalls(int balls) { // sets to launch this many balls
        ballsRemaining = 3;
        launchIntervalTimer.resetTimer(); // reset interval timer (it may be off if cancelled)
        launchStateTimer.resetTimer(); // reset launch state timer (it may be off if cancelled)
        launchState = LaunchState.START;
    }

    public void cancelLaunch() { // set servos to default position, this could break if activated at the right time
        ballsRemaining = 0;
        upperTransfer.setPosition(Tunables.upperTransferClosed); // make sure balls can't accidentally be launched
        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit); // allow robot to store all balls
    }

    public boolean updateLaunch() { // outputs true/false whether we are done with launching
        if (ballsRemaining == 0) {
            return true; // we're done with launching balls
        } else if (ballsRemaining == 1 && launchIntervalTimer.getElapsedTime() <= Tunables.lastInterLaunchWait){
            return false; // keep waiting for our last interval
        } else if (ballsRemaining == 2 && launchIntervalTimer.getElapsedTime() <= Tunables.firstInterLaunchWait) {
            return false; // keep waiting on our first interval
        } else { // 3 balls remaining, no interval needed
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
                case PUSHING_LOWER_TRANSFER:
                    if (launchStateTimer.getElapsedTime() >= Tunables.pushDelay) {
                        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit);
                        upperTransfer.setPosition(Tunables.upperTransferClosed);
                        launchStateTimer.resetTimer();
                        launchState = LaunchState.WAITING_FOR_EXIT;
                    }
                case WAITING_FOR_EXIT:
                    if (launchStateTimer.getElapsedTime() >= Tunables.firstInterLaunchWait) { // ideally with this we won't need to separate first/last inter launch delay
                        launchState = LaunchState.START; // get ready for next one
                        ballsRemaining -= 1; // we've launched a ball
                        launchIntervalTimer.resetTimer();
                    }
            }
        }
        return false; // we're still working on launching balls
    }
}
