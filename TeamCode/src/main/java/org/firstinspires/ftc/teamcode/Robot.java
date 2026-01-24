/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// hardware imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

// unit imports
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

// Pedro Pathing imports
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.Range;


public class Robot { // create our global class for our robot
    public static final int MOTOR_TICKS_PER_REV = 28; // REV Robotics 5203/4 series motors have 28ticks/revolution
    public static final int TURRET_TICKS_PER_REV = 1024; // tested 1-13-26
    public static final double TURRET_ENCODER_RATIO = 5.5; // ratio from turretEncoder->turret
    //public static final double TURRET_SERVO_RATIO = 5.5 / 3; // ratio from turret1/2->turret
    public static Pose switchoverPose; // this must be initialized by the auto and is used to persist our current position from auto->TeleOp
    public DcMotorEx intake, launchLeft, launchRight, turretEncoder; // drive motors are handled by Pedro Pathing
    public Servo lowerTransfer, upperTransfer; // servos
    private Servo hood; // we only want to modify hood through setHoodPosition(pos), to ensure we don't set it out of bounds
    public CRServo turret1, turret2; // continuous servos
    public Rev2mDistanceSensor intakeSensor, lowerTransferSensor, upperTransferSensor; // all of our distance sensors for detecting balls
    private PIDF turretSinglePIDF, turretDoublePIDF, launchPIDF;

    private final Timer launchStateTimer, // tracks time since we started our last launch state
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
    public double desiredLaunchVelocity; // this stores our desired launch velocity, used to check if we're in range
    public double desiredTurretPosition; // this stores our desired turret position, in radians +/- facing directly forwards where + is clockwise
    private boolean isLaunching = false; // since we are now using ballsRemaining to see how many balls we have, we need this to track when we actually want to launch
    private int ballsRemaining = 0; // tracks how many balls are in the robot
    private boolean wasBallInIntake = false; // this tracks whether we had a ball in intake last time we checked, use to calculate whether we have gathered all of our balls
    private double lastLaunchInterval; // stores the amount of time it took for our last launch
    /** end vars that change **/

    /** for testing **/
    public double bestTarget;
    public double bestDist;
    public double candidate;
    public double launchCorrection; // power to apply to launch motors

    public Robot(HardwareMap hw) { // create all of our hardware and initialize our class
        // DC motors (all are DcMotorEx for current monitoring)
        intake = hw.get(DcMotorEx.class, "intake"); // intake motor
        launchLeft = hw.get(DcMotorEx.class, "launchLeft"); // left launch motor, connected with a 1:1 ratio
        launchRight = hw.get(DcMotorEx.class, "launchRight"); // right launch motor, connected with a 1:1 ratio
        turretEncoder = hw.get(DcMotorEx.class, "turretEncoder"); // encoder for turret, no motor is connected to it

        // servos
        lowerTransfer = hw.get(Servo.class, "lowerTransfer");
        upperTransfer = hw.get(Servo.class, "upperTransfer");
        hood = hw.get(Servo.class, "hood");

        // continuous servos
        turret1 = hw.get(CRServo.class, "turret1");
        turret2 = hw.get(CRServo.class, "turret2");

        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder

        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);
        launchRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        launchPIDF = new PIDF(Tunables.launchP, Tunables.launchI, Tunables.launchD, Tunables.launchF); // create our PIDF controller for our launch motors

        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // reset our encoder (this only seems to work when run after the OpMode is started)
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // we won't be using the motor at all
        // don't use F for the turret; F bases off of our target, which has no relation to the amount of power needed
        turretSinglePIDF = new PIDF(Tunables.turretSingleP, Tunables.turretSingleI, Tunables.turretSingleD, 0); // create our PIDF controller for 1servos for our turret
        turretDoublePIDF = new PIDF(Tunables.turretDoubleP, Tunables.turretDoubleI, Tunables.turretDoubleD, 0); // create our PIDF controller for 2servos for our turret

        // distance sensors
        intakeSensor = hw.get(Rev2mDistanceSensor.class, "intakeSensor");
        lowerTransferSensor = hw.get(Rev2mDistanceSensor.class, "lowerTransferSensor");
        upperTransferSensor = hw.get(Rev2mDistanceSensor.class, "upperTransferSensor");

        // timers
        launchStateTimer = new Timer(); // set up timer for the launch state machine
        intakeTimer = new Timer(); // set up timer to measure balls in intake
        intakeOvercurrentTimer = new Timer();
        launchIntervalTimer = new Timer();
        transferTimer = new Timer();
    }

    public double getTurretPosition() { // return our current turret angle in radians +/- from facing forwards
        int ticks = turretEncoder.getCurrentPosition();
        double encoderRevs = (double) ticks / TURRET_TICKS_PER_REV;
        double turretRevs = encoderRevs / TURRET_ENCODER_RATIO;
        return turretRevs * 2 * Math.PI; // convert to radians
    }

    public void setDesiredTurretPosition(double radians) { // set desired turret angle
        //desiredTurretPosition = Math.atan2(Math.sin(radians), Math.cos(radians)); // wrap the angle to +/-pi
        desiredTurretPosition = normalizeRadians(radians);
    }

    public double getDesiredTurretPosition() {
        return desiredTurretPosition;
    }

    private void calcTurretPIDFError(double desired) {
    }

    public void rotateTurret(double degrees) { // rotate our turret using degree tx from limelight
        double radianOffset = Math.toRadians(degrees);
        double newTurretPosition = getTurretPosition() + radianOffset;
        setDesiredTurretPosition(newTurretPosition); // should auto convert to unit circle
    }

    public void calcPIDF() { // this calculates and applies our PIDFs for our launch motors and turret servos
        // update all PIDF coefficients in controllers
        launchPIDF.updateTerms(Tunables.launchP, Tunables.launchI, Tunables.launchD, Tunables.launchF);
        // don't use F for the turret; F bases off of our target, which has no relation to the amount of power needed
        turretSinglePIDF.updateTerms(Tunables.turretSingleP, Tunables.turretSingleI, Tunables.turretSingleD, 0);
        turretDoublePIDF.updateTerms(Tunables.turretDoubleP, Tunables.turretDoubleI, Tunables.turretDoubleD, 0);

        // calc launch PIDF
        // we don't need to negate values from launchLeft, because we have already set its direction to reversed
        double launchTPS = getLaunchVelocity();
        if (TPSToRPM(desiredLaunchVelocity - launchTPS) > Tunables.launchMaxPowerThreshold) {
            launchCorrection = 1; // if our RPM diff is really larger, ignore PIDF and just go full power
        } else {
            launchCorrection = launchPIDF.calc(desiredLaunchVelocity, launchTPS); // use PIDF to calculate needed correction
            launchCorrection = Range.clip(launchCorrection, -0.05, 1.0); // clamp motor output
        }
        launchLeft.setPower(launchCorrection); // apply correction
        launchRight.setPower(launchCorrection); // apply correction

        // calc turret PIDF
        bestTarget = 0; // if we can't find a good one, don't compute a change
        bestDist = Double.POSITIVE_INFINITY;

        double current = getTurretPosition();
        /*
        for (int k = -2; k <= 2; k++) {
            candidate = desiredTurretPosition + k * 2 * Math.PI;
            if (candidate < -Tunables.maxTurretRotation || candidate > Tunables.maxTurretRotation) continue;

            double dist = Math.abs(candidate - current);
            if (dist < bestDist) {
                bestDist = dist;
                bestTarget = candidate;
            }
        }

        double turretCorrection = turretPIDF.calc(bestTarget, getTurretPosition());
         */
        double turretCorrection;

        if (Math.abs(desiredTurretPosition - current) < Tunables.turretSingleMargin) {
            // single servo control
            turretCorrection = turretSinglePIDF.calc(desiredTurretPosition, current);
            turretCorrection = Range.clip(turretCorrection, -1.0, 1.0); // clip PIDF correction
            turret1.setPower(turretCorrection); // maybe switch between which servo is used for single correction in the future?
            turret2.setPower(0); // make sure they aren't fighting each other
        } else {
            // double servo control
            turretCorrection = turretDoublePIDF.calc(desiredTurretPosition, current);
            turretCorrection = Range.clip(turretCorrection, -1.0, 1.0); // clip PIDF correction
            turret1.setPower(turretCorrection); // turret1/2 should be operating in the same direction
            turret2.setPower(turretCorrection); // turret1/2 should be operating in the same direction
        }


    }

    public void zeroTurret() { // zero turret (set current position to forwards)
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getDstFromGoal(Pose currentPosition, Pose goalPose) { // get our distance from the goal in inches
        return currentPosition.distanceFrom(goalPose); // use poses to find our distance easily :)
    }

    public double getGoalHeading(@NonNull Pose currentPosition, @NonNull Pose goalPose) { // return bot heading to point towards goal in radians
        double xDst = goalPose.getX() - currentPosition.getX();
        double yDst = goalPose.getY() - currentPosition.getY();
        double desiredHeading = Math.atan2(yDst, xDst); // need atan2 to account for negatives
        return normalizeRadians(desiredHeading);
    }

    public void setAutomatedLaunchVelocity(double d) { // given positions, use our functions to set our launch speed
        if (!isLaunching()) { // don't update if we're launching
            double RPM = 705.41704 * Math.pow(d, 0.33109); // from Desmos data: 1-7-26
            // R^2 = 0.9829 using power regression (with log mode)
            RPM += Tunables.magicNumber;
            setLaunchVelocity(RPMToTPS(RPM)); // this also updates our neededLaunchVelocity
        }
    }

    public double TPSToRPM(double TPS) { return (TPS / MOTOR_TICKS_PER_REV) * 60 * Tunables.launchRatio; }
    public double RPMToTPS(double RPM) { return (RPM * MOTOR_TICKS_PER_REV / 60) / Tunables.launchRatio;}

    /** hardware methods **/
    public void resetLaunchServos() { // set servos to starting state
        upperTransfer.setPosition(Tunables.upperTransferClosed); // make sure balls cannot launch
        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit); // make sure lower transfer is not getting in the way
    }

    public double getIntakeCurrent() { return intake.getCurrent(CurrentUnit.AMPS); } // return intake current in amps
    public boolean isFull() {
        return ballsRemaining >= 3;
    }
    public double getLaunchRPM() { return TPSToRPM(getLaunchVelocity()); } // return launch velocity in RPM

    public double getLaunchCurrent() { // return total launch current in amps
        return launchLeft.getCurrent(CurrentUnit.AMPS) + launchRight.getCurrent(CurrentUnit.AMPS); // sum current from both servos
    }

    public boolean isLaunchWithinMargin() {
        if (desiredLaunchVelocity == 0) return true; // if our needed launch velocity is 0 (off) then we're within range
        return Math.abs(desiredLaunchVelocity - getLaunchVelocity()) < Tunables.scoreMargin; // measure if our launch velocity is within our margin of error
    }

    public double getLaunchVelocity() {
        // account for disconnected encoders
        if (launchLeft.getVelocity() <= 5) return launchRight.getVelocity();
        if (launchRight.getVelocity() <= 5) return launchLeft.getVelocity();
        else {
            return (launchLeft.getVelocity() + launchRight.getVelocity()) / 2; // average our TPS from both motors (the difference should be low)
        }
    }

    public void setLaunchVelocity(double velocity) { // velocity is in TPS
        desiredLaunchVelocity = velocity; // update our desired launch velocity
        // our actual changes to motor power are handled in calcPIDF()
    }

    public double getDesiredLaunchRPM() { return TPSToRPM(desiredLaunchVelocity); }
    public boolean isBallInIntake() { // return true if there is a ball reducing our measured distance
        if (intakeSensor.getDistance(DistanceUnit.MM) == 0) return true; // if our intake sensor isn't working
        else return intakeSensor.getDistance(DistanceUnit.MM) < Tunables.intakeSensorOpen;
    }
    public boolean isBallInLowerTransfer() { // return true if there is a ball reducing our measured distance
        if (lowerTransferSensor.getDistance(DistanceUnit.MM) == 0) return true; // if our lower transfer sensor isn't working
        else return lowerTransferSensor.getDistance(DistanceUnit.MM) < Tunables.lowerTransferSensorOpen; // a hole in the ball could be allowing a sensor to report a false negative, so we need to check both
    }
    public boolean isBallInUpperTransfer() { // return true if there is a ball reducing our measured distance
        // we want to return false when the sensor has disconnected, so our launch state machine can default to the static wait time
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
                    if (ballsRemaining > 1) intake.setPower(Tunables.launchingIntakePower); // hopefully allow lowerTransfer to go down
                    break;
                case OPENING_UPPER_TRANSFER:
                    /* because lowerTransferSensor keeps disconnecting, it helps to disable this
                    if (!isBallInLowerTransfer() // if we don't have a ball in lower transfer
                            && !isBallInIntake() // AND we don't have a ball waiting in intake
                            && launchStateTimer.getElapsedTime() >= Tunables.maxTransferDelay ) { // AND we have waited our max time for transfer to happen, we don't have any balls, let's not waste our time
                        ballsRemaining = 0;
                        launchState = LaunchState.START;
                        isLaunching = false;
                        return true;
                    }
                    */
                    if (ballsRemaining == 1) {
                        if (launchStateTimer.getElapsedTime() <= Tunables.lastOpenDelay) { break; }
                    } else {
                        if (launchStateTimer.getElapsedTime() <= Tunables.openDelay) { break; }
                    }
                    lowerTransfer.setPosition(Tunables.lowerTransferUpperLimit);
                    launchStateTimer.resetTimer();
                    launchState = LaunchState.WAITING_FOR_EXIT;
                    break;
                case WAITING_FOR_EXIT:
                    if (isBallInUpperTransfer() // wait until we detect a ball in upper transfer (ball has been launched)
                            || launchStateTimer.getElapsedTime() >= Tunables.maxPushDelay) { // or if that hasn't happened in a while, just go to the next launch
                        resetLaunchServos(); // reset our servos
                        launchState = LaunchState.START; // get ready for next one
                        ballsRemaining -= 1; // we've launched a ball
                        launchStateTimer.resetTimer(); // reset our timer
                        lastLaunchInterval = launchIntervalTimer.getElapsedTime();
                        transferTimer.resetTimer(); // we are starting transfer for next ball
                        intake.setPower(1);
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

    public void setHoodPosition(double pos) {
        if (pos < 0) return; // don't allow negative positions
        // set hood position relative to our minimum, so it is easy to recalibrate
        double newHoodPosition = Tunables.hoodMinimum + pos;
        // ensure pos is within acceptable hw range
        hood.setPosition(Math.min(newHoodPosition, Tunables.hoodMaximum)); // don't allow our hood to be set higher than our max position
    }

    public double getHoodPosition() { return hood.getPosition(); }

    public double setAutomatedHoodPosition(double d) {
        // TODO: fill this in with data from Desmos
        double neededHoodPosition = 0.5;
        hood.setPosition(neededHoodPosition);
        return neededHoodPosition;
    }
}
