/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// hardware imports
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// unit imports
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsys.PIDF;
import org.firstinspires.ftc.teamcode.subsys.TimeProfiler;

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
    public static final double TURRET_SERVO_MAX_RANGE = 1.3962634015954636; // max range that the turret can go left or right from center

    public LynxModule controlHub;
    public LynxModule expansionHub;
    public DcMotorEx intake, launchLeft, launchRight, turretEncoder; // drive motors are handled by Pedro Pathing
    public Servo lowerTransfer, upperTransfer; // servos
    private Servo hood; // we only want to modify hood through setHoodPosition(pos), to ensure we don't set it out of bounds
    public Servo turret1, turret2; // continuous servos
    public Rev2mDistanceSensor intakeSensor; // all of our distance sensors for detecting balls
    public DigitalChannel upperTransferSensor, lowerTransferSensor;
    private PIDF turretSinglePIDF, turretDoublePIDF, launchPIDF;

    private final Timer launchStateTimer, // tracks time since we started our last launch state
            launchIntervalTimer; // this timer measures the time between individual launches

    private enum LaunchState { // these are the possible states our launch state machine can be in
        START,
        OPEN_UPPER_TRANSFER,
        RAISE_LOWER_TRANSFER,
        WAIT_FOR_SENSOR_HIT,
        WAIT_FOR_EXIT,
        WAIT_FOR_LOWER,
        WAIT_FOR_TRANSFER
    }

    /** only these variables should change during runtime **/
    public static LaunchState launchState = LaunchState.START; // set our launch state to start
    public double desiredLaunchVelocity; // this stores our desired launch velocity, used to check if we're in range
    public double desiredTurretPosition; // this stores our desired turret position, in radians +/- facing directly forwards where + is clockwise
    private boolean isLaunching = false; // since we are now using ballsRemaining to see how many balls we have, we need this to track when we actually want to launch
    private int ballsRemaining = 0; // tracks how many balls are in the robot
    private boolean wasBallInIntake = false; // this tracks whether we had a ball in intake last time we checked, use to calculate whether we have gathered all of our balls
    private double lastLaunchInterval; // stores the amount of time it took for our last launch
    private double lastTurretPos = 0;
    private boolean leftLaunchDisconnected = false; // whether our left launch encoder is disconnected
    private TimeProfiler timeProfiler;
    String profilerOutput;
    /** end vars that change **/

    /** for testing **/
    public double launchCorrection; // power to apply to launch motors

    public Robot(HardwareMap hw) { // create all of our hardware and initialize our class
        controlHub = hw.get(LynxModule.class, "Control Hub");
        expansionHub = hw.get(LynxModule.class, "Expansion Hub 2"); // I believe this starts at 2

        // DC motors (all are DcMotorEx for current monitoring)
        intake = hw.get(DcMotorEx.class, "intake"); // intake motor
        launchLeft = hw.get(DcMotorEx.class, "launchLeft"); // left launch motor, connected with a 1:1 ratio
        launchRight = hw.get(DcMotorEx.class, "launchRight"); // right launch motor, connected with a 1:1 ratio
        turretEncoder = hw.get(DcMotorEx.class, "turretEncoder"); // encoder for turret, no motor is connected to it

        // servos
        lowerTransfer = hw.get(Servo.class, "lowerTransfer");
        upperTransfer = hw.get(Servo.class, "upperTransfer");
        hood = hw.get(Servo.class, "hood");

        // turret servos
        turret1 = hw.get(Servo.class, "turret1");
        turret2 = hw.get(Servo.class, "turret2");

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
        lowerTransferSensor = hw.get(DigitalChannel.class, "lowerTransferSensor");
        upperTransferSensor = hw.get(DigitalChannel.class, "upperTransferSensor");
        lowerTransferSensor.setMode(DigitalChannel.Mode.INPUT);
        upperTransferSensor.setMode(DigitalChannel.Mode.INPUT);

        // timers
        launchStateTimer = new Timer(); // set up timer for the launch state machine
        launchIntervalTimer = new Timer();

        timeProfiler = new TimeProfiler();
    }

    public String getProfilerOutput() {
        return profilerOutput;
    }
    public double getSystemCurrent() { // return total system current (control + expansion hub) in amps
        return controlHub.getCurrent(CurrentUnit.AMPS) + expansionHub.getCurrent(CurrentUnit.AMPS);
    }

    public double getSystemVoltage() { // return system current in volts
        return (controlHub.getInputVoltage(VoltageUnit.VOLTS) + expansionHub.getInputVoltage(VoltageUnit.VOLTS)) / 2; // average values for more accuracy
    }

    private double turretTicksToRadians(double ticks) {
        double encoderRevs = -ticks / TURRET_TICKS_PER_REV; // negate ticks because positive is ccw for radians 🤦
        double turretRevs = encoderRevs / TURRET_ENCODER_RATIO;
        return turretRevs * 2 * Math.PI; // convert to radians
    }

    public double getTurretPosition() { // return our current turret angle in radians +/- from facing forwards
        return turretTicksToRadians(turretEncoder.getCurrentPosition());
        //return -(turret1.getPosition() - 0.5) * 2 * TURRET_SERVO_MAX_RANGE;
    }

    public void setDesiredTurretPosition(double radians) { // set desired turret angle
        //desiredTurretPosition = Math.atan2(Math.sin(radians), Math.cos(radians)); // wrap the angle to +/-pi
        desiredTurretPosition = normalizeRadians(radians);
    }

    public double getTurretVelocity() { // get turret speed in radians/s
        return turretTicksToRadians(turretEncoder.getVelocity());
    }

    public double getDesiredTurretPosition() {
        return desiredTurretPosition;
    }

    public void applyTxToTurret(double degrees, boolean isVisionStale) { // rotate our turret using degree tx from limelight
        if (!isVisionStale && Math.abs(getTurretVelocity()) <= Tunables.turretMaxVelocityForVision) {
            double radianOffset = Math.toRadians(degrees);
            radianOffset /= Tunables.turretTxReduction;
            //double newTurretPosition = getTurretPosition() - radianOffset;
            double newTurretPosition = desiredTurretPosition - radianOffset;
            setDesiredTurretPosition(newTurretPosition); // should auto convert to unit circle
        }
    }

    public void calcPIDF() { // this calculates and applies our PIDFs for our launch motors and turret servos
        timeProfiler.start("update terms");
        // update all PIDF coefficients in controllers
        launchPIDF.updateTerms(Tunables.launchP, Tunables.launchI, Tunables.launchD, Tunables.launchF);
        // don't use F for the turret; F bases off of our target, which has no relation to the amount of power needed
        //turretSinglePIDF.updateTerms(Tunables.turretSingleP, Tunables.turretSingleI, Tunables.turretSingleD, 0);
        //turretDoublePIDF.updateTerms(Tunables.turretDoubleP, Tunables.turretDoubleI, Tunables.turretDoubleD, 0);

        // calc launch PIDF
        // we don't need to negate values from launchLeft, because we have already set its direction to reversed
        timeProfiler.start("launch velocity query");
        double launchTPS = getLaunchVelocity();
        timeProfiler.start("launch calc");
        if (TPSToRPM(desiredLaunchVelocity - launchTPS) > Tunables.launchMaxPowerThreshold) {
            launchCorrection = 1; // if our RPM diff is really larger, ignore PIDF and just go full power
        } else {
            launchCorrection = launchPIDF.calc(desiredLaunchVelocity, launchTPS); // use PIDF to calculate needed correction
            launchCorrection = Range.clip(launchCorrection, -0.05, 1.0); // clamp motor output
        }
        timeProfiler.start("set launch power");
        launchLeft.setPower(launchCorrection); // apply correction
        launchRight.setPower(launchCorrection); // apply correction

        // calc turret PIDF
        timeProfiler.start("turret math");
        double turretServoPos = ((-desiredTurretPosition + Tunables.turretOffset) / (TURRET_SERVO_MAX_RANGE * 2)) + 0.5;

        turretServoPos = Range.clip(turretServoPos, 0.0, 1.0);
        timeProfiler.start("turret set pos");
        if (turretServoPos == lastTurretPos) {
            // do nothing, save loop time
        } else {
            turret1.setPosition(turretServoPos);
            turret2.setPosition(turretServoPos);
        }
        timeProfiler.stop();
        profilerOutput = timeProfiler.getOutputAndReset();
    }

    public void zeroTurret() { // zero turret (set current position to forwards)
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getGoalDst(Pose currentPosition, Pose goalPose) { // get our distance from the goal in inches
        return currentPosition.distanceFrom(goalPose) + Tunables.goalOffset; // use poses to find our distance easily :)
    }

    public double getTurretGoalHeading(@NonNull Pose currentPosition, @NonNull Pose goalPose) { // return turret heading to point towards goal in radians
        double xDst = goalPose.getX() - currentPosition.getX();
        double yDst = goalPose.getY() - currentPosition.getY();
        double desiredAbsoluteHeading = Math.atan2(yDst, xDst);
        return desiredAbsoluteHeading - currentPosition.getHeading() + Tunables.magicOffset;
        //return desiredAbsoluteHeading - currentPosition.getHeading();
    }

    public void setAutomatedLaunch(double d) { // given positions, use our functions to set our launch speed and hood position
        if (!isLaunching()) { // don't update if we're launching or if
            double RPM = 0;
            double hoodPos = 0;
            if (d < Tunables.farZoneDataStart) { // use close zone data
                RPM = -0.00943691 * Math.pow(d, 2) +12.47649 * d + 1772.38054;
                // from Desmos data: 2-5-26 (removing outliers)
                // R^2 = 0.973 using linear regression

                hoodPos = 0.00000256939 * Math.pow(d, 3) - 0.000531448 * Math.pow(d, 2) + 0.0367024 * d - 0.675026;
                // from Desmos data: 2-5-26 (removing outliers)
                // R^2 = 0.6066 using cubic regression
            } else { // use far zone data
                RPM = 0.0498139 * Math.pow(d, 2) - 1.32898 * d + 2370.65436 + Tunables.farAutoRPMOffset;
                // from Desmos data: 2-5-26 (removing outliers)
                // R^2 = 0.9822 using quadratic regression

                hoodPos = 0.22;
                // from Desmos data: 2-5-26 (removing outliers)
                // at this distance, we should always be using the lowest angle
            }

            RPM += Tunables.autoRPMOffset;
            setLaunchVelocity(RPMToTPS(RPM)); // this also updates our neededLaunchVelocity
            setHoodPosition(hoodPos);
        }
    }

    public void setAutomatedHoodPosition(double d) {
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
        if (!leftLaunchDisconnected) {
            double leftVelocity = launchLeft.getVelocity();
            if (leftVelocity <= 5) {
                leftLaunchDisconnected = true;
                return launchRight.getVelocity();
            } else {
                return leftVelocity;
            }
        } else {
            double rightVelocity = launchRight.getVelocity();
            if (rightVelocity <= 5) {
                leftLaunchDisconnected = false;
                return launchLeft.getVelocity();
            } else { return rightVelocity; }
        }
    }

    public void setLaunchVelocity(double velocity) { // velocity is in TPS
        if (!Double.isNaN(velocity)) { // make sure velocity is a number
            desiredLaunchVelocity = velocity; // update our desired launch velocity
            // our actual changes to motor power are handled in calcPIDF()
        }
    }

    public double getDesiredLaunchRPM() { return TPSToRPM(desiredLaunchVelocity); }
    public boolean isBallInIntake() { // return true if there is a ball reducing our measured distance
        if (intakeSensor.getDistance(DistanceUnit.MM) == 0) return true; // if our intake sensor isn't working
        else return intakeSensor.getDistance(DistanceUnit.MM) < Tunables.intakeSensorOpen;
    }
    public boolean isBallInLowerTransfer() { // return true if there is a ball reducing our measured distance
        return !lowerTransferSensor.getState(); // invert
        // if this sensor disconnects, it reports true (which is inverted to false), so the launch state machine will just fallback to the fixed delay
    }
    public boolean isBallInUpperTransfer() { // return true if there is a ball reducing our measured distance
        return !upperTransferSensor.getState(); // invert
        // if this sensor disconnects, it reports true (which is inverted to false), so the launch state machine will just fallback to the fixed delay
    }

    /** ball launching methods **/
    public void launchBalls(int balls) { // sets to launch this many balls
        ballsRemaining += balls;
        if (ballsRemaining > 3) ballsRemaining = 3;
        isLaunching = true; // we are launching now
        launchStateTimer.resetTimer(); // reset launch state timer (it may be off if cancelled)
        launchIntervalTimer.resetTimer();
        launchState = LaunchState.START; // reset our state machine to the start
    }

    public void cancelLaunch() { // set servos to default position, this could break if activated at the right time
        isLaunching = false; // stop launching
        ballsRemaining = 0;
        resetLaunchServos();
    }

    public boolean updateLaunch() { // outputs true/false whether we are done with launching
        if (ballsRemaining == 0) { // if we are done with balls or our launch isn't running fast enough
            cancelLaunch();
            return true; // we're done with launching balls
        } else if (isLaunching) { // balls remaining > 0 && we are launching
            switch (launchState) {
                case START:
                    launchStateTimer.resetTimer();
                    launchState = LaunchState.OPEN_UPPER_TRANSFER;
                    //if (ballsRemaining > 1) intake.setPower(Tunables.launchingIntakePower); // hopefully allow lowerTransfer to go down
                    if (upperTransfer.getPosition() != Tunables.upperTransferOpen) {
                        upperTransfer.setPosition(Tunables.upperTransferOpen);
                        launchState = LaunchState.OPEN_UPPER_TRANSFER;
                    } else {
                        launchState = LaunchState.RAISE_LOWER_TRANSFER;
                    }
                    break;
                case OPEN_UPPER_TRANSFER:
                    if (launchStateTimer.getElapsedTime() >= Tunables.openDelay) {
                        launchState = LaunchState.RAISE_LOWER_TRANSFER;
                    }
                    break;
                case RAISE_LOWER_TRANSFER:
                    lowerTransfer.setPosition(Tunables.lowerTransferUpperLimit);
                    intake.setPower(Tunables.launchingIntakePower);
                    launchStateTimer.resetTimer();
                    launchState = LaunchState.WAIT_FOR_SENSOR_HIT;
                    break;
                case WAIT_FOR_SENSOR_HIT:
                    if (isBallInUpperTransfer() // wait until we detect a ball in upper transfer (ball has been launched)
                            || launchStateTimer.getElapsedTime() >= Tunables.maxPushDelay) { // or if that hasn't happened in a while, just go to the next launch
                        launchStateTimer.resetTimer();
                        launchState = LaunchState.WAIT_FOR_EXIT;
                    }
                    break;
                case WAIT_FOR_EXIT:
                    if (launchStateTimer.getElapsedTime() >= Tunables.extraPushDelay) {
                        lowerTransfer.setPosition(Tunables.lowerTransferLowerLimit);
                        ballsRemaining -= 1; // we've launched a ball
                        if (ballsRemaining == 0) { // we're done with launching
                            launchState = LaunchState.START;
                            lastLaunchInterval = launchIntervalTimer.getElapsedTimeSeconds();
                        }
                        else {
                            launchState = LaunchState.WAIT_FOR_LOWER;
                        }
                        launchStateTimer.resetTimer(); // reset our timer
                    }
                    break;
                case WAIT_FOR_LOWER:
                    if (launchStateTimer.getElapsedTime() >= Tunables.lowerDelay) {
                        launchState = LaunchState.WAIT_FOR_TRANSFER;
                        intake.setPower(1);
                        launchStateTimer.resetTimer();
                    }
                    break;
                case WAIT_FOR_TRANSFER:
                    if (ballsRemaining == 1) {
                        if (launchStateTimer.getElapsedTime() < Tunables.lastTransferDelay && !isBallInLowerTransfer()) {
                            break;
                        }
                    } else {
                        if (launchStateTimer.getElapsedTime() < Tunables.transferDelay && !isBallInLowerTransfer()) {
                            break;
                        }
                    }
                    launchStateTimer.resetTimer();
                    launchState = LaunchState.START;
                    break;
            }
        }
        return false;
    }

    public void setBallsRemaining(int balls) { ballsRemaining = balls; }
    public int getBallsRemaining() { return ballsRemaining; }
    public double getLastLaunchInterval() { return lastLaunchInterval; }
    public boolean isLaunching() { return isLaunching; }

    public void setHoodPosition(double pos) { // set our hood position relative to minimum
        // we want to set relative to minimum in case we have to recalibrate our hood servo
        double desiredPos = Tunables.hoodMinimum + pos;
        hood.setPosition(Range.clip(desiredPos, Tunables.hoodMinimum, Tunables.hoodMaximum)); // force hood to stay inside of limits
    }

    public double getHoodPosition() {
        double hoodPos = hood.getPosition();
        if (hoodPos < 0.01) {
            return 0; // fix weird floating point display error
        } else {
            return hoodPos;
        }
    }
}
