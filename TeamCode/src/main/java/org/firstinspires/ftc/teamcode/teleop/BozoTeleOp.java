/** this is our base blue teleop
 * it is extended by either BlueTeleOp/RedTeleOp
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

// Panels imports
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

// Pedro Pathing imports
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@Configurable
// this class will never be run as a TeleOp, and will always be extended by either RedTeleOp or BlueTeleOp
public abstract class BozoTeleOp extends OpMode {
    private Robot robot;
    private Follower follower;
    private Pose goalPose; // this will be set by the specific OpMode
    private Timer intakeTimer; // used for polling whether intake is stalled
    private Timer loopTimer; // measures the speed of our loop
    private boolean automatedDrive = false; // whether our drive is manually controlled or following a path
    private boolean automatedLaunch = false; // whether our launch speed is manually controlled or based off of distance from goal
    private TelemetryManager telemetryM;
    private boolean isIntakePowered = true; // start with intake powered
    private boolean isIntakeReversed = false; // 1 is for intake; -1 is for emergency eject/unclog
    private boolean isRobotCentric = false; // allow driver to disable field-centric control if something goes wrong
    double targetHeading;
    double launchVelocity; // target launch velocity in TPS

    @Override
    public void init() {
        // create timers and reset them
        loopTimer = new Timer();
        loopTimer.resetTimer();
        intakeTimer = new Timer();
        intakeTimer.resetTimer();

        robot = Robot.getInstance(hardwareMap); // get our robot instance (hopefully preserved from auto)
        follower = Constants.createFollower(hardwareMap);
        if (Robot.switchoverPose == null) follower.setStartingPose(new Pose());
        else { // hopefully this works
            follower.setPose(Robot.switchoverPose);
        }
        goalPose = getGoalPose();
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        launchVelocity = robot.RPMToTPS(Tunables.initialLaunchRPM); // convert from RPM->TPS, starting point
        telemetryM.debug("init time: " + loopTimer.getElapsedTime()); // tell how long our init tool
        telemetryM.update(telemetry);
    }

    protected abstract double flipControl(); // this will be filled in by Blue/Red TeleOp
    protected abstract Pose getGoalPose(); // this will be filled in by Blue/Red TeleOp

    public void start() {
        follower.startTeleopDrive(Tunables.useBrakes); // start the teleop, and use brakes
        robot.resetServos(); // set servos to starting state
    }

    @Override
    public void loop() {
        loopTimer.resetTimer();
        follower.update(); // update our Pedro Pathing follower
        //robot.updateBalls(); // update how many balls we have in our intake
        boolean updateLaunchStatus = robot.updateLaunch(); // idk if running it directly with the && might cause it to be skipped
        if (updateLaunchStatus && !follower.isTeleopDrive()) { // check if we're done with holding position
            follower.startTeleOpDrive();
        }

        if (gamepad1.aWasReleased()) { // toggle intake
            isIntakePowered = !isIntakePowered;
        }
        if (gamepad1.bWasReleased()) { // toggle automated launch
            automatedLaunch = !automatedLaunch;
        }
        if (gamepad1.yWasReleased()) {
            if (robot.isLaunching()) { // if we release y while we're launching, it will cancel
                robot.cancelLaunch();
                follower.startTeleOpDrive(Tunables.useBrakes); // stop holding pose
            } else { // if we're not already launching
                follower.holdPoint(follower.getPose()); // hold our pose while we're launching
                //automatedDrive = true; i don't this is necessary
                robot.launchBalls(3); // launch 3 balls
            }
        }
        if (gamepad1.rightBumperWasReleased()) {
            follower.holdPoint(follower.getPose()); // hold our pose while we're launching
            //automatedDrive = true; i don't this is necessary
            robot.launchBalls(1);
        }
        if (gamepad1.startWasReleased()) { // if we press the start button, swap between robot and field centric
            isRobotCentric = !isRobotCentric;
        }
        if (gamepad1.xWasReleased()) isIntakeReversed = !isIntakeReversed; // reverse intake to eject/unclog
        if (gamepad1.backWasReleased()) { // reset field-centric heading
            Pose headingPose = follower.getPose();
            headingPose = headingPose.setHeading(Math.toRadians(90)); // the driver must point toward the top of the field (goals) to recalibrate the heading
            follower.setPose(headingPose); // see if this works
        }

        if (gamepad1.dpadUpWasPressed()) launchVelocity += robot.RPMToTPS(Tunables.adjustRPM); // increment by adjustRPM (in TPS)
        if (gamepad1.dpadDownWasPressed()) launchVelocity -= robot.RPMToTPS(Tunables.adjustRPM); // decrement by adjustRPM (in TPS)

        if (!automatedDrive) {
            double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1; // amount to multiply for by slow mode
            // slow mode is built in using slowModeMultiplier controlled by left trigger
            if (isRobotCentric) { // we are controlling relative to the robot
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
                        true // true = robot centric; false = field centric
                );
            } else { // we are controlling relative to the field
                // we need to modify our x input to be in accordance with the driver's control position
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier * flipControl(),
                        -gamepad1.left_stick_x * slowModeMultiplier * flipControl(),
                        -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
                        false // true = robot centric; false = field centric
                );
            }
            //if (gamepad1.leftBumperWasReleased()) teleOpLaunchPrep(); // turn to goal if we're not in automated drive
        } else { // we're in automated drive
            if (gamepad1.leftBumperWasReleased() // if the user presses the left bumper again, cancel
                    || !follower.isTurning() // if the follower is done, cancel
                    || Math.abs(follower.getHeading() - targetHeading) <= Tunables.launchTurnMargin) { // sometimes the follower gets stuck, so we will just check if it's within our margin
                follower.startTeleOpDrive();
                automatedDrive = false;
            }
        }

        if (automatedLaunch) {
            robot.setAutomatedLaunchVelocity(follower.getPose(), getGoalPose()); // set our launch to its needed speed and get our needed TPS
        } else { // set our launch velocity manually based off the right trigger
            robot.setLaunchVelocity(launchVelocity); // set our launch velocity to our desired launch velocity
            /*
            double launchTPS = ((gamepad1.right_trigger) * (2800)); // calculates max motor speed and multiplies it by the float of the right trigger
            if (launchTPS == 0)
                robot.launchOff(); // if right trigger isn't pressed, don't even use PIDF
            else robot.setLaunchVelocity(launchTPS); // set our launch power manually */
        }

        // intake control
        if (isIntakePowered) { // if we want to power our intake, and it isn't full
            // our intake is 0% or 100%
            if (!isIntakeReversed && !robot.isFull()) robot.intake.setPower(1); // only power intake normal direction if we aren't full
            if (isIntakeReversed) robot.intake.setPower(-1); // reverse intake to eject/unclog (works even if intake is full)
        } else if (robot.isLaunching()) {
            robot.intake.setPower(1); // always power intake while we're launching
        } else {
            robot.intake.setPower(0); // turn off intake if other conditions aren't fulfilled
        }
        switch (robot.getBallsRemaining()) { // haptic feedback based on how many balls are in the robot
            case 0:
                gamepad1.stopRumble(); // don't rumble if we don't have any balls
                break;
            case 1:
                gamepad1.runRumbleEffect(Tunables.rumble1);
                break;
            case 2:
                gamepad1.runRumbleEffect(Tunables.rumble2);
                break;
            case 3:
                gamepad1.runRumbleEffect(Tunables.rumble3);
                break;
        }

        // all telemetry with a question mark (?) indicates a boolean
        if (isIntakeReversed) telemetryM.addLine("WARNING: INTAKE REVERSED!!!"); // alert driver if intake is reversed
        if (robot.isFull()) telemetryM.addLine("WARNING: INTAKE FULL!!!"); // alert driver intake is over current
        telemetryM.addData("d", robot.getDstFromGoal(follower.getPose(), getGoalPose()));
        telemetryM.addData("ballsRemaining", robot.getBallsRemaining()); // display balls remaining to driver
        // TODO: convey ballsRemaining with haptic feedback
        telemetryM.debug("target heading: " + robot.getGoalHeading(follower.getPose(), getGoalPose()));
        telemetryM.debug("current heading: " + follower.getHeading());
        telemetryM.debug("launch within margin?: " + robot.isLaunchWithinMargin()); // hopefully the bool should automatically be serialized
        telemetryM.debug("automated drive?: " + automatedDrive);
        telemetryM.debug("TeleOp drive?: " + follower.isTeleopDrive());
        telemetryM.debug("automated launch?: " + automatedLaunch);
        telemetryM.debug("follower busy?: " + follower.isBusy());
        telemetryM.debug("desired launch RPM: " + robot.TPSToRPM(robot.neededLaunchVelocity)); // make sure to convert from TPS->RPM
        // we're using addData for these because we want to be able to graph them
        telemetryM.addData("launch RPM", robot.getLaunchRPM()); // convert from ticks/sec to rev/min
        telemetryM.addData("launch current", robot.getLaunchCurrent()); // display launch current
        telemetryM.addData("intake current", robot.getIntakeCurrent()); // display intake current
        telemetryM.debug("x: " + follower.getPose().getX());
        telemetryM.debug("y: " + follower.getPose().getY());
        telemetryM.debug("goalPose x: " + goalPose.getX());
        telemetryM.debug("goalPose y: " + goalPose.getY());
        telemetryM.addData("loop time (millis)", loopTimer.getElapsedTime()); // we want to be able to graph this
        telemetryM.update(telemetry); // update telemetry (don't know why we need to pass in 'telemetry' object)
    }

    public void teleOpLaunchPrep() { // start spinning up and following the turn path
        // we shouldn't need to set our needed velocity because this should automatically be done by the teleop every loop
        // yet we will still check one more time
        double neededTangentialSpeed = robot.getTangentialSpeed(follower.getPose(), goalPose);
        double neededVelocity = robot.getNeededVelocity(neededTangentialSpeed); // honestly can combine these into the same function and return our needed TPS to check if we're spun up
        robot.launch.setVelocity(neededVelocity); // set our velocity to what we want

        targetHeading = robot.getGoalHeading(follower.getPose(), goalPose);
        /*Pose holdPose = follower.getPose().setHeading(targetHeading);
        PathChain turnPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), holdPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetHeading) // we want to turn from our current heading to our target heading
                .build();
        follower.followPath(turnPath, Tunables.holdEnd); // follow this path and hold end */
        follower.turnTo(targetHeading); // see if this works
        //follower.holdPoint(holdPose); // hopefully this doesn't interfere
        automatedDrive = true; // we're driving automatically now
        automatedLaunch = true; // make sure our launch is automated while we're turning to the goal
    }
}
