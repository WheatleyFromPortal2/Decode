/** this is our base blue teleop
 * it is extended by either BlueTeleOp/RedTeleOp
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

// Panels imports
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

// Pedro Pathing imports
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@Configurable
// this class will never be run as a TeleOp, and will always be extended by either RedTeleOp or BlueTeleOp
public abstract class BozoTeleOp extends OpMode {
    private Robot robot;
    private Follower follower;
    private Vision vision;
    private Pose goalPose; // this will be set by the specific OpMode
    private Timer loopTimer; // measures the speed of our loop
    private boolean isAutomatedDrive = false; // whether our drive is manually controlled or following a path
    private boolean isAutomatedLaunch = true; // whether our launch speed is manually controlled or based off of distance from goal
    private TelemetryManager telemetryM;
    private boolean isIntakePowered = true; // start with intake powered
    private boolean isIntakeReversed = false; // 1 is for intake; -1 is for emergency eject/unclog
    private boolean isRobotCentric = false; // allow driver to disable field-centric control if something goes wrong
    private double manualLaunchVelocity; // target launch velocity in TPS
    private double manualLaunchVelocityOffset = 0;
    private boolean isHoodLocked = false; // whether we want to change our

    @Override
    public void init() {
        // create timers and reset them
        loopTimer = new Timer();
        loopTimer.resetTimer();

        robot = new Robot(hardwareMap);
        vision = new Vision(hardwareMap, isBlueTeam());
        vision.start();
        follower = Constants.createFollower(hardwareMap);

        follower.setPose(HandoffState.pose);
        robot.setBallsRemaining(HandoffState.ballsRemaining);
        robot.zeroTurret(); // assume turret has been brought to zero position at the end of auto

        goalPose = getGoalPose();
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        manualLaunchVelocity = robot.RPMToTPS(Tunables.initialManualLaunchRPM); // convert from RPM->TPS, starting point
        telemetryM.debug("init time: " + loopTimer.getElapsedTime()); // tell how long our init tool
        telemetryM.update(telemetry);
    }

    protected abstract boolean isBlueTeam(); // this will be filled in by Blue/Red TeleOp
    protected abstract Pose getGoalPose(); // this will be filled in by Blue/Red TeleOp

    public void start() {
        follower.startTeleopDrive(Tunables.useBrakes); // start the teleop, and use brakes
        robot.resetLaunchServos(); // set servos to starting state
    }

    @Override
    public void loop() {
        robot.calcPIDF();
        vision.update();
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
            isAutomatedLaunch = !isAutomatedLaunch;
        }
        if (gamepad1.yWasReleased()) {
            if (robot.isLaunching()) { // if we release y while we're launching, it will cancel
                robot.cancelLaunch();
                follower.startTeleOpDrive(Tunables.useBrakes); // stop holding pose
            } else { // if we're not already launching
                follower.holdPoint(follower.getPose()); // hold our pose while we're launching
                //isAutomatedDrive = true; i don't this is necessary
                robot.launchBalls(3); // launch 3 balls
            }
        }

        if (gamepad1.leftBumperWasReleased()) {
            if (!isAutomatedLaunch) {
                isHoodLocked = !isHoodLocked; // toggle whether our hood is locked
            } else { // in normal operation mode
                robot.setDesiredTurretPosition(0); // lock turret
            }
        }

        if (gamepad1.rightBumperWasReleased()) {
            follower.holdPoint(follower.getPose()); // hold our pose while we're launching
            //isAutomatedDrive = true; i don't this is necessary
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

        if (gamepad1.dpadUpWasReleased()) manualLaunchVelocityOffset += robot.RPMToTPS(Tunables.adjustRPM); // increment by adjustRPM (in TPS)
        if (gamepad1.dpadDownWasReleased()) manualLaunchVelocityOffset -= robot.RPMToTPS(Tunables.adjustRPM); // decrement by adjustRPM (in TPS)
        if (gamepad1.dpadLeftWasReleased()) manualLaunchVelocityOffset -= robot.RPMToTPS(Tunables.adjustRPM) / 2; // decrement by half of adjustRPM (in TPS)
        if (gamepad1.dpadRightWasReleased()) manualLaunchVelocityOffset += robot.RPMToTPS(Tunables.adjustRPM) / 2; // increment by half of adjustRPM (in TPS)

        if (!isAutomatedDrive) {
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
                double flipControl;
                if (isBlueTeam()) flipControl = -1; // blue team needs flipped
                else flipControl = 1; // red team doesn't need flip
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier * flipControl,
                        -gamepad1.left_stick_x * slowModeMultiplier * flipControl,
                        -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
                        false // true = robot centric; false = field centric
                );
            }
        } else { // we're in automated drive
            if (gamepad1.leftBumperWasReleased() // if the user presses the left bumper again, cancel
                    || !follower.isBusy()) { // if the follower is done, cancel
                follower.startTeleOpDrive();
                isAutomatedDrive = false;
            }
        }

        if (isAutomatedLaunch) { // set our launch velocity and hood angle automatically
            if (vision.isStale()) { // if it has been a while since our last vision reading
                //robot.setAutomatedLaunchVelocity(follower.getPose().distanceFrom(getGoalPose())); // get goal distance using odo
                //robot.setLaunchVelocity(robot.RPMToTPS(3000));
            } else {
                robot.setAutomatedLaunchVelocity(vision.getLastGoalDistance()); // get goal distance using vision
                robot.setAutomatedHoodPosition(vision.getLastGoalDistance()); // get goal distance using vision
            }
        } else { // set our launch velocity and hood angle manually
            robot.setLaunchVelocity(manualLaunchVelocity + manualLaunchVelocityOffset); // set our launch velocity to our desired launch velocity with our offset

            if (!isHoodLocked) { // only if we don't have our hood position locked
                // set our hood position manually using right stick y by mapping it between our hood min/max
                double hoodRange = Tunables.hoodMaximum - Tunables.hoodMinimum;
                double manualHoodPos = Tunables.hoodMinimum + hoodRange * gamepad1.left_stick_y; // multiply increase from min by left stick y value
                robot.setHoodPosition(manualHoodPos);
            }
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
        telemetryM.addData("hood position", robot.getHoodPosition());
        telemetryM.addData("lastGoalDistance", vision.getLastGoalDistance());
        telemetryM.addData("d", follower.getPose().distanceFrom(getGoalPose()));
        telemetryM.debug("vision stale?: " + vision.isStale());
        telemetryM.debug("lastTx: " + vision.getLastGoalTx());
        telemetryM.addData("ballsRemaining", robot.getBallsRemaining()); // display balls remaining to driver
        telemetryM.debug("target heading: " + robot.getGoalHeading(follower.getPose(), getGoalPose()));
        telemetryM.debug("current heading: " + follower.getHeading());
        telemetryM.debug("launch within margin?: " + robot.isLaunchWithinMargin()); // hopefully the bool should automatically be serialized
        telemetryM.debug("automated drive?: " + isAutomatedDrive);
        telemetryM.debug("TeleOp drive?: " + follower.isTeleopDrive());
        telemetryM.debug("automated launch?: " + isAutomatedLaunch);
        telemetryM.debug("follower busy?: " + follower.isBusy());
        telemetryM.debug("desired launch RPM: " + robot.getDesiredLaunchRPM()); // make sure to convert from TPS->RPM
        telemetryM.debug("desired launch RPM offset: " + robot.TPSToRPM(manualLaunchVelocityOffset)); // make sure to convert from TPS->RPM
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
}
