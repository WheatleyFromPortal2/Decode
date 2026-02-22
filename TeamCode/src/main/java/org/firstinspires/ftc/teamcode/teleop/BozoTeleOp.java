/** this is our base blue teleop
 * it is extended by either BlueTeleOp/RedTeleOp
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.gamepad.GamepadManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Physics;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;
import org.firstinspires.ftc.teamcode.subsys.TimeProfiler;
import org.firstinspires.ftc.teamcode.Tunables;

// Panels imports
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

// Pedro Pathing imports
import org.firstinspires.ftc.teamcode.subsys.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.Range;

@Configurable
// this class will never be run as a TeleOp, and will always be extended by either RedTeleOp or BlueTeleOp
public abstract class BozoTeleOp extends OpMode {
    private Robot robot;
    private Physics physics;
    private Follower follower;
    private Vision vision;
    private TimeProfiler timeProfiler;
    @SuppressWarnings("FieldCanBeLocal")
    private Pose goalPose; // this will be set by the specific OpMode
    private Timer loopTimer; // measures the speed of our loop
    private boolean isAutomatedDrive = false; // whether our drive is manually controlled or following a path
    private boolean isAutomatedLaunch = true; // whether our launch speed is manually controlled or based off of distance from goal
    private TelemetryManager telemetryM;
    private GamepadManager gamepadManager;
    private boolean isIntakeReversed = false; // 1 is for intake; -1 is for emergency eject/unclog
    private boolean isRobotCentric = false; // allow driver to disable field-centric control if something goes wrong
    private LaunchSetpoints setpoints;
    private boolean isHoodLocked = true; // whether we want to change our hood with our right stick y
    double lastFollowerHeading;

    @Override
    public void init() {
        gamepadManager = new GamepadManager();
        gamepad1 = gamepadManager.asCombinedFTCGamepad(gamepad1);

        // create timers and reset them
        loopTimer = new Timer();
        loopTimer.resetTimer();

        robot = new Robot(hardwareMap);
        physics = new Physics();
        vision = new Vision(hardwareMap, isBlueTeam());
        vision.start();
        follower = Constants.createFollower(hardwareMap);

        follower.setPose(HandoffState.pose);
        lastFollowerHeading = HandoffState.pose.getHeading();
        robot.turret.zero(); // assume turret has been brought to zero position at the end of auto

        setpoints = new LaunchSetpoints(0, 0, 0);
        setpoints.setRPM(Tunables.initialManualLaunchRPM);
        robot.setSetpoints(setpoints);

        timeProfiler = new TimeProfiler();

        goalPose = getGoalPose();
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.debug("init time: " + loopTimer.getElapsedTime()); // tell how long our init tool
        telemetryM.update(telemetry);
    }

    protected abstract boolean isBlueTeam(); // this will be filled in by Blue/Red TeleOp
    protected abstract Pose getGoalPose(); // this will be filled in by Blue/Red TeleOp

    public void start() {
        follower.startTeleopDrive(Tunables.useBrakes); // start the teleop, and use brakes
        robot.transfer.reset();
        robot.intake.forward();
    }

    @Override
    public void loop() {
        timeProfiler.start("PIDF");
        timeProfiler.start("vision");
        vision.update();
        timeProfiler.stop();
        loopTimer.resetTimer();
        timeProfiler.start("follower");
        follower.update(); // update our Pedro Pathing follower
        //robot.updateBalls(); // update how many balls we have in our intake
        timeProfiler.start("update launch");
        timeProfiler.start("buttons");
        if (gamepad1.aWasReleased()) { // toggle intake
            robot.intake.toggle();
        }
        if (gamepad1.bWasReleased()) { // toggle automated launch
            isAutomatedLaunch = !isAutomatedLaunch;
        }
        if (gamepad1.yWasReleased()) {
            if (robot.isLaunching()) { // if we release y while we're launching, it will cancel
                robot.endLaunch();
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
                //robot.setDesiredTurretPosition(0); // lock turret
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

        if (gamepad1.dpadUpWasReleased()) setpoints.incrementRPM(Tunables.adjustRPM); // increment by adjustRPM (in TPS)
        if (gamepad1.dpadDownWasReleased()) setpoints.decrementRPM(Tunables.adjustRPM); // decrement by adjustRPM (in TPS)
        if (gamepad1.dpadRightWasReleased()) setpoints.incrementRPM(Tunables.adjustRPM / 2.0); // increment by half of adjustRPM (in TPS)
        if (gamepad1.dpadLeftWasReleased()) setpoints.decrementRPM(Tunables.adjustRPM / 2.0); // decrement by half of adjustRPM (in TPS)

        if (!isAutomatedDrive) {
            double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1; // amount to multiply for by slow mode
            // slow mode is built in using slowModeMultiplier controlled by left trigger
            if (isRobotCentric) { // we are controlling relative to the robot
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * Tunables.robotCentricTurnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
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
                        -gamepad1.right_stick_x * Tunables.fieldCentricTurnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
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

        timeProfiler.start("launch");

        if (isAutomatedLaunch) { // set our launch velocity and hood angle automatically
            if (Tunables.isDynamicPhysics) {
                setpoints = physics.getNeededVelocityDynamic(follower.getPose(), follower.getVelocity(), getGoalPose(), robot.getFirstShotDelay());
            } else {
                setpoints = physics.getNeededVelocityStatic(follower.getPose(), getGoalPose());
            }

            //double goalDst = robot.getGoalDst(follower.getPose(), getGoalPose()); // get goal distance using odo
            //double neededHoodPos = robot.getTurretGoalHeading(follower.getPose(), getGoalPose());
            //robot.setAutomatedLaunchSetpoints(goalDst);
            //robot.setAutomatedLaunchSetpoints(vision.getLastGoalDistance());
        } else { // set our launch velocity and hood angle manually
            if (!isHoodLocked) { // only if we don't have our hood position locked
                // set our hood position manually using right stick y by mapping it between our hood min/max
                setpoints.setHoodRadians(Range.scale(-gamepad1.right_stick_y, -1, 1, Tunables.hoodMinimumPos, Tunables.hoodMaximumPos));
            }
        }

        robot.setSetpoints(setpoints);

        // TODO: refactor this to use fused auto-aim
        lastFollowerHeading = follower.getHeading();

        timeProfiler.start("rumble");
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

        boolean updateLaunchStatus = robot.update(); // idk if running it directly with the && might cause it to be skipped
        if (updateLaunchStatus && !follower.isTeleopDrive()) { // check if we're done with holding position
            follower.startTeleOpDrive();
        }

        timeProfiler.start("telemetry");
        updateTelemetry();
    }

    private void updateTelemetry() {
        // all telemetry with a question mark (?) indicates a boolean
        // use .debug() for values not to be graphed and to save space
        // use .addData() for values to be graphed on Panels

        // warnings!
        if (robot.intake.isReverse()) {
            telemetryM.addLine("WARNING: INTAKE REVERSED!!!"); // alert driver if intake is reversed
        }

        // launch system
        if (isAutomatedLaunch) {
            telemetryM.addLine("launch is in AUTOMATED control");
            telemetryM.addData("desired RPM", robot.getSetpoints().getRPM());
            telemetryM.addData("desired hood pos", robot.getSetpoints().getHoodRadians());
        } else {
            telemetryM.addLine("launch is in MANUAL control");
            telemetryM.addData("desired flywheel RPM", setpoints.getRPM()); // make sure to convert from TPS->RPM
            telemetryM.addData("actual flywheel RPM", robot.flywheel.getRPM());
            if (isHoodLocked) { telemetryM.debug("hood LOCKED"); }
            telemetryM.addData("hood pos", robot.hood.getPos());
        }

        if (Tunables.isDebugging) {
            telemetryM.addData("hood pos", robot.getSetpoints().getHoodRadians());
            telemetryM.addData("setpoint turret pos", robot.getSetpoints().getTurretPos());
            telemetryM.addData("ballsRemaining", robot.getBallsRemaining()); // display balls remaining to driver

            // vision & distances
            telemetryM.addData("last vision goal dst", vision.getLastGoalDistance());
            telemetryM.addData("odo goal dst", follower.getPose().distanceFrom(getGoalPose()));
            telemetryM.debug("vision stale?: " + vision.isStale());
            telemetryM.debug("lastTx: " + vision.getLastGoalTx());

            // state
            telemetryM.debug("automated drive?: " + isAutomatedDrive);
            telemetryM.debug("TeleOp drive?: " + follower.isTeleopDrive());
            telemetryM.debug("follower busy?: " + follower.isBusy());

            // current monitoring
            telemetryM.addData("flywheel current", robot.flywheel.getCurrent()); // display combined launch current
            telemetryM.addData("intake current", robot.intake.getCurrent()); // display intake current
            telemetryM.addData("system current", robot.getSystemCurrent()); // display total system current

            // odo
            telemetryM.debug("current heading: " + follower.getHeading());
            telemetryM.debug("x: " + follower.getPose().getX());
            telemetryM.debug("y: " + follower.getPose().getY());
            //telemetryM.debug("goalPose x: " + goalPose.getX());
            //telemetryM.debug("goalPose y: " + goalPose.getY());

            // timing
            telemetryM.addData("time profiler", timeProfiler.getOutputAndReset());
        }

        telemetryM.addData("loop time (millis)", loopTimer.getElapsedTime()); // we want to be able to graph this
        telemetryM.update(telemetry); // needed to pass in telemetry object to also update on driver station
    }
}
