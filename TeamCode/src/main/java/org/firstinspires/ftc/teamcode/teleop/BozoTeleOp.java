/** this is our base blue teleop
 * it is extended by either BlueTeleOp/RedTeleOp
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TimeProfiler;
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
    private TimeProfiler timeProfiler;
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
    private boolean isHoodLocked = true; // whether we want to change our hood with our right stick y
    double lastFollowerHeading;

    @Override
    public void init() {
        // create timers and reset them
        loopTimer = new Timer();
        loopTimer.resetTimer();

        robot = new Robot(hardwareMap);
        vision = new Vision(hardwareMap);
        vision.startPipeline(Vision.Pipeline.FULL_POS);
        follower = Constants.createFollower(hardwareMap);

        follower.setPose(HandoffState.pose);
        lastFollowerHeading = HandoffState.pose.getHeading();
        robot.setBallsRemaining(HandoffState.ballsRemaining);
        robot.zeroTurret(); // assume turret has been brought to zero position at the end of auto
        robot.setLaunchVelocity(robot.RPMToTPS(Tunables.initialManualLaunchRPM)); // warm up launch

        timeProfiler = new TimeProfiler();

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
        timeProfiler.start("PIDF");
        robot.calcPIDF();
        timeProfiler.start("vision");
        vision.updateFullPos(follower.getHeading(), robot.getTurretPosition());
        timeProfiler.stop();
        loopTimer.resetTimer();
        timeProfiler.start("follower");
        follower.update(); // update our Pedro Pathing follower
        //robot.updateBalls(); // update how many balls we have in our intake
        timeProfiler.start("update launch");
        boolean updateLaunchStatus = robot.updateLaunch(); // idk if running it directly with the && might cause it to be skipped
        timeProfiler.start("buttons");
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

        timeProfiler.start("launch");

        if (isAutomatedLaunch) { // set our launch velocity and hood angle automatically
            if (vision.isStale()) { // if it has been a while since our last vision reading
                double neededHoodPos = robot.getTurretGoalHeading(follower.getPose(), getGoalPose());

                double goalDst = robot.getGoalDst(follower.getPose(), getGoalPose()); // get goal distance using odo
                robot.setAutomatedLaunch(goalDst);
                robot.setAutomatedHoodPosition(goalDst);
            } else {
                // TODO: fill in with appropriate automatic methods
            }
        } else { // set our launch velocity and hood angle manually
            robot.setLaunchVelocity(manualLaunchVelocity + manualLaunchVelocityOffset); // set our launch velocity to our desired launch velocity with our offset
            //robot.setDesiredTurretPosition(0); // lock turret

            if (!isHoodLocked) { // only if we don't have our hood position locked
                // set our hood position manually using right stick y by mapping it between our hood min/max
                double hoodRange = Tunables.hoodMaximum - Tunables.hoodMinimum;
                double stickValue = (-gamepad1.right_stick_y + 1) / 2; // reverse stick and map from (-1)<->(1) to (0)<->(1)
                double manualHoodPos = hoodRange * stickValue; // multiply increase from min by right stick y value
                robot.setHoodPosition(manualHoodPos);
            }
        }

        // turret control
        // TODO: fix to use absolute combined positioning from vision
        lastFollowerHeading = follower.getHeading();

        // intake control
        if (!robot.isLaunching()) { // allow Robot.java to handle intake while launching
            if (isIntakePowered) { // if we want to power our intake, and it isn't full
                // our intake is 0% or 100%
                if (!isIntakeReversed && !robot.isFull()) robot.intake.setPower(1); // only power intake normal direction if we aren't full
                if (isIntakeReversed) robot.intake.setPower(-1); // reverse intake to eject/unclog (works even if intake is full)
            }
            //else if (robot.isLaunching()) {
            //    robot.intake.setPower(1); // always power intake while we're launching
            //}
            else {
                robot.intake.setPower(0); // turn off intake if other conditions aren't fulfilled
            }
        }

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

        timeProfiler.start("telemetry");
        updateTelemetry();
    }

    private void updateTelemetry() {
        // all telemetry with a question mark (?) indicates a boolean
        // use .debug() for values not to be graphed and to save space
        // use .addData() for values to be graphed on Panels

        // warnings!
        if (isIntakeReversed) telemetryM.addLine("WARNING: INTAKE REVERSED!!!"); // alert driver if intake is reversed
        if (robot.isFull()) telemetryM.addLine("WARNING: INTAKE FULL!!!"); // alert driver intake is over current

        // launch system
        if (isAutomatedLaunch) {
            telemetryM.addLine("launch is in AUTOMATED control");
        } else {
            telemetryM.addLine("launch is in MANUAL control");
            telemetryM.debug("desired launch RPM offset: " + robot.TPSToRPM(manualLaunchVelocityOffset)); // make sure to convert from TPS->RPM
            telemetryM.debug("hood locked?: " + isHoodLocked);
        }

        if (Tunables.isDebugging) {
            telemetryM.debug("desired launch RPM: " + robot.getDesiredLaunchRPM()); // make sure to convert from TPS->RPM
            telemetryM.addData("launch RPM", robot.getLaunchRPM()); // convert from ticks/sec to rev/min
            telemetryM.addData("hood pos", robot.getHoodPosition());
            telemetryM.addData("turret pos", robot.getTurretPosition());
            telemetryM.addData("ballsRemaining", robot.getBallsRemaining()); // display balls remaining to driver
            telemetryM.debug("launch within margin?: " + robot.isLaunchWithinMargin());

            // vision & distances
            telemetryM.addData("odo goal dst", follower.getPose().distanceFrom(getGoalPose()));
            telemetryM.debug("vision stale?: " + vision.isStale());

            // state
            telemetryM.debug("automated drive?: " + isAutomatedDrive);
            telemetryM.debug("TeleOp drive?: " + follower.isTeleopDrive());
            telemetryM.debug("follower busy?: " + follower.isBusy());

            // current monitoring
            telemetryM.addData("launch current", robot.getLaunchCurrent()); // display combined launch current
            telemetryM.addData("intake current", robot.getIntakeCurrent()); // display intake current
            telemetryM.addData("system current", robot.getSystemCurrent()); // display total system current

            // odo
            telemetryM.debug("current heading: " + follower.getHeading());
            telemetryM.debug("odo goal target heading (deg): " + Math.toDegrees(robot.getTurretGoalHeading(follower.getPose(), getGoalPose())));
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
