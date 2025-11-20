package org.firstinspires.ftc.teamcode.teleop;

// OpMode imports
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot; // get our Robot.java object

// Panels imports
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

// Pedro Pathing imports
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Configurable
@TeleOp(name="BozoTeleOp", group="TeleOp")
public class BozoTeleOp extends OpMode {
    // TODO: convert this OpMode to use a finite state machine
    // until that is done, the launch command will have to be manually issued
    private enum State { // define our possible states for our FSM
        START, // starting state, waiting for OpMode to begin
        MANUAL_DRIVE, // we are in driver-controlled drive
        WAITING_TO_TURN, // we are waiting for Pedro Pathing to turn to the goal
        WAITING_TO_SPINUP, // we are waiting for Pedro Pathing to spinup
        LAUNCH, // we are waiting for the manual/automatic launch to finish
        END // end state: do nothing (might not be necessary)
    }
    // TODO: use d-pad up/down to control offset for RPM
    private Robot robot;
    private Follower follower;
    private boolean automatedDrive = false; // whether our drive is manually controlled or following a path
    private boolean automatedLaunch = false; // whether our launch speed is manually controlled or based off of distance from goal
    private TelemetryManager telemetryM;
    private boolean isIntakePowered = false;
    private boolean isRobotCentric = false; // allow driver to disable field-centric control if something goes wrong
    double targetHeading;
    double launchVelocity; // target launch velocity in TPS

    // variables to be tuned
    private final double turnRateMultiplier = 0.75; // always have our turns 75% speed
    private final int adjustRPM = 50; // driver increments/decrements by adjustRPM
    private double initialLaunchRPM = 2400; // maybe 2500; from crease

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap); // get our robot instance (hopefully preserved from auto)
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(Robot.switchoverPose == null ? new Pose() : Robot.switchoverPose); // if we don't already have a starting pose, set it
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        launchVelocity = robot.RPMToTPS(initialLaunchRPM); // convert from RPM->TPS, starting point
    }

    public void start() {
        follower.startTeleopDrive(true); // start the teleop, and use brakes
        robot.initServos(); // set servos to starting state
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update(); // update telemetry manager (Panels)

        if (gamepad1.aWasReleased()) {
            isIntakePowered = !isIntakePowered;
        }
        if (gamepad1.yWasReleased()) {
            automatedLaunch = !automatedLaunch; // invert automated launch
        }
        if (gamepad1.rightBumperWasReleased()) {
            try {
                robot.launchBall();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if (gamepad1.startWasReleased()) { // if we press the start button, swap between robot and field centric
            isRobotCentric = !isRobotCentric;
        }
        if (gamepad1.backWasReleased()) { // reset field-centric heading
            Pose headingPose = follower.getPose();
            headingPose = headingPose.setHeading(Math.toRadians(0)); // i think this is right
            follower.setPose(headingPose); // see if this works
        }

        if (gamepad1.dpadUpWasPressed()) launchVelocity += robot.RPMToTPS(adjustRPM); // increment by adjustRPM (in TPS)
        if (gamepad1.dpadDownWasPressed()) launchVelocity -= robot.RPMToTPS(adjustRPM); // decrement by adjustRPM (in TPS)

        if (!automatedDrive) {
            double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1; // amount to multiply for by slow mode
            // slow mode is built in using slowModeMultiplier controlled by left trigger
            follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * turnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
                isRobotCentric // true = robot centric; false = field centric
            );
            if (gamepad1.xWasReleased()) teleOpLaunchPrep(); // turn to goal if we're not in automated drive
        } else { // we're in automated drive
            if (gamepad1.xWasPressed() || !follower.isBusy()) { // if the user presses X, OR its done, then go to TeleOp
                follower.startTeleOpDrive();
                automatedDrive = false;
            }
        }

        if (isIntakePowered) robot.intake.setPower(1); // our intake is 0% or 100%
        else robot.intake.setPower(0);

        if (automatedLaunch) {
            //robot.setAutomatedLaunchVelocity(follower.getPose()); // set our launch to its needed speed and get our needed TPS
            robot.setLaunchVelocity(launchVelocity); // set our launch velocity to our desired launch velocity
        } else { // set our launch velocity manually based off the right trigger
            double launchTPS = ((gamepad1.right_trigger) * (2800)); // calculates max motor speed and multiplies it by the float of the right trigger
            if (launchTPS == 0)
                robot.launchOff(); // if right trigger isn't pressed, don't even use PIDF
            else robot.setLaunchVelocity(launchTPS); // set our launch power manually
        }
        // all telemetry with a question mark (?) indicates a boolean
        telemetryM.debug("target heading: " + targetHeading);
        telemetryM.debug("launch within margin?: " + robot.isLaunchWithinMargin()); // hopefully the bool should automatically be serialized
        telemetryM.debug("automated drive?: " + automatedDrive);
        telemetryM.debug("automated launch?: " + automatedLaunch);
        telemetryM.debug("follower busy?: " + follower.isBusy());
        telemetryM.debug("desired launch RPM: " + robot.TPSToRPM(robot.neededLaunchVelocity)); // make sure to convert from TPS->RPM
        telemetryM.debug("launch RPM: " + robot.getLaunchRPM()); // convert from ticks/sec to rev/min
        telemetryM.debug("launch current: " + robot.getLaunchCurrent()); // display launch current
        telemetryM.debug("intake current: " + robot.getIntakeCurrent()); // display intake current
        telemetryM.debug("lowerTransfer: " + robot.lowerTransfer.getPosition());
        telemetryM.debug("x: " + follower.getPose().getX());
        telemetryM.debug("y: " + follower.getPose().getY());
        //telemetryM.debug("heading: " + follower.getPose().getHeading()); bruh - robot.goalPose doesn't work
        //telemetryM.debug("goalPose x: " + Robot.goalPose.getX());
        //telemetryM.debug("goalPose y: " + Robot.goalPose.getY());
        telemetryM.update(telemetry); // update telemetry (don't know why we need to pass in 'telemetry' object)
    }

    public void teleOpLaunchPrep() { // start spinning up and following the turn path
        // we shouldn't need to set our needed velocity because this should automatically be done by the teleop every loop
        // yet we will still check one more time
        double neededTangentialSpeed = robot.getTangentialSpeed(follower.getPose());
        double neededVelocity = robot.getNeededVelocity(neededTangentialSpeed); // honestly can combine these into the same function and return our needed TPS to check if we're spun up
        robot.launch.setVelocity(neededVelocity); // set our velocity to what we want


        targetHeading = robot.getGoalHeading(follower.getPose());
        PathChain turnPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), follower.getPose())) // our x-y pos will stay the same so just give our current position twice
                .setLinearHeadingInterpolation(follower.getHeading(), targetHeading) // we want to turn from our current heading to our target heading
                .build();

        follower.followPath(turnPath); // follow this path
        automatedDrive = true; // we're driving automatically now
        automatedLaunch = true; // make sure our launch is automated while we're turning to the goal
    }
}
