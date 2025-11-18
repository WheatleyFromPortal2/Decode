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

    private Robot robot;
    private Follower follower;
    private boolean automatedDrive = false; // whether our drive is manually controlled or following a path
    private boolean automatedLaunch = false; // whether our launch speed is manually controlled or based off of distance from goal
    private TelemetryManager telemetryM;
    private final double turnRateMultiplier = 0.75; // always have our turns 75% speed
    private boolean isIntakePowered = false;
    private boolean isRobotCentric = false; // allow driver to disable field-centric control if something goes wrong

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap); // get our robot instance (hopefully preserved from auto)
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(Robot.switchoverPose == null ? new Pose() : Robot.switchoverPose); // if we don't already have a starting pose, set it
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void start() {
        follower.startTeleopDrive(true); // start the teleop, and use brakes
        robot.initServos(); // set servos to starting state
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update(); // update telemetry manager (Panels)

        if (gamepad1.dpad_up) {
            robot.teleOpLaunchPrep(follower);
        }

        double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1; // amount to multiply for by slow mode

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
            headingPose = headingPose.setHeading(Math.toRadians(90)); // i think this is right
            follower.setPose(headingPose); // see if this works
        }
        if (!automatedDrive) {
            // slow mode is built in using slowModeMultiplier controlled by left trigger
            follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * turnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
                isRobotCentric // true = robot centric; false = field centric
            );
            if (gamepad1.xWasReleased()) turnToGoal(); // turn to goal if we're not in automated drive
        } else { // we're in automated drive
            automatedLaunch = true; // make sure our launch is automated while we're turning to the goal
            if (gamepad1.xWasPressed() || !follower.isBusy()) { // if the user presses X, OR its done, then go to TeleOp
                follower.startTeleOpDrive();
                automatedDrive = false;
            }
        }

        if (isIntakePowered) robot.intake.setPower(1); // our intake is 0% or 100%
        else robot.intake.setPower(0);

        if (automatedLaunch) {
            robot.setAutomatedLaunchVelocity(follower.getPose()); // set our launch to its needed speed and get our needed TPS
        } else { // set our launch velocity manually based off the right trigger
            double launchTPS = ((gamepad1.right_trigger) * (2800)); // calculates max motor speed and multiplies it by the float of the right trigger
            if (launchTPS == 0) robot.launchOff(); // if right trigger isn't pressed, don't even use PIDF
            else robot.setLaunchVelocity(launchTPS); // set our launch power manually
        }

        // all telemetry with a question mark (?) indicate a boolean
        telemetryM.debug("launch within margin?", robot.isLaunchWithinMargin()); // hopefully the bool should automatically be serialized
        telemetryM.debug("automated drive?", automatedDrive);
        telemetryM.debug("automated launch?", automatedLaunch);

        telemetryM.debug("desired launch RPM", robot.TPSToRPM(robot.neededLaunchVelocity)); // make sure to convert from TPS->RPM
        telemetryM.debug("launch RPM", robot.getLaunchRPM()); // convert from ticks/sec to rev/min
        telemetryM.debug("launch current", robot.getLaunchCurrent()); // display launch current
        telemetryM.debug("intake current", robot.getIntakeCurrent()); // display intake current
        telemetryM.debug("lowerTransfer", robot.lowerTransfer.getPosition());
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.update(telemetry); // update telemetry (don't know why we need to pass in 'telemetry' object)
    }

    public void turnToGoal() {
        PathChain turnPath = follower.pathBuilder() // create our path (it just changes the heading)
                .addPath(new BezierLine(follower.getPose(), follower.getPose()))
                .setLinearHeadingInterpolation(follower.getHeading(), robot.getGoalHeading(follower.getPose()))
                .build();

        follower.followPath(turnPath); // start following our path
        automatedDrive = true; // now we're in automated drive mode
    }
}
