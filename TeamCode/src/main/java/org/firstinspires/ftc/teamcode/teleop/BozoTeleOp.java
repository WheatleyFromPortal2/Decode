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
    private Robot robot;
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive = false; // whether our drive is manually controlled or following a path
    private boolean automatedLaunch = false; // whether our launch speed is manually controlled or based off of distance from goal
    private TelemetryManager telemetryM;
    private final double turnRateMultiplier = 0.75; // always have our turns 75% speed
    private boolean isIntakePowered = false;
    private boolean isLaunchPowered = false;
    private boolean isRobotCentric = false; // allow driver to disable field-centric control if something goes wrong

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap); // get our robot instance (hopefully preserved from auto)

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose); // if we don't already have a starting pose, set it
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

        double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1; // amount to multiply for by slow mode

        if (gamepad1.aWasReleased()) {
            isIntakePowered = !isIntakePowered;
        }
        if (gamepad1.bWasReleased()) {
            isLaunchPowered = !isLaunchPowered;
        }
        if (!automatedDrive && gamepad1.xWasPressed()) { // if we're not in automated drive and we press X, turn to the goal
            turnToGoal();
        }
        if (automatedDrive && (gamepad1.xWasPressed() || !follower.isBusy())) { // if we're in automated drive and the user presses x or its done, then go to teleop
            follower.startTeleOpDrive();
            automatedDrive = false;
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
            headingPose.setHeading(Math.toRadians(90));
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
        }

        double launchTPS = ((gamepad1.right_trigger) * (2800)); // calculates max motor speed and multiplies it by the float of the right trigger

        if (isIntakePowered) robot.intake.setPower(1); // our intake is 0% or 100%
        else robot.intake.setPower(0);

        if (isLaunchPowered) {
            if (automatedLaunch) { // overwrite launchTPS with automatically calculated one
                double launchRPM = robot.setAutomatedLaunch(follower.getPose()); // set our launch to its needed speed and get our needed TPS
                launchTPS = robot.RPMToTPS(launchRPM); // tell the driver our correct desired RPM
                if (robot.isLaunchWithinMargin(launchTPS)) { // check if our current launch speed is within our margin
                    telemetryM.addLine("launch within margin!"); // tell the driver we're good to go
                } else telemetryM.addLine("launch out of margin!"); // tell the driver they need to wait
            }
            robot.launch.setVelocity(launchTPS);
        } else { // launch not powered
            robot.launch.setPower(0); // setting velocity to 0 causes oscillations
            launchTPS = 0; // indicate that launch isn't powered
        }

        telemetryM.debug("desired launch RPM", (launchTPS / Robot.TICKS_PER_REV) * 60 * Robot.launchRatio);
        telemetryM.debug("launch RPM", robot.getLaunchRPM()); // convert from ticks/sec to rev/min
        telemetryM.debug("launch current", robot.getLaunchCurrent()); // display launch current
        telemetryM.debug("intake current", robot.getIntakeCurrent()); // display intake current
        telemetryM.debug("lowerTransfer", robot.lowerTransfer.getPosition());
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.update(telemetry); // update telemetry
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
