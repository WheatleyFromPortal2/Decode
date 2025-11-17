package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

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
    private boolean automatedDrive = false;
    private boolean automatedLaunch = false;
    private TelemetryManager telemetryM;
    private final double turnRateMultiplier = 0.75;
    private boolean isIntakePowered = false;
    private boolean isLaunchPowered = false;
    private boolean isRobotCentric = false;
    private boolean autoShootActive = false;
    private long autoShootStart;

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void start() {
        follower.startTeleopDrive(true);
        robot.initServos();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        if (autoShootActive) {
            if (System.currentTimeMillis() - autoShootStart < 5000) {
                double launchRPM = robot.setAutomatedLaunch(follower.getPose());
                robot.launch.setVelocity(robot.RPMToTPS(launchRPM));
                return;
            } else if (System.currentTimeMillis() - autoShootStart < 6500) {
                try { robot.launchBall(); } catch (InterruptedException e) { throw new RuntimeException(e); }
                return;
            } else if (System.currentTimeMillis() - autoShootStart < 8000) {
                try { robot.launchBall(); } catch (InterruptedException e) { throw new RuntimeException(e); }
                return;
            } else if (System.currentTimeMillis() - autoShootStart < 9500) {
                try { robot.launchBall(); } catch (InterruptedException e) { throw new RuntimeException(e); }
                return;
            } else {
                autoShootActive = false;
                follower.startTeleOpDrive();
            }
        }

        double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1;

        if (gamepad1.aWasReleased()) isIntakePowered = !isIntakePowered;
        if (gamepad1.bWasReleased()) isLaunchPowered = !isLaunchPowered;

        if (!automatedDrive && gamepad1.xWasPressed()) turnToGoal();
        if (automatedDrive && (gamepad1.xWasPressed() || !follower.isBusy())) {
            follower.startTeleOpDrive();
            automatedDrive = false;
        }

        if (gamepad1.yWasReleased()) automatedLaunch = !automatedLaunch;

        if (gamepad1.rightBumperWasReleased()) {
            try { robot.launchBall(); } catch (InterruptedException e) { throw new RuntimeException(e); }
        }

        if (gamepad1.startWasReleased()) isRobotCentric = !isRobotCentric;
        if (gamepad1.backWasReleased()) {
            Pose headingPose = follower.getPose();
            headingPose.setHeading(Math.toRadians(90));
            follower.setPose(headingPose);
        }

        if (gamepad1.dpad_up && !autoShootActive) {
            turnToGoal();
            autoShootActive = true;
            autoShootStart = System.currentTimeMillis();
        }

        if (!automatedDrive) {
            follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * turnRateMultiplier * slowModeMultiplier,
                isRobotCentric
            );
        }

        double launchTPS = (gamepad1.right_trigger) * (2800);

        if (isIntakePowered) robot.intake.setPower(1);
        else robot.intake.setPower(0);

        if (isLaunchPowered) {
            if (automatedLaunch) {
                double launchRPM = robot.setAutomatedLaunch(follower.getPose());
                launchTPS = robot.RPMToTPS(launchRPM);
            }
            robot.launch.setVelocity(launchTPS);
        } else {
            robot.launch.setPower(0);
            launchTPS = 0;
        }

        telemetryM.debug("desired launch RPM", (launchTPS / Robot.TICKS_PER_REV) * 60 * Robot.launchRatio);
        telemetryM.debug("launch RPM", robot.getLaunchRPM());
        telemetryM.debug("launch current", robot.getLaunchCurrent());
        telemetryM.debug("intake current", robot.getIntakeCurrent());
        telemetryM.debug("lowerTransfer", robot.lowerTransfer.getPosition());
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.update(telemetry);
    }

    public void turnToGoal() {
        PathChain turnPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), follower.getPose()))
                .setLinearHeadingInterpolation(follower.getHeading(), robot.getGoalHeading(follower.getPose()))
                .build();
        follower.followPath(turnPath);
        automatedDrive = true;
    }
}
