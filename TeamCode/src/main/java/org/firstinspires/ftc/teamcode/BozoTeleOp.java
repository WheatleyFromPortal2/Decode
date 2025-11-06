package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.function.Supplier;

@Configurable
@TeleOp(name="BozoTeleOp", group="TeleOp")
public class BozoTeleOp extends OpMode {
    private Robot robot;
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private final double slowModeMultiplier = 0.5;
    private final double turnRateMultiplier = 0.75;
    private boolean isIntakePowered = false;
    private boolean isLaunchPowered = false;
    private static final double debounceTime = 1; // wait for half a second before reading new button inputs

    @Override
    public void init() {
        robot = new Robot(hardwareMap); // create our robot class

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose); // if we don't already have a starting pose, set it
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // no idea what this is
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    public void start() {
        follower.startTeleopDrive(true); // start the teleop, and use brakes

        robot.upperTransfer.setPosition(Robot.upperTransferClosed); // make sure balls cannot launch
        robot.lowerTransfer.setPosition(Robot.lowerTransferLowerLimit); // make sure lower transfer is not getting in the way
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update(); // update telemetry manager (Panels)
        telemetry.update();  // update driver station telemetry

        // Read raw joystick inputs
        double ry = gamepad1.right_stick_y; // launch power (temporary until algorithm)

        if (gamepad1.aWasReleased()) {
            isIntakePowered = !isIntakePowered;
        }
        if (gamepad1.bWasReleased()) {
            isLaunchPowered = !isLaunchPowered;
        }
        if (gamepad1.yWasReleased()) {
            try {
                robot.launchBall();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (!automatedDrive) {
            if (!gamepad1.left_bumper) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y, // fix skewed directions
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * turnRateMultiplier,
                    false // true = robot centric; false = field centric
            );
            else follower.setTeleOpDrive( // slow mode
                    -gamepad1.left_stick_y * slowModeMultiplier, // reduce speed by our slow mode multiplier
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier * turnRateMultiplier,
                    false // true = robot centric; false = field centric
            );
        }

        double launchRPM = ((-ry + 1) * (double) 6000 / 2); // calculates max motor speed and multiplies it by the float of the joystick y value

        if (isIntakePowered) robot.intake.setPower(1);
        else robot.intake.setPower(0);

        if (isLaunchPowered) robot.launch.setVelocity((launchRPM / (60)) * Robot.TICKS_PER_REV );
        else {
            robot.launch.setPower(0);
            launchRPM = 0; // indicate that launch isn't powered
        }

        telemetryM.debug("desired launch RPM", launchRPM * Robot.launchRatio); // account for the launch ratio
        //telemetry.addData("desired launch TPS", (launchRPM / 60) * Robot.TICKS_PER_REV);
        telemetryM.debug("launch RPM", robot.getLaunchRPM()); // convert from ticks/sec to rev/min
        telemetryM.debug("launch current", robot.getLaunchCurrent()); // display launch current
        telemetryM.debug("intake current", robot.getIntakeCurrent()); // display intake current
        telemetryM.debug("lowerTransfer", robot.lowerTransfer.getPosition());
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.update(telemetry); // update telemetry
    }
}
