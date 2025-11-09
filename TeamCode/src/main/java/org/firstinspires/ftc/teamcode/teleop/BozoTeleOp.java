package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Configurable
@TeleOp(name="BozoTeleOp", group="TeleOp")
public class BozoTeleOp extends OpMode {
    private Robot robot;
    private static Pose goal = new Pose(0, 0, 0); // filler goal position on field (for calculating launch), this will be overridden by the red and blue specific teleops
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive = false;
    private TelemetryManager telemetryM;
    private final double slowModeMultiplier = 0.5; // slow mode is 50% power
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
        if (gamepad1.startWasReleased()) { // if we press the start button, swap between robot and field centric
            isRobotCentric = !isRobotCentric;
        }

        if (!automatedDrive) {
            if (!gamepad1.left_bumper) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * turnRateMultiplier, // reduce speed by our turn rate
                    isRobotCentric // true = robot centric; false = field centric
            );
            else follower.setTeleOpDrive( // slow mode
                    -gamepad1.left_stick_y * slowModeMultiplier, // reduce speed by our slow mode multiplier
                    -gamepad1.left_stick_x * slowModeMultiplier, // reduce speed by our slow mode multiplier
                    -gamepad1.right_stick_x * slowModeMultiplier * turnRateMultiplier, // reduce speed by our slow mode multiplier and our turn rate
                    isRobotCentric // true = robot centric; false = field centric
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
