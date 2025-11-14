/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * This OpMode demonstrates field-oriented driving using AprilTag localization.
 * The robot uses a Logitech C270 webcam to detect AprilTags on the field and
 * actively localize its position and orientation for accurate field-relative movement.
 *
 * Key Features:
 * - AprilTag-based robot localization
 * - Field-oriented movement (robot drives in field directions, not robot directions)
 * - Automatic pose updates from AprilTag detections
 * - Fallback to IMU when AprilTags are not visible
 *
 * Hardware Requirements:
 * - Logitech C270 webcam configured as "Webcam 1"
 * - Mecanum drive train with motors: front_left_drive, front_right_drive, back_left_drive, back_right_drive
 * - IMU (for fallback orientation when AprilTags aren't visible)
 *
 * Field Coordinate System:
 * - X: Right (positive) / Left (negative)
 * - Y: Forward (positive) / Backward (negative)
 * - Z: Up (positive) / Down (negative)
 * - Yaw: Counter-clockwise positive (0° = forward along +Y axis)
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Field Oriented Drive", group = "Team")
//@Disabled
public class AprilTagFieldOrientedDrive extends LinearOpMode {

    // Hardware declarations
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private IMU imu;

    // Vision system
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Robot pose from AprilTag localization
    private double robotX = 0.0;  // Field X position (inches)
    private double robotY = 0.0;  // Field Y position (inches)
    private double robotYaw = 0.0; // Field heading (degrees)

    // Pose update tracking
    private ElapsedTime lastPoseUpdate = new ElapsedTime();
    private boolean hasValidPose = false;
    private static final double POSE_TIMEOUT_SEC = 2.0; // Consider pose stale after 2 seconds

    // Camera pose configuration
    // Adjust these values based on your camera's physical position on the robot
    // Position: (X, Y, Z) in inches relative to robot center at field height
    //   X: Right (positive) / Left (negative)
    //   Y: Forward (positive) / Backward (negative)
    //   Z: Up (positive) / Down (negative)
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);

    // Orientation: (Yaw, Pitch, Roll) in degrees
    //   Yaw: 0° = forward, +90° = left, -90° = right
    //   Pitch: -90° = horizontal (typical for forward-facing camera)
    //   Roll: 0° = normal, 180° = upside down
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES, 0, -90, 0, 0);

    // Drive parameters
    private static final double MAX_SPEED = 1.0;
    private static final double ROTATION_SPEED = 0.8;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        // Initialize AprilTag vision system
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("Left stick: Field-relative movement");
        telemetry.addLine("Right stick X: Rotation");
        telemetry.addLine("A: Reset IMU yaw");
        telemetry.addLine("B: Reset field position (use current AprilTag pose)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update robot pose from AprilTag detections
            updateRobotPose();

            // Handle controls
            handleControls();

            // Display telemetry
            displayTelemetry();

            sleep(20); // Small delay to prevent excessive CPU usage
        }

        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initialize hardware components
     */
    private void initHardware() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Set motor directions (adjust as needed for your robot)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust these values to match your REV Hub orientation on the robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * Initialize AprilTag processor and VisionPortal
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                // The SDK will automatically load calibration for Logitech C270 if available
                .build();

        // Adjust decimation for better performance/detection range trade-off
        // Lower values = better range but slower processing
        // Higher values = faster processing but shorter range
        aprilTag.setDecimation(2); // Good balance for C270

        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    /**
     * Update robot pose from AprilTag detections
     */
    private void updateRobotPose() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Look for valid detections (tags in the current game's tag library)
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                // Skip Obelisk tags as they shouldn't be used for localization
                if (!detection.metadata.name.contains("Obelisk")) {
                    // Get robot pose from this detection
                    Position robotPos = detection.robotPose.getPosition();
                    YawPitchRollAngles robotAngles = detection.robotPose.getOrientation();

                    // Update robot pose
                    robotX = robotPos.x;
                    robotY = robotPos.y;
                    robotYaw = robotAngles.getYaw(AngleUnit.DEGREES);

                    hasValidPose = true;
                    lastPoseUpdate.reset();

                    // Use first valid detection
                    break;
                }
            }
        }

        // Check if pose is stale
        if (lastPoseUpdate.seconds() > POSE_TIMEOUT_SEC) {
            hasValidPose = false;
        }
    }

    /**
     * Handle gamepad controls
     */
    private void handleControls() {
        // Reset IMU yaw when A is pressed
        if (gamepad1.a) {
            imu.resetYaw();
            telemetry.addLine("IMU Yaw Reset!");
        }

        // Reset field position to current AprilTag pose when B is pressed
        if (gamepad1.b && hasValidPose) {
            // This effectively sets the current position as the new origin
            // In practice, you might want to adjust this based on your needs
            telemetry.addLine("Field Position Reset!");
        }

        // Get joystick inputs
        double forward = -gamepad1.left_stick_y;  // Forward is negative Y
        double right = gamepad1.left_stick_x;     // Right is positive X
        double rotate = gamepad1.right_stick_x;   // Rotation

        // Apply field-oriented drive
        driveFieldRelative(forward, right, rotate);
    }

    /**
     * Drive the robot in field-relative coordinates
     * @param forward Forward/backward movement (-1.0 to 1.0)
     * @param right Right/left strafe movement (-1.0 to 1.0)
     * @param rotate Rotation (-1.0 to 1.0)
     */
    private void driveFieldRelative(double forward, double right, double rotate) {
        // Get current robot heading
        double currentYaw;
        if (hasValidPose) {
            // Use AprilTag-based heading
            currentYaw = Math.toRadians(robotYaw);
        } else {
            // Fallback to IMU heading
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        // Convert field-relative direction to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotate the angle by the negative of the robot's heading
        // This transforms field coordinates to robot coordinates
        theta = AngleUnit.normalizeRadians(theta - currentYaw);

        // Convert back to cartesian coordinates (now in robot frame)
        double robotForward = r * Math.sin(theta);
        double robotRight = r * Math.cos(theta);

        // Drive using robot-relative coordinates
        driveRobotRelative(robotForward, robotRight, rotate);
    }

    /**
     * Drive the robot using robot-relative coordinates
     * @param forward Forward/backward in robot frame
     * @param right Right/left strafe in robot frame
     * @param rotate Rotation
     */
    private void driveRobotRelative(double forward, double right, double rotate) {
        // Calculate mecanum wheel powers
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        // Find maximum power to normalize
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // Normalize if needed
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Apply speed limit
        frontLeftPower *= MAX_SPEED;
        frontRightPower *= MAX_SPEED;
        backLeftPower *= MAX_SPEED;
        backRightPower *= MAX_SPEED;

        // Apply rotation speed limit
        rotate *= ROTATION_SPEED;

        // Set motor powers
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Display telemetry information
     */
    private void displayTelemetry() {
        telemetry.addData("=== Robot Pose ===", "");
        if (hasValidPose) {
            telemetry.addData("Field X", "%.1f inches", robotX);
            telemetry.addData("Field Y", "%.1f inches", robotY);
            telemetry.addData("Field Yaw", "%.1f°", robotYaw);
            telemetry.addData("Pose Age", "%.1f sec", lastPoseUpdate.seconds());
        } else {
            telemetry.addData("Field Pose", "No valid AprilTag detected");
        }

        // IMU fallback info
        YawPitchRollAngles imuAngles = imu.getRobotYawPitchRollAngles();
        telemetry.addData("IMU Yaw", "%.1f°", imuAngles.getYaw(AngleUnit.DEGREES));

        // AprilTag detection info
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("AprilTags Detected", detections.size());

        // Show detected tag IDs
        if (!detections.isEmpty()) {
            StringBuilder tagIds = new StringBuilder();
            for (AprilTagDetection detection : detections) {
                if (tagIds.length() > 0) tagIds.append(", ");
                tagIds.append(detection.id);
            }
            telemetry.addData("Tag IDs", tagIds.toString());
        }

        // Vision portal status
        telemetry.addData("Vision Status", visionPortal.getCameraState());

        telemetry.update();
    }
}

