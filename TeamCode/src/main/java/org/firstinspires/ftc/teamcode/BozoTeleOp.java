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



// AprilTag imports

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



import java.util.List;



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



    // AprilTag variables

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private static final String WEBCAM_NAME = "Webcam 1"; // Name of the webcam in the hardware config

    /**

     * Camera position and orientation relative to robot center.

     * 

     * Camera axes (from camera's perspective):

     * - Origin: Center of the lens

     * - +x: right, +y: down, +z: forward

     * 

     * Robot axes (typical FTC convention):

     * - Origin: Center of the robot at field height

     * - +x: right, +y: forward, +z: upward

     * 

     * Position (translation from robot center to camera lens):

     * - If camera is at robot center: (0, 0, 0)

     * - Example: camera 5" left, 7" forward, 12" above ground: (-5, 7, 12)

     * 

     * Orientation (rotation of camera relative to robot):

     * - Pitch: rotation about X axis (typically -90° for horizontal camera)

     * - Roll: rotation about Y axis (typically 0° for upright camera)

     * - Yaw: rotation about Z axis (0° = forward, +90° = left, -90° = right)

     * 

     * IMPORTANT: Adjust these values based on your camera's physical mounting position!

     */

    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);

    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private boolean useAprilTagLocalization = true; // Toggle to enable/disable AprilTag localization

    private double lastAprilTagUpdateTime = 0;

    /**

     * Update interval for AprilTag pose updates (in seconds).

     * 

     * For continuous autonomous re-localization, updates should be as frequent as possible.

     * Reduced to 50ms (0.05s) to allow 20 updates per second for responsive field-oriented driving.

     * This ensures the robot's pose is constantly corrected, preventing drift accumulation.

     * 

     * Note: If CPU usage becomes an issue, you can increase this value, but 50ms provides

     * excellent responsiveness while still managing resources effectively.

     */

    private static final double APRILTAG_UPDATE_INTERVAL = 0.05; // 50ms = 20 updates/second

    

    /**

     * Maximum allowed pose jump distance (in inches) for validation.

     * 

     * This prevents erroneous pose updates from causing erratic robot behavior.

     * If an AprilTag detection suggests the robot moved more than this distance

     * since the last update, the update is rejected as likely erroneous.

     * 

     * Set based on maximum expected robot velocity. For example:

     * - Max speed: 60 in/s, update interval: 0.05s → max expected change: 3 inches

     * - Using 12 inches provides safety margin while allowing normal movement

     */

    private static final double MAX_POSE_JUMP_DISTANCE = 12.0; // inches

    

    /**

     * Maximum allowed heading change (in radians) for validation.

     * 

     * Prevents sudden heading jumps that could indicate detection errors.

     * Approximately 45 degrees (0.785 radians) provides reasonable tolerance.

     */

    private static final double MAX_HEADING_JUMP = Math.toRadians(45.0); // radians

    

    /**

     * Track the last successful AprilTag update time for telemetry.

     */

    private double lastSuccessfulUpdateTime = 0;



    @Override

    public void init() {

        robot = Robot.getInstance(hardwareMap); // get our robot instance (hopefully preserved from auto)



        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startingPose == null ? new Pose() : startingPose); // if we don't already have a starting pose, set it

        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();



        // Initialize AprilTag processor

        initAprilTag();

    }



    public void start() {

        follower.startTeleopDrive(true); // start the teleop, and use brakes

        robot.initServos(); // set servos to starting state

    }



    @Override

    public void loop() {

        // Update pose from AprilTag detections BEFORE follower.update()

        // This ensures the follower uses the most recent localization data

        // Pedro Pathing best practice: update external localization before calling follower.update()

        if (useAprilTagLocalization) {

            updatePoseFromAprilTag();

        }



        // Update the follower (must be called every loop iteration)

        // This processes the current pose and calculates drive vectors

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

        telemetryM.debug("AprilTags Detected", aprilTag.getDetections().size());

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



    /**

     * Initialize the AprilTag processor and VisionPortal.

     * 

     * This method follows the recommended initialization sequence:

     * 1. Create the AprilTag Processor with camera pose configuration

     * 2. Set decimation for optimal detection range vs. rate trade-off

     * 3. Create the VisionPortal with the processor

     * 

     * Camera Calibration:

     * - The SDK contains predefined calibration data for Logitech C270 at 640x480 resolution

     * - If using a different resolution or camera, you may need to provide custom lens intrinsics

     * - Without proper calibration, pose estimation accuracy will be reduced

     * - See ConceptAprilTagOptimizeExposure sample for exposure/gain optimization

     */

    private void initAprilTag() {

        // Create the AprilTag processor using Builder pattern

        aprilTag = new AprilTagProcessor.Builder()

                // Set camera pose relative to robot center (required for robotPose calculation)

                .setCameraPose(cameraPosition, cameraOrientation)

                // Camera calibration: SDK will attempt to load predefined calibration for Logitech C270

                // If calibration is not found, you'll see a warning but detection will still work

                // For custom calibration, uncomment and provide lens intrinsics:

                // .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) // fx, fy, cx, cy

                .build();



        // Adjust Image Decimation to trade-off detection-range for detection-rate.

        // 

        // Decimation values for Logitech C270 (typical):

        // - Decimation = 1: Detect 2" tag from 10 feet away at 10 FPS

        // - Decimation = 2: Detect 2" tag from 6 feet away at 22 FPS (good balance)

        // - Decimation = 3: Detect 2" tag from 4 feet away at 30 FPS (default, better for close tags)

        // 

        // Note: Decimation can be changed on-the-fly during runtime if needed

        aprilTag.setDecimation(2);



        // Create the vision portal using Builder pattern

        VisionPortal.Builder builder = new VisionPortal.Builder();



        // Set the camera (Logitech C270 webcam)

        // Ensure the webcam name matches your hardware configuration

        builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));



        // Camera resolution: Logitech C270 supports 640x480 (default)

        // The SDK has calibration data for Logitech C270 at 640x480

        // If you change resolution, you may need custom calibration:

        // builder.setCameraResolution(new Size(640, 480));



        // Add and enable the AprilTag processor

        // Note: addProcessor() automatically enables the processor

        builder.addProcessor(aprilTag);



        // Build the Vision Portal

        // This starts the camera stream and begins processing frames

        visionPortal = builder.build();

    }



    /**

     * Update robot pose from AprilTag detections.

     * 

     * This method provides CONTINUOUS, AUTONOMOUS re-localization for accurate field-oriented driving.

     * It runs every loop iteration (with rate limiting) to constantly correct pose and prevent drift.

     * 

     * Key Features for Continuous Localization:

     * 1. High-frequency updates (20 Hz) ensure pose is always current

     * 2. Smart validation prevents bad updates while allowing normal movement

     * 3. Automatic operation - no driver intervention required

     * 4. Handles intermittent detections gracefully

     * 

     * AprilTag Best Practices:

     * - Only uses tags with valid metadata (in the tag library)

     * - Skips Obelisk tags (not suitable for localization)

     * - Uses robotPose (field-relative position) rather than ftcPose (camera-relative)

     * - Selects the closest tag for best accuracy

     * - Rate limiting balances responsiveness with CPU usage

     * 

     * Pedro Pathing Best Practices:

     * - Updates pose before calling follower.update() to ensure follower uses latest data

     * - Uses follower.setPose() to update localization (standard Pedro Pathing method)

     * - Heading must be in radians (Pedro Pathing requirement)

     * - Coordinate system: Pedro Pathing uses right-hand coordinate system

     *   - X increases as robot moves right

     *   - Y increases as robot moves forward (up on field)

     *   - Heading: 0 radians = facing right, π/2 = facing up, π = facing left, 3π/2 = facing down

     *   - Counterclockwise rotation is positive

     * 

     * Continuous Localization Strategy:

     * - Updates occur automatically whenever valid tags are detected

     * - Validation ensures updates are reasonable (prevents jumps from detection errors)

     * - When no tags are visible, the robot continues using last known pose (odometry continues)

     * - Field-oriented driving remains accurate as long as tags are periodically visible

     * 

     * Note: AprilTag robotPose uses FTC field coordinate system. Pedro Pathing uses its own

     * coordinate system. The coordinate systems should align if both use standard FTC conventions

     * (X = right, Y = forward). However, heading conventions may differ - verify during testing.

     */

    private void updatePoseFromAprilTag() {

        // Rate limit updates to balance responsiveness with CPU usage

        // For continuous autonomous re-localization, we update frequently (20 Hz)

        // This ensures field-oriented driving stays accurate throughout robot operation

        double currentTime = System.currentTimeMillis() / 1000.0;

        if (currentTime - lastAprilTagUpdateTime < APRILTAG_UPDATE_INTERVAL) {

            return; // Skip this iteration if not enough time has passed

        }

        lastAprilTagUpdateTime = currentTime;



        // Get all current detections from the AprilTag processor

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();



        // Find the best detection (closest tag with valid metadata)

        // Using the closest tag typically provides the most accurate pose estimate

        AprilTagDetection bestDetection = null;

        double bestDistance = Double.MAX_VALUE;



        // Process each detection to find the best one

        for (AprilTagDetection detection : currentDetections) {

            // CRITICAL: Only use tags that have metadata (are in the tag library)

            // Tags without metadata don't have size information, so pose estimation is not possible

            if (detection.metadata == null) {

                continue;

            }



            // IMPORTANT: Skip Obelisk tags as they should not be used for localization

            // Obelisk tags are mounted on moving objects and don't provide reliable field position

            if (detection.metadata.name.contains("Obelisk")) {

                continue;

            }



            // Check if this detection has valid robotPose (field-relative position)

            // robotPose is only available when:

            // 1. Tag has metadata (size information)

            // 2. Camera has calibration data

            // 3. Camera pose is properly configured

            if (detection.robotPose != null) {

                // Use the tag with the smallest range (closest to camera)

                // Closer tags generally provide more accurate pose estimates

                double range = detection.ftcPose.range;

                if (range < bestDistance) {

                    bestDistance = range;

                    bestDetection = detection;

                }

            }

        }



        // Update pose if we found a valid detection

        // This provides continuous autonomous re-localization for accurate field-oriented driving

        if (bestDetection != null && bestDetection.robotPose != null) {

            // Get position from AprilTag robotPose (field coordinates in inches)

            // robotPose.getPosition() gives the robot's position relative to field origin

            // Pedro Pathing coordinate system: X = right, Y = forward (should align with FTC standard)

            double x = bestDetection.robotPose.getPosition().getX(DistanceUnit.INCH);

            double y = bestDetection.robotPose.getPosition().getY(DistanceUnit.INCH);

            

            // VALIDATION: Prevent sudden pose jumps that could indicate detection errors

            // This ensures continuous localization remains stable while allowing normal movement

            // Get current pose for validation

            Pose currentPose = follower.getPose();

            double distanceChange = Math.hypot(x - currentPose.getX(), y - currentPose.getY());

            

            // Validate position change is reasonable (prevents erroneous updates)

            // This allows normal robot movement while rejecting detection errors

            if (distanceChange > MAX_POSE_JUMP_DISTANCE) {

                // Pose jump too large - likely a detection error, skip this update

                // This prevents bad updates while still allowing continuous updates when valid

                telemetryM.debug("AprilTag Update Rejected", String.format("Jump: %.2f\"", distanceChange));

                return;

            }



            // Get heading from AprilTag (robot orientation relative to field)

            // Convert from degrees to radians for Pedro Pathing Pose

            // Pedro Pathing requires all headings to be in radians

            // Pedro Pathing coordinate system: 0 radians = facing right, π/2 = facing up

            // Counterclockwise rotation is positive

            double heading = Math.toRadians(

                bestDetection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)

            );

            

            // VALIDATION: Prevent sudden heading jumps that could indicate detection errors

            // Normalize heading difference to [-π, π] range for comparison

            double headingChange = Math.abs(AngleUnit.normalizeRadians(heading - currentPose.getHeading()));

            if (headingChange > MAX_HEADING_JUMP) {

                // Heading jump too large - likely a detection error, skip this update

                telemetryM.debug("AprilTag Update Rejected", String.format("Heading jump: %.2f°", Math.toDegrees(headingChange)));

                return;

            }



            // Create new pose from AprilTag data

            // Pose constructor: (x, y, heading) where heading is in radians

            Pose aprilTagPose = new Pose(x, y, heading);



            // CONTINUOUS AUTONOMOUS RE-LOCALIZATION: Update the follower's pose with AprilTag data

            // 

            // This is the core of continuous localization - constantly correcting pose prevents drift

            // Pedro Pathing best practice: Use follower.setPose() to update localization

            // This is the standard method for integrating external localization sources

            // 

            // Strategy for Continuous Localization:

            // - Direct pose replacement: AprilTag provides absolute field position, so we use it directly

            // - High update frequency: 20 Hz ensures pose is always current

            // - Smart validation: Prevents bad updates while allowing normal movement

            // - Automatic operation: No driver intervention needed - localization happens continuously

            // 

            // When tags are visible: Pose is constantly updated from AprilTag (this code)

            // When tags are not visible: Robot continues using last known pose (odometry continues)

            // Result: Field-oriented driving remains accurate as long as tags are periodically visible

            // 

            // Note: If you have dead wheels or drive encoder odometry, you could fuse the data:

            // - Weighted average: Combine AprilTag (high weight when close) with odometry

            // - Exponential moving average: Smooth AprilTag updates with previous pose

            // - Confidence-based: Use AprilTag when close, odometry when far

            // For now, direct replacement provides excellent accuracy for field-oriented driving

            // 

            // Coordinate System Note:

            // Pedro Pathing uses: X = right, Y = forward, heading in radians

            // AprilTag robotPose uses: X = right, Y = forward, heading in degrees (converted to radians)

            // If coordinate systems don't align during testing, you may need to transform coordinates

            follower.setPose(aprilTagPose);

            

            // Track successful update for telemetry

            lastSuccessfulUpdateTime = currentTime;



            // Add telemetry for debugging and monitoring continuous localization

            telemetryM.debug("AprilTag X", String.format("%.2f", x));

            telemetryM.debug("AprilTag Y", String.format("%.2f", y));

            telemetryM.debug("AprilTag Heading", String.format("%.2f", Math.toDegrees(heading)));

            telemetryM.debug("AprilTag Range", String.format("%.2f", bestDistance));

            telemetryM.debug("AprilTag Status", "ACTIVE - Localizing");

            

        } else {

            // No valid tags detected - localization not updating, but odometry continues

            // This is normal when tags are temporarily out of view

            double timeSinceLastUpdate = currentTime - lastSuccessfulUpdateTime;

            if (timeSinceLastUpdate > 1.0) { // Only warn if no updates for >1 second

                telemetryM.debug("AprilTag Status", String.format("No tags - Last update: %.1fs ago", timeSinceLastUpdate));

            } else {

                telemetryM.debug("AprilTag Status", "Searching for tags...");

            }

        }

    }



    @Override

    public void stop() {

        // Clean up vision portal when OpMode stops

        if (visionPortal != null) {

            visionPortal.close();

        }

    }

}

