/** this is our base class for all of our autos
 * it is extended by BlueAuto/RedAuto
 * and then further extended by ...TriAuto/...GoalAuto
 */

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.Tunables;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot;


//@Autonomous(name = "BozoAuto", group = "Auto") // this is the base file which will just be extended so we don't want this OpMode to be directly run
public abstract class BozoAuto extends OpMode {
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();
    protected abstract Pose getStartPose();
    Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer, loopTimer;
    private TelemetryManager telemetryM; // create our telemetry object
    private Pose startPose;

    private enum State { // define our possible states for our FSM
        START, // starting state, waiting for OpMode to begin
        TRAVEL_TO_LAUNCH, // travel to our defined position to launch balls from. an internal switch statement control which path it will take
        LAUNCH, // wait for us to stop moving
        TRAVEL_TO_BALLS, // travel to the starting point of gathering balls
        RELOAD, // drive in the straight line to grab the balls
        GO_TO_END, // travel to our end position
        END // end state: do nothing
    }

    /** these are the **only variables** that should change throughout the auto **/
    State state = State.START; // set PathState to start
    private int ballTripletsRemaining = 4; // start with 4 ball triplets (1 in robot, 3 on field), decrements every launch

    // example paths
    private PathChain // some of these can probably just be Paths, but whatever
            scorePreload,
            startPickup1,
            grabPickup1,
            scorePickup1,
            startPickup2,
            grabPickup2,
            scorePickup2,
            startPickup3,
            grabPickup3,
            scorePickup3,
            goToEnd;

    public void buildPaths() {
        config = buildConfig(); // get our config

        // this path goes from the starting point to our scoring point
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, config.scoreIntermediatePose, config.scorePose)) // test if this works
                .setLinearHeadingInterpolation(startPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime) // hopefully this works
                .build();

        // this path goes from the score point to the beginning of the first set of balls
        startPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup1StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup1StartPose.getHeading(), Tunables.grabEndTime)
                .build();

        // this path picks up the first set of balls
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup1StartPose, config.pickup1EndPose))
                .setLinearHeadingInterpolation(config.pickup1StartPose.getHeading(), config.pickup1EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup to our score position
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup1EndPose, config.scorePose))
                .setLinearHeadingInterpolation(config.pickup1EndPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime)
                .build();

        // this path goes from the score point to the beginning of the second set of balls
        startPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup2StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup2StartPose.getHeading(), Tunables.grabEndTime)
                .build();

        // this path picks up the second set of balls
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup2StartPose, config.pickup2EndPose))
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.pickup2EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup to our score position
        scorePickup2 = follower.pathBuilder() // extra 2 lines to prevent hitting anything
                .addPath(new BezierCurve(config.pickup2EndPose, config.pickup2StartPose, config.scorePose)) // test if this works
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime) // this heading should work
                .build();

        // this path goes from the score point to the beginning of the third set of balls
        startPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup3StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup3StartPose.getHeading(), Tunables.grabEndTime)
                .build();

        // this path picks up the third set of balls
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup3StartPose, config.pickup3EndPose))
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.pickup3EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup our score position
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup3EndPose, config.scorePose))
                .setLinearHeadingInterpolation(config.pickup3EndPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime)
                .build();

        // this path goes form our score point to our ending position
        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.endPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.endPose.getHeading())
                .build();
    }

    // isn't as flexible as https://state-factory.gitbook.io/state-factory, but it should be good enough for now
    public void autonomousPathUpdate() throws InterruptedException {
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
        switch (state) {
            case START:
                follower.followPath(scorePreload);
                robot.launchBalls(3);
                setPathState(State.LAUNCH);
                break;
            case TRAVEL_TO_LAUNCH:
                if(!follower.isBusy()) {
                    if (ballTripletsRemaining == 0) { // if we're out of balls, just go to the end
                        state = State.GO_TO_END;
                        break;
                    }
                    switch (ballTripletsRemaining) { // this should always be between 3 and 0
                        case 3:
                            follower.followPath(scorePickup1, true); // hold end to prevent other robots from moving us
                            break;
                        case 2:
                            follower.followPath(scorePickup2, true);
                            break;
                        case 1:
                            follower.followPath(scorePickup3, true);
                            break;
                    }
                    state = State.LAUNCH; // let's launch
                    robot.launchBalls(3); // set up to launch 3 balls, it should not start launching until we call robot.updateLaunch()
                }
                break;
            case LAUNCH:
                if (!follower.isBusy() // check if our follower is busy
                        && robot.isLaunchWithinMargin() // check if our launch velocity is within our margin
                        && opmodeTimer.getElapsedTime() >= Tunables.beginningLaunchDelay // check if we've waited enough time since the last launch
                        && follower.getPose().roughlyEquals(config.scorePose, Tunables.launchDistanceMargin)) { // check if we're holding position close enough to where we want to shoot
                    //follower.holdPoint(config.scorePose); // this should already be done by holdEnd: true
                    /* if we're holding point, we shouldn't have to disable motors
                    follower.pausePathFollowing();
                    follower.deactivateAllPIDFs(); */
                    robot.intake.setPower(1); // turn on intake
                    if (robot.updateLaunch()) { // we're done with launching balls
                        ballTripletsRemaining -= 1;
                        robot.intake.setPower(0); // save power
                        setPathState(State.TRAVEL_TO_BALLS);
                        /* if we're holding point, we shouldn't have to re-enable motors
                        follower.activateAllPIDFs();
                        follower.resumePathFollowing(); */
                    } // if we're not done with launching balls, just break
                }
                break;
            case TRAVEL_TO_BALLS: // travel to the start position of the balls, but don't grab them yet
                if(!follower.isBusy()) {
                    if (ballTripletsRemaining == 0) { // if we're out of balls, just go to the end
                        state = State.GO_TO_END;
                        break;
                    }
                    switch (ballTripletsRemaining) { // this should always be between 3 and 0
                        case 3:
                            follower.followPath(startPickup1);
                            break;
                        case 2:
                            follower.followPath(startPickup2);
                            break;
                        case 1:
                            follower.followPath(startPickup3);
                            break;
                    }
                    state = State.RELOAD; // now lets reload
                }
                break;
            case RELOAD: // grab the balls in a straight line
                if(!follower.isBusy()) {
                    robot.intake.setPower(1); // re-enable intake to pickup balls
                    switch (ballTripletsRemaining) { // this should always be between 3 and 0
                        case 3:
                            follower.followPath(grabPickup1);
                            break;
                        case 2:
                            follower.followPath(grabPickup2);
                            break;
                        case 1:
                            follower.followPath(grabPickup3);
                            break;
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(State.TRAVEL_TO_LAUNCH); // now that we're reloaded, let's go to launch
                }
                break;
            case GO_TO_END: // travels to the end
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToEnd);
                    setPathState(State.END);
                }
                break;
            case END:
                if (!follower.isBusy()) {
                    robot.intake.setPower(0); // turn off intake
                    Robot.switchoverPose = follower.getPose(); // try to prevent drift
                    follower.deactivateAllPIDFs(); // stop the follower
                    requestOpModeStop(); // request to stop our OpMode so it auto transfers to TeleOp
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(State pState) {
        state = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        loopTimer.resetTimer();
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

        Robot.switchoverPose = follower.getPose(); // get our switchover pose ready for TeleOp (this may drift if we stop the OpMode mid-motion)

        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub for debugging
        telemetryM.debug("path state: " + state);
        telemetryM.addData("is follower busy", follower.isBusy());
        telemetryM.addData("ball triplets remaining", ballTripletsRemaining);
        telemetryM.addData("desired launch RPM", Tunables.scoreRPM);
        telemetryM.addData("launch RPM", robot.getLaunchRPM());
        telemetryM.debug("x", follower.getPose().getX());
        telemetryM.debug("y", follower.getPose().getY());
        telemetryM.debug("heading", follower.getPose().getHeading());
        telemetryM.addData("loop time (millis)", loopTimer.getElapsedTime()); // we want to be able to graph this
        telemetryM.update(telemetry); // update telemetry
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        // set up our timers
        loopTimer = new Timer();
        loopTimer.resetTimer();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = Robot.getInstance(hardwareMap); // create our robot class

        follower = Constants.createFollower(hardwareMap);
        startPose = getStartPose(); // the getStartPose method will be included in different classes for start points
        buildPaths(); // this will create our paths from our predefined variables
        follower.setStartingPose(startPose); // this will set our starting pose from our getStartPose() function
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // this gets our telemetryM object so we can write telemetry to Panels
        telemetryM.debug("init time (millis): " + loopTimer.getElapsedTime()); // i don't think addData works in init()
        telemetryM.update(telemetry);
    }

    /* not needed
    // This method is called continuously after Init while waiting for "play".
    @Override
    public void init_loop() {}
    */

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        robot.resetServos(); // get servos ready
        robot.intake.setPower(1); // start intake
        robot.setLaunchVelocity(robot.RPMToTPS(Tunables.scoreRPM)); // we're just gonna keep our score RPM constant for now
        setPathState(State.START);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        robot.resetServos(); // return servos to starting position
    }
}