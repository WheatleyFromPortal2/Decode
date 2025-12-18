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
    private Timer stateTimer, opModeTimer, loopTimer;
    private TelemetryManager telemetryM; // create our telemetry object
    private Pose startPose;

    private enum State { // define our possible states for our FSM
        START, // starting state, waiting for OpMode to begin
        TRAVEL_TO_LAUNCH, // travel to our defined position to launch balls from. an internal switch statement control which path it will take
        LAUNCH, // wait for us to stop moving
        TRAVEL_TO_BALLS, // travel to the starting point of gathering balls
        RELOAD, // drive in the straight line to grab the balls
        GO_TO_CLEAR,
        CLEAR,
        GO_TO_END, // travel to our end position
        END // end state: do nothing
    }

    /** these are the **only variables** that should change throughout the auto **/
    State state = State.START; // set PathState to start
    private int ballTripletsRemaining = 5; // start with 5 ball triplets (1 in robot, 4 on field), decrements every LAUNCH

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
            startPickup4,
            grabPickup4,
            scorePickup4,
            hitRelease,
            goToEnd;

    public void buildPaths() {
        config = buildConfig(); // get our config

        // this path goes from the starting point to our scoring point
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, config.scoreIntermediatePose, config.scorePose)) // test if this works
                .setLinearHeadingInterpolation(startPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime) // hopefully this works
                .build();

        // this path goes from the score point to the beginning of the 1st set of balls
        startPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup1StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup1StartPose.getHeading(), Tunables.grabEndTime)
                .build();

        // this path picks up the 1st set of balls
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup1StartPose, config.pickup1EndPose))
                .setLinearHeadingInterpolation(config.pickup1StartPose.getHeading(), config.pickup1EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup to our score position
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup1EndPose, config.scorePose))
                .setLinearHeadingInterpolation(config.pickup1EndPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime)
                .build();

        // this path goes from the score point to the beginning of the 2nd set of balls
        startPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup2StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup2StartPose.getHeading(), Tunables.grabEndTime)
                .build();

        // this path picks up the 2nd set of balls
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup2StartPose, config.pickup2EndPose))
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.pickup2EndPose.getHeading())
                .build();

        // this path hits the release once we are done with grabbing the second pickup, so we don't lose any pts to overflow
        hitRelease = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup2EndPose, config.pickup2StartPose))
                .setConstantHeadingInterpolation(config.pickup2StartPose.getHeading())
                .addPath(new BezierCurve(config.pickup2StartPose, config.releasePose))
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.releasePose.getHeading(), Tunables.clearEndTime)
                .build();

        // this path goes from the endpoint of the ball pickup to our score position
        scorePickup2 = follower.pathBuilder() // extra 2 lines to prevent hitting anything
                .addPath(new BezierCurve(config.releasePose, config.scorePose)) // test if this works
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime) // this heading should work
                .build();

        // this path goes from the score point to the beginning of the 3rd set of balls
        startPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup3StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup3StartPose.getHeading(), Tunables.grabEndTime)
                .build();

        // this path picks up the 3rd set of balls
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup3StartPose, config.pickup3EndPose))
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.pickup3EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup our score position
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup3EndPose, config.scorePose))
                .setLinearHeadingInterpolation(config.pickup3EndPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime)
                .build();

        // this path goes from the score point to the beginning of the 4th set of balls
        startPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(config.scorePose, config.pickup4StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup4StartPose.getHeading())
                .build();

        // this path picks up the 4th set of balls
        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup4StartPose, config.pickup4EndPose))
                .setLinearHeadingInterpolation(config.pickup4StartPose.getHeading(), config.pickup4EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup to our score position
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup4EndPose, config.scorePose))
                .setLinearHeadingInterpolation(config.pickup4EndPose.getHeading(), config.scorePose.getHeading())
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
            case START: // while we are traveling to our first launch
                if (!follower.isBusy()) {
                    setPathState(State.LAUNCH);
                }
                break;
            case TRAVEL_TO_LAUNCH:
                if (!follower.isBusy() // check if our follower is busy
                        && robot.isLaunchWithinMargin() // check if our launch velocity is within our margin
                        && follower.getPose().roughlyEquals(config.scorePose, Tunables.launchDistanceMargin)) { // check if we're holding position close enough to where we want to shoot
                    robot.intake.setPower(1); // turn on intake
                    setPathState(State.LAUNCH);
                }
                break;
            case LAUNCH:
                // robot.updateLaunch() only returns true when we are done launching balls, so otherwise we just break
                if (robot.updateLaunch()) { // we're done with launching balls
                    ballTripletsRemaining -= 1;
                    robot.intake.setPower(0); // save power
                    if (ballTripletsRemaining == 0) { // if we're out of balls, just go to the end
                        follower.followPath(goToEnd); // start going to end
                        setPathState(State.GO_TO_END);
                        break;
                    }
                    switch (ballTripletsRemaining) { // this should always be between 4 and 1
                        case 4:
                            follower.followPath(startPickup1);
                            break;
                        case 3:
                            follower.followPath(startPickup2);
                            break;
                        case 2:
                            follower.followPath(startPickup3);
                            break;
                        case 1:
                            follower.followPath(startPickup4);
                            break;
                    }
                    setPathState(State.TRAVEL_TO_BALLS);
                } // if we're not done with launching balls, just break
                break;
            case TRAVEL_TO_BALLS: // traveling to the start position of the balls, but not grabbing them just yet
                if(!follower.isBusy()) { // we're done traveling with balls, let's get ready to grab them
                    robot.intake.setPower(1); // re-enable intake to pickup balls
                    switch (ballTripletsRemaining) { // this should always be between 3 and 0
                        case 4:
                            follower.followPath(grabPickup1);
                            break;
                        case 3:
                            follower.followPath(grabPickup2);
                            break;
                        case 2:
                            follower.followPath(grabPickup3);
                            break;
                        case 1:
                            follower.followPath(grabPickup4);
                            break;
                    }
                    setPathState(State.RELOAD); // now that we're reloaded, let's go to launch
                }
                break;
            case RELOAD: // grab the balls in a straight line
                if(!follower.isBusy()) { // we're done reloading balls
                    robot.intake.setPower(0); // disable intake to save power
                    if (ballTripletsRemaining == 3) {
                        follower.followPath(hitRelease, false);
                        setPathState(State.GO_TO_CLEAR);
                        break; // exit without running the rest of the function
                    }
                    switch (ballTripletsRemaining) { // this should always be between 4 and 1
                        case 4:
                            follower.followPath(scorePickup1, true); // hold end to prevent other robots from moving us
                            break;
                        case 2:
                            follower.followPath(scorePickup3, true);
                            break;
                        case 1:
                            follower.followPath(scorePickup4, true);
                            break;
                    }
                    robot.launchBalls(3); // set up to launch 3 balls, it should not start launching until we call robot.updateLaunch()
                    setPathState(State.TRAVEL_TO_LAUNCH);
                }
                break;
            case GO_TO_CLEAR:
                if (!follower.isBusy()) { // wait until we are done moving
                    setPathState(State.CLEAR);
                }
                break;
            case CLEAR:
                if (stateTimer.getElapsedTime() >= Tunables.clearTime) { // wait until we're done clearing
                    follower.followPath(scorePickup2);
                    setPathState(State.TRAVEL_TO_LAUNCH);
                }
                break;
            case GO_TO_END: // travels to the end
                if(!follower.isBusy()) {
                    setPathState(State.END); // we're completely done now
                }
                break;
            case END:
                Robot.switchoverPose = follower.getPose(); // try to prevent drift
                follower.deactivateAllPIDFs(); // stop the follower
                telemetryM.addData("total auto time", opModeTimer.getElapsedTimeSeconds());
                telemetryM.update(telemetry);
                requestOpModeStop(); // request to stop our OpMode so it auto transfers to TeleOp
                break;
        }
    }

    // change our state, and update our timer for it
    public void setPathState(State pState) {
        state = pState;
        stateTimer.resetTimer();
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
        telemetryM.addData("balls remaining", robot.getBallsRemaining());
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
        stateTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        robot = Robot.getInstance(hardwareMap); // create our robot class

        follower = Constants.createFollower(hardwareMap);
        startPose = getStartPose(); // the getStartPose method will be included in different classes for start points
        buildPaths(); // this will create our paths from our predefined variables
        follower.setStartingPose(startPose); // this will set our starting pose from our getStartPose() function
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // this gets our telemetryM object so we can write telemetry to Panels
        telemetryM.debug("init time (millis): " + loopTimer.getElapsedTime()); // i don't think addData works in init()
        telemetryM.update(telemetry);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    /* not needed
    @Override
    public void init_loop() {}
    */

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opModeTimer.resetTimer();
        robot.resetServos(); // get servos ready
        robot.intake.setPower(1); // start intake
        robot.setLaunchVelocity(robot.RPMToTPS(Tunables.scoreRPM)); // we're just gonna keep our score RPM constant for now
        follower.followPath(scorePreload);
        robot.launchBalls(3);
        setPathState(State.START);
    }

    // everything else should automatically disable, but we should probably reset our servos just in case
    @Override
    public void stop() {
        robot.resetServos(); // return servos to starting position
    }
}