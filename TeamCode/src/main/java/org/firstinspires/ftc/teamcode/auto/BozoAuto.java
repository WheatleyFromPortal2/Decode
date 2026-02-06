/** this is our base class for all of our autos
 * it is extended by BlueAuto/RedAuto
 * and then extended again by ...TriAuto/...GoalAuto
 **/

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

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Tunables;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot;


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
        RELOAD, // drive in the straight line with intake on to grab balls
        GO_TO_TURN,
        TURN,
        GO_TO_CLEAR, // go to the clear gate
        CLEAR, // wait at the clear gate for the balls
        GO_TO_END, // travel to our end position
        END // end state: do nothing
    }

    private final int maxBallTriplets = 4; // amount of ball triplets we will try to score before going to end

    /** these are the **only variables** that should change throughout the auto **/

    State state = State.START; // set PathState to start
    private int ballTripletsScored = 0; // start with 4 ball triplets (1 in robot, 3 on field), decrements every launch

    /** end vars that can change **/

    // example paths
    private PathChain // some of these can probably just be Paths, but whatever
            scorePreload,
            startPickup1,
            grabPickup1,
            scorePickup1,
            startPickup2,
            grabPickup2,
            scorePickup2,
            goToTurn,
            scoreClear,
            startPickup3,
            grabPickup3,
            scorePickup3,
            getClear,
            goToEnd;

    public void buildPaths() {
        config = buildConfig(); // get our config

        // this path goes from the starting point to our scoring point
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, config.scorePose)) // test if this works
                //.addPath(new BezierCurve(startPose, config.scoreIntermediatePose, config.scorePose)) // test if this works
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
                .setVelocityConstraint(Tunables.maxGrabVelocity)
                .build();

        // this path goes from the endpoint of the ball pickup to our score position
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.releasePose, config.scorePose))
                .setLinearHeadingInterpolation(config.releasePose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime)
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
                .setVelocityConstraint(Tunables.maxGrabVelocity)
                .build();

        // this path goes from the end of the 2nd set of balls to our score position
        scorePickup2 = follower.pathBuilder() // extra 2 lines to prevent hitting anything
                .addPath(new BezierCurve(config.pickup2EndPose, config.pickup2StartPose, config.scorePose)) // test if this works
                .setLinearHeadingInterpolation(config.pickup2EndPose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime) // this heading should work
                .build();

        double midX = (config.pickup1StartPose.getX() + config.pickup1EndPose.getX()) / 2;
        Pose releaseMidPose = new Pose(midX, config.pickup1EndPose.getY());
        // this path gets our balls from clear from our scorePose
        goToTurn = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup1EndPose, releaseMidPose)) // to prevent coming in at a weird angle, we first go to our pickup2StartPose
                .setConstantHeadingInterpolation(config.pickup1EndPose.getHeading())
                .build();

        getClear = follower.pathBuilder()
                .addPath(new BezierLine(releaseMidPose, config.releasePose))
                .setConstantHeadingInterpolation(config.releasePose.getHeading())
                .build();

        // this path goes from the release position to the scoring position
        /*
        scoreClear = follower.pathBuilder()
                .addPath(new BezierCurve(config.releasePose, config.pickup2StartPose, config.scorePose)) // use the pickup2StartPose so it backs out more directly
                .setLinearHeadingInterpolation(config.releasePose.getHeading(), config.scorePose.getHeading(), Tunables.scoreEndTime) // this heading should work
                .build(); */

        // this path goes from the score point to the beginning of the 3rd set of balls
        startPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup3StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup3StartPose.getHeading(), Tunables.grabEndTime)
                .build();

        // this path picks up the 3rd set of balls
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup3StartPose, config.pickup3EndPose))
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.pickup3EndPose.getHeading())
                .setVelocityConstraint(Tunables.maxGrabVelocity)
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
    public void autonomousPathUpdate() {
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
        switch (state) {
            case START:
                follower.followPath(scorePreload);
                robot.launchBalls(3);
                setPathState(State.TRAVEL_TO_LAUNCH);
                break;
            case TRAVEL_TO_LAUNCH:
                if (!follower.isBusy() // check if our follower is busy
                        && robot.isLaunchWithinMargin() // check if our launch velocity is within our margin
                        && follower.getPose().roughlyEquals(config.scorePose, Tunables.launchDistanceMargin)) { // check if we're holding position close enough to where we want to shoot
                    //follower.holdPoint(config.scorePose); // this should already be done by holdEnd: true
                    /* if we're holding point, we shouldn't have to disable motors
                    follower.pausePathFollowing();
                    follower.deactivateAllPIDFs(); */
                    robot.intake.setPower(1); // turn on intake so transfer can work
                    robot.launchBalls(3); // set up to launch 3 balls, it should not start launching until we call robot.updateLaunch()
                    setPathState(State.LAUNCH); // let's launch
                }
                break;
            case LAUNCH:
                if (robot.updateLaunch()) { // we're done with launching balls
                    ballTripletsScored++; // increment the amount of triplets that we have scored if we have a successful launch
                    robot.intake.setPower(0); // save power
                    setPathState(State.TRAVEL_TO_BALLS);
                    /* if we're holding point, we shouldn't have to re-enable motors
                    follower.activateAllPIDFs();
                    follower.resumePathFollowing(); */

                    if (ballTripletsScored == maxBallTriplets) { // if we're out of balls, just go to the end
                        follower.followPath(goToEnd);
                        setPathState(State.GO_TO_END);
                        break;
                    } else {
                        switch (ballTripletsScored) { // this should always be between 3 and 0
                            case 1:
                                follower.followPath(startPickup1);
                                setPathState(State.TRAVEL_TO_BALLS); // now we reload
                                break;
                            case 2:
                                follower.followPath(startPickup2);
                                setPathState(State.TRAVEL_TO_BALLS); // now we reload
                                break;
                            case 3:
                                follower.followPath(startPickup3);
                                setPathState(State.TRAVEL_TO_BALLS); // now we reload
                                break;
                            default: // if we have another amount of ball triplets scored, then crash the program and report the error
                                throw new IllegalStateException("[LAUNCH] invalid amount of ballTripletsScored: " + ballTripletsScored);
                        }
                    }
                } // if we're not done with launching balls, just break
                break;
            case TRAVEL_TO_BALLS: // travel to the start position of the balls, but don't grab them yet
                if(!follower.isBusy()) {
                    robot.intake.setPower(1); // re-enable intake to pickup balls
                    switch (ballTripletsScored) { // this should always be between 3 and 0
                        case 1:
                            follower.followPath(grabPickup1);
                            break;
                        case 2:
                            follower.followPath(grabPickup2);
                            break;
                        // case 3: clearing (done in LAUNCH)
                        case 3:
                            follower.followPath(grabPickup3);
                            break;
                        default: // if we have another amount of ball triplets scored, then crash the program and report the error
                            throw new IllegalStateException("[TRAVEL_TO_BALLS] invalid amount of ballTripletsScored: " + ballTripletsScored);
                    }
                    setPathState(State.RELOAD); // now that we're reloaded, let's go to launch
                }
                break;
            case RELOAD: // grab the balls in a straight line
                if (!follower.isBusy()) {
                    switch (ballTripletsScored) { // this should always be between 3 and 0
                        case 1:
                            //follower.followPath(scorePickup1, true);
                            follower.followPath(goToTurn, true);
                            robot.intake.setPower(0);
                            setPathState(State.GO_TO_TURN);
                            break;
                        case 2:
                            follower.followPath(scorePickup2, true); // hold end to prevent other robots from moving us
                            setPathState(State.TRAVEL_TO_LAUNCH);
                            break;
                        // case 3: clearing (done in LAUNCH)
                        case 3:
                            follower.followPath(scorePickup3, true);
                            setPathState(State.TRAVEL_TO_LAUNCH);
                            break;
                        default: // if we have another amount of ball triplets scored, then crash the program and report the error
                            throw new IllegalStateException("[RELOAD] invalid amount of ballTripletsScored: " + ballTripletsScored);
                    }
                }
            case GO_TO_TURN:
                if (!follower.isBusy()) {
                    follower.turnTo(config.releasePose.getHeading());
                    setPathState(State.TURN);
                }
                break;
            case TURN:
                if (!follower.isBusy()) {
                    follower.followPath(getClear);
                    setPathState(State.GO_TO_CLEAR);
                }
            case GO_TO_CLEAR:
                if (!follower.isBusy()) {
                    setPathState(State.CLEAR); // this will also reset stateTimer, allowing us to measure how long we have been at clear
                }
                break;
            case CLEAR:
                if (stateTimer.getElapsedTime() >= Tunables.clearTime) { // we have waited long enough at clear
                    follower.followPath(scorePickup1);
                    setPathState(State.TRAVEL_TO_LAUNCH);
                }
                break;
            case GO_TO_END: // travels to the end
                robot.setDesiredTurretPosition(0); // lock turret in middle
                if(!follower.isBusy()) {
                    setPathState(State.END);
                }
                break;
            case END:
                robot.intake.setPower(0); // turn off intake
                updateHandoff();
                follower.deactivateAllPIDFs(); // stop the follower
                requestOpModeStop(); // request to stop our OpMode so it auto transfers to TeleOp
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    private void setPathState(State pState) {
        state = pState;
        stateTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        loopTimer.resetTimer();
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        robot.calcPIDF();

        updateHandoff();

        autonomousPathUpdate(); // update our state machine and run its actions

        if (Tunables.isDebugging) {
            sendTelemetry(false); // we don't want to send our init time now that our OpMode is running
        }

        telemetryM.addData("loop time (millis)", loopTimer.getElapsedTime()); // we want to be able to graph this
        telemetryM.update(telemetry);
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        // set up our timers
        loopTimer = new Timer();
        loopTimer.resetTimer();
        stateTimer = new Timer();
        opModeTimer = new Timer();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // this gets our telemetryM object so we can write telemetry to Panels
        robot = new Robot(hardwareMap);
        robot.resetLaunchServos(); // get servos ready

        telemetryM.debug("creating follower... (this may take a while)");
        telemetryM.update(telemetry);

        follower = Constants.createFollower(hardwareMap);
        startPose = getStartPose(); // the getStartPose method will be included in different classes for start points

        telemetryM.debug("building paths... (this may take a while)");
        telemetryM.update(telemetry);

        buildPaths(); // this will create our paths from our predefined variables
        robot.setDesiredTurretPosition(config.scoreTurretPos); // we will just always keep our turret in score position
        robot.setHoodPosition(Tunables.scoreHoodPos); // we will just always keep our hood in the same position

        follower.setStartingPose(startPose); // this will set our starting pose from our getStartPose() function

        sendTelemetry(true); // begin our full telemetry, so in Panels we can set all graphs to true
        // this lets us observe our telemetry graphs from the start
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    // this could be used for detecting april tag pattern before we even begin

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opModeTimer.resetTimer();
        robot.resetLaunchServos(); // get servos ready
        robot.intake.setPower(1); // start intake
        robot.setLaunchVelocity(robot.RPMToTPS(Tunables.scoreRPM)); // we're just gonna keep our score RPM constant for now
        setPathState(State.START);
    }

    // everything else should automatically disable, but we should probably reset our servos just in case
    @Override
    public void stop() {
        updateHandoff(); // update our hand off when we stop
        // doesn't really work because stopping OpMode disables servo power
        robot.resetLaunchServos(); // return servos to starting position
    }

    public void sendTelemetry(boolean sendInitTime) {
        // sendInitTime = true; is only for init()
        if (sendInitTime) telemetryM.debug("init time (millis): " + loopTimer.getElapsedTime()); // i don't think addData works in init()

        // warnings!
        if (Math.abs(robot.getDesiredLaunchRPM() - robot.getLaunchRPM()) > 100) telemetryM.debug("WARNING: LAUNCH OUT OF 100RPM RANGE");

        // state
        telemetryM.debug("path state: " + state);
        telemetryM.debug("maxBallTriplets: " + maxBallTriplets);
        telemetryM.addData("ballTripletsScored", ballTripletsScored);
        telemetryM.debug("is follower busy?: " + follower.isBusy());

        // launch system
        telemetryM.addData("desiredLaunchRPM", robot.getDesiredLaunchRPM());
        telemetryM.addData("launch RPM", robot.getLaunchRPM());
        telemetryM.addData("ballsRemaining", robot.getBallsRemaining());
        telemetryM.debug("is launch within margin?: " + robot.isLaunchWithinMargin());

        // odo
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());

        // timing
        telemetryM.debug("OpMode time (seconds): " + opModeTimer.getElapsedTimeSeconds());
    }

    public void updateHandoff() {
        HandoffState.pose = follower.getPose();
        HandoffState.ballsRemaining = robot.getBallsRemaining();
    }
}