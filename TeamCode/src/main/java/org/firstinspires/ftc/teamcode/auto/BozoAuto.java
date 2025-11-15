package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve; // hopefully use these in the future
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot;


//@Autonomous(name = "BozoAuto", group = "Auto") // this is the base file which will just be extended so we don't want this OpMode to be directly run
public abstract class BozoAuto extends OpMode {
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();
    protected abstract Pose getStartPose();
    Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private TelemetryManager telemetryM; // create our telemetry object
    private Pose startPose;

    public enum State { // define our possible states for our FSM
        START, // starting state, waiting for OpMode to begin
        TRAVEL_TO_LAUNCH, // travel to our defined position to launch balls from. an internal switch statement control which path it will take
        LAUNCH, // launch our balls
        TRAVEL_TO_BALLS, // travel to the starting point of gathering balls
        RELOAD, // drive in the straight line to grab the balls
        GO_TO_END, // travel to our end position
        END // end state: do nothing
    }

    // these are the **only 2 variables** that should change throughout the auto
    State state = State.START; // set PathState to start
    private int ballTripletsRemaining = 4; // start with 4 ball triplets, decrements every launch

    // variables to be tuned
    // TODO: tune these
    private final double scoreRPM = 2700; // 2850 pretty good
    private final int interLaunchWait = 1000; // wait 0.5s between ball launches
    private final double scoreEndTime = 0.3; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations
    private final double grabEndTime = 0.8; // this defines how long Pedro Pathing should wait until reaching its target heading, lower values are more precise but run the risk of oscillations


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

        Robot.goalPose = config.goalPose; // add our goalPose to the Robot class so it can be used in teleop

        // this path goes from the starting point to our scoring point
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, config.scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), config.scorePose.getHeading(), scoreEndTime)
                .build();

        // this path goes from the score point to the beginning of the first set of balls
        startPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup1StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup1StartPose.getHeading(), grabEndTime)
                .build();

        // this path picks up the first set of balls
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup1StartPose, config.pickup1EndPose))
                .setLinearHeadingInterpolation(config.pickup1StartPose.getHeading(), config.pickup1EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup to our score position
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup1EndPose, config.scorePose))
                .setLinearHeadingInterpolation(config.pickup1EndPose.getHeading(), config.scorePose.getHeading(), scoreEndTime)
                .build();

        // this path goes from the score point to the beginning of the second set of balls
        startPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup2StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup2StartPose.getHeading(), grabEndTime)
                .build();

        // this path picks up the second set of balls
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup2StartPose, config.pickup2EndPose))
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.pickup2EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup to our score position
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup2EndPose, config.scorePose))
                .setLinearHeadingInterpolation(config.pickup2EndPose.getHeading(), config.scorePose.getHeading(), scoreEndTime)
                .build();

        // this path goes from the score point to the beginning of the third set of balls
        startPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.pickup3StartPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.pickup3StartPose.getHeading(), grabEndTime)
                .build();

        // this path picks up the third set of balls
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup3StartPose, config.pickup3EndPose))
                .setLinearHeadingInterpolation(config.pickup2StartPose.getHeading(), config.pickup3EndPose.getHeading())
                .build();

        // this path goes from the endpoint of the ball pickup our score position
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(config.pickup3EndPose, config.scorePose))
                .setLinearHeadingInterpolation(config.pickup3EndPose.getHeading(), config.scorePose.getHeading(), scoreEndTime)
                .build();

        // this path goes form our score point to our ending position
        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(config.scorePose, config.endPose))
                .setLinearHeadingInterpolation(config.scorePose.getHeading(), config.endPose.getHeading())
                .build();
    }

    // example FSM
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
                setPathState(State.LAUNCH);
                break;
            case TRAVEL_TO_LAUNCH:
                if(!follower.isBusy()) {
                    if (ballTripletsRemaining == 0) { // if we're out of balls, just go to the end
                        state = State.END;
                        break;
                    }
                    switch (ballTripletsRemaining) { // this should always be between 3 and 0
                        case 3:
                            follower.followPath(scorePickup1);
                            break;
                        case 2:
                            follower.followPath(scorePickup2);
                            break;
                        case 1:
                            follower.followPath(scorePickup3);
                            break;
                    }
                    state = State.LAUNCH; // let's launch
                }
                break;
            case LAUNCH:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() && robot.isLaunchWithinMargin(scoreRPM)) { // check if we're busy and if our launch velocity is within our margin
                    follower.pausePathFollowing(); // pause path following while launching (idk why its following?!?)
                    robot.launchBall(); // launch our first ball
                    sleep(interLaunchWait); // could rework this to also watch for velocity
                    robot.launchBall(); // launch our second ball
                    sleep(interLaunchWait);
                    robot.launchBall(); // launch our third ball
                    sleep(interLaunchWait); // make sure ball has fully exited robot
                    ballTripletsRemaining -= 1; // we have launched a triplet of balls
                    setPathState(State.TRAVEL_TO_BALLS); // let's get some more balls!
                    follower.resumePathFollowing();
                }
                break;
            case TRAVEL_TO_BALLS: // travel to the start position of the balls, but don't grab them yet
                if(!follower.isBusy()) {
                    if (ballTripletsRemaining == 0) { // if we're out of balls, just go to the end
                        state = State.END;
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
                    follower.followPath(goToEnd,true);
                    setPathState(State.END);
                }
            case END:
                robot.intake.setPower(0); // turn off intake
                robot.launch.setPower(1); // warm up launch
                follower.deactivateAllPIDFs(); // stop the follower
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
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub for debugging
        telemetryM.debug("path state", state);
        telemetryM.debug("is follower busy", follower.isBusy());
        telemetryM.debug("ball triplets remaining", ballTripletsRemaining);
        telemetryM.debug("desired launch RPM", scoreRPM);
        telemetryM.debug("launch RPM", robot.getLaunchRPM());
        telemetryM.debug("x", follower.getPose().getX());
        telemetryM.debug("y", follower.getPose().getY());
        telemetryM.debug("heading", follower.getPose().getHeading());
        telemetryM.update(telemetry); // update telemetry
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        // set up our timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot = Robot.getInstance(hardwareMap); // create our robot class

        follower = Constants.createFollower(hardwareMap);
        startPose = getStartPose(); // the getStartPose method will be included in different classes for start points
        buildPaths(); // this will create our paths from our predefined variables
        follower.setStartingPose(startPose); // this will set our starting pose from our getStartPose() function
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // this gets our telemetryM object so we can write telemetry to Panels
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        robot.initServos(); // get servos ready
        robot.intake.setPower(1); // start intake
        robot.launch.setVelocity(robot.RPMToTPS(scoreRPM)); // we're just gonna keep our score RPM constant for now
        //robot.launch.setVelocity(2800); // full speed BRRRR
        setPathState(State.START);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}