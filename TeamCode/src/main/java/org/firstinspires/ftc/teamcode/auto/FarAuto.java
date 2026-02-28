package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsys.LaunchSetpoints;

public abstract class FarAuto extends OpMode {
    // we should only alternate between these two positions
    protected abstract Pose getStartPose(); // pose for launch/start
    protected abstract Pose getPickupPose();
    protected abstract double getTurretPos();

    private Pose startPose;
    private Pose pickupPose;

    private Robot robot;
    private Follower follower;
    private Timer stateTimer, loopTimer;
    private TelemetryManager telemetryM;

    private enum State {
        START, // run once at start of OpMode
        WAIT_LAUNCH, // wait before launching based off of times in Tunables
        LAUNCH, // launch balls
        RELOAD, // reload from corner
        TRAVEL_TO_LAUNCH, // travel back to start/launch position
        END // safely shut down OpMode
    }

    /** these are the **only variables** that should change during runtime **/

    private State state = State.START;
    private int cycle = 0;

    /** end vars that change **/

    private PathChain getCornerPickup, goToLaunch;

    private void buildPaths() {
        // go from start/launch to corner get balls
        getCornerPickup = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickupPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // go from corner to launch
        goToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, startPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }

    private void autoPathUpdate(boolean robotUpdateStatus) {
        switch (state) {
            case START:
                robot.intake.forward();
                robot.transfer.close();

                if (robot.flywheel.isWithinMargin()) {
                    setPathState(State.WAIT_LAUNCH);
                }
                break;
            case WAIT_LAUNCH:
                int wait;
                if (cycle >= Tunables.farLaunchWaits.length) {
                    // farLaunchWaits array is too short
                    wait = 0; // default to 0 wait
                    telemetryM.debug("No wait configured for cycle: " + cycle + ", defaulting to " + wait + "ms");
                } else {
                    wait = Tunables.farLaunchWaits[cycle];
                }

                if (stateTimer.getElapsedTime() >= wait) {
                    // we're done waiting; let's launch
                    setPathState(State.LAUNCH);
                    robot.launch();
                }
                break;
            case LAUNCH:
                if (robotUpdateStatus) {
                    cycle++;

                    if (cycle >= Tunables.farCycles) { // we're done
                        setPathState(State.END);
                    } else { // still have more cycles to do
                        follower.followPath(getCornerPickup);
                        setPathState(State.RELOAD);
                    }
                } // if we're not done with launching balls, just break
                break;
            case RELOAD:
                if (!follower.isBusy()) {
                    follower.followPath(goToLaunch);
                    setPathState(State.TRAVEL_TO_LAUNCH);
                }
                break;
            case TRAVEL_TO_LAUNCH:
                if (!follower.isBusy()) {
                    setPathState(State.WAIT_LAUNCH);
                }
                break;
            case END:
                updateHandoff();
                follower.deactivateAllPIDFs();
                requestOpModeStop();
                break;
        }
    }

    /** These changes our path state while also resetting its timer **/
    private void setPathState(State newState) {
        state = newState;
        stateTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        loopTimer.resetTimer();

        follower.update(); // update our follower before anything else

        updateHandoff();

        autoPathUpdate(robot.update());

        if (Tunables.isDebugging) {
            sendTelemetry(false);
        }

        telemetryM.addData("loop time (millis)", loopTimer.getElapsedTime());
        telemetryM.update(telemetry);
    }


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        startPose = getStartPose();
        pickupPose = getPickupPose();

        // set up our timers
        loopTimer = new Timer();
        stateTimer = new Timer();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // this gets our telemetryM object so we can write telemetry to Panels
        robot = new Robot(hardwareMap);

        LaunchSetpoints setpoints = new LaunchSetpoints(0,
                Tunables.hoodMinimumRadians, // use the lowest hood angle possible
                getTurretPos() - startPose.getHeading());

        setpoints.setRPM(Tunables.farScoreRPM);
        robot.turret.setDesiredPos(setpoints.getTurretPos());
        robot.turret.update();

        robot.setSetpoints(setpoints);
        robot.launch();

        telemetryM.debug("creating follower... (this may take a while)");
        telemetryM.update(telemetry);

        follower = Constants.createFollower(hardwareMap);

        telemetryM.debug("building paths... (this may take a while)");
        telemetryM.update(telemetry);

        buildPaths(); // this will create our paths from our predefined variables

        follower.setStartingPose(startPose);

        sendTelemetry(true); // send full telemetry
        // this lets us observe our telemetry graphs from the start
        telemetryM.update(telemetry);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetryM.addLine("INIT COMPLETE: ENSURE THAT CYCLE COUNT AND WAITS ARE CORRECT");

        if (Tunables.farCycles > Tunables.farLaunchWaits.length) {
            telemetryM.addLine("WARNING: NOT ENOUGH WAITS FOR CYCLES");
        }

        telemetryM.addLine("");

        // print out all config data so we can ensure that it's correct before our round
        telemetryM.debug("cycles", Tunables.farCycles);
        telemetryM.addLine("");
        telemetryM.addLine("waits (millis)");
        for (int i = 0; i < Tunables.farLaunchWaits.length; i++) { // loop thru all launch waits
            telemetryM.addLine(i + 1 + ". " + Tunables.farLaunchWaits[i]);
        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        setPathState(State.START);
    }

    // everything else should automatically disable, but we still want to update our handoff
    @Override
    public void stop() {
        updateHandoff(); // update our handoff when we stop
        // moving servos doesn't really work because stopping the OpMode disables servo power
    }

    public void updateHandoff() {
        HandoffState.pose = follower.getPose();
    }

    public void sendTelemetry(boolean sendInitTime) {
        // sendInitTime = true; is only for init()
        if (sendInitTime) {
            telemetryM.addLine("INIT COMPLETE: READY TO START");
            telemetryM.debug("init time (millis): " + loopTimer.getElapsedTime()); // i don't think addData works in init()
        } else { // don't send warnings at init
            if (!robot.flywheel.isWithinMargin()) telemetryM.debug("WARNING: LAUNCH OUT OF MARGIN");
        }


        // state
        telemetryM.debug("path state: " + state);
        telemetryM.addData("cycle", cycle);
        telemetryM.debug("is follower busy?: " + follower.isBusy());

        // launch system
        telemetryM.addData("launch RPM", robot.flywheel.getRPM());

        // odo
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());

        // timing
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}
