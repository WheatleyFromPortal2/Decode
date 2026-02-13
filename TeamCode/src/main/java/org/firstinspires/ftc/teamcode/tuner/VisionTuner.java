/** the purpose of OpMode is to test our vision code */

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsys.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.util.Timer;

@TeleOp(name="VisionTuner", group="Tuner")
public class VisionTuner extends LinearOpMode {
    public boolean isBlueTeam = true;

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
        Follower follower = Constants.createFollower(hardwareMap);
        Robot robot = new Robot(hardwareMap);
        Timer loopTimer = new Timer();

        vision.startPipeline(Vision.Pipeline.FULL_POS);
        follower.setStartingPose(HandoffState.pose); // should be middle of field by default

        waitForStart();

        while (opModeIsActive()) {
            loopTimer.resetTimer();
            follower.update();
            vision.updateFullPos(follower.getHeading(), robot.turret.getPos());

            telemetryM.addData("loop time millis (without telemetry)", loopTimer.getElapsedTime());
            telemetryM.addData("stale?", vision.isStale());
            telemetryM.addData("lastBotPose", vision.getLastBotPose());
            telemetryM.addData("pipeline", vision.getPipeline());
            telemetryM.addData("limelight status", vision.getStatus());
            telemetryM.update(telemetry); // update our telemetry
            idle();
        }
    }
}