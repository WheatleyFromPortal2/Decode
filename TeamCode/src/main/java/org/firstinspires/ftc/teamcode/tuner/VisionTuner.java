/** the purpose of this OpMode is to test our vision code */

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.subsys.Fusion;
import org.firstinspires.ftc.teamcode.subsys.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsys.launch.Intake;
import org.firstinspires.ftc.teamcode.subsys.launch.Turret;

import com.pedropathing.util.Timer;

@TeleOp(name="VisionTuner", group="Tuner")
public class VisionTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);
        Fusion fusion = new Fusion(HandoffState.pose);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
        Follower follower = Constants.createFollower(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap, intake.getMotor());
        Timer loopTimer = new Timer();

        vision.startPipeline(Vision.Pipeline.FULL_POS);
        follower.setStartingPose(HandoffState.pose); // should be middle of field by default

        waitForStart();

        while (opModeIsActive()) {
            loopTimer.resetTimer();

            follower.update();
            vision.updateFullPos(follower.getHeading(), turret.getPos());

            fusion.update(follower.getPose(), vision.getLastBotPose());
            Pose fusionPose = fusion.getState();

            telemetryM.addData("loop time millis (without telemetry)", loopTimer.getElapsedTime());
            telemetryM.addData("stale?", vision.isStale());
            Pose lastBotPose = vision.getLastBotPose();
            if (lastBotPose != null) {
                telemetryM.addData("lastBotPose", vision.getLastBotPose());
            } else {
                telemetryM.debug("last bot pose null");
            }
            telemetryM.addData("odo pose", follower.getPose());
            telemetryM.addData("fusion pose", fusionPose);
            telemetryM.addData("pipeline", vision.getPipeline());
            telemetryM.addData("limelight status", vision.getStatus());
            telemetryM.update(telemetry); // update our telemetry
            idle();
        }
    }
}