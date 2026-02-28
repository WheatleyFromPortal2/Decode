package org.firstinspires.ftc.teamcode.tuner;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsys.Vision;

@TeleOp(name="ObeliskTuner", group="Tuner")
public class ObeliskTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);

        vision.startPipeline(Vision.Pipeline.OBELISK);

        waitForStart();

        while (opModeIsActive()) {
            vision.updateObelisk();

            LLResult result = vision.limelight.getLatestResult();

            if (result.getFiducialResults().isEmpty()) {
                telemetry.addLine("fiducial results is empty!");
            } else {
                for (int i = 0; i < result.getFiducialResults().size(); i++) {
                    telemetry.addData("id", result.getFiducialResults().get(i).getFiducialId());
                }
            }

            telemetry.addData("code", vision.getLastTriplet());
            telemetry.update();
        }
    }
}
