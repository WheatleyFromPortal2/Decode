/** this OpMode is used to test/tune the turret **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsys.Vision;

@TeleOp(name="TurretTuner", group="Tuner")
public class TurretTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Vision vision = new Vision(hardwareMap, true);

        // reset turretEncoder
        robot.turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager
        vision.start();

        waitForStart();
        while (opModeIsActive()) {
            robot.update();
            vision.update();
            robot.applyTxToTurret(vision.getLastGoalTx(), vision.isStale());

            if (gamepad1.dpadLeftWasReleased()) robot.setDesiredTurretPosition(Math.toRadians(-45));
            if (gamepad1.dpadUpWasReleased()) robot.setDesiredTurretPosition(Math.toRadians(0));
            if (gamepad1.dpadRightWasReleased()) robot.setDesiredTurretPosition(Math.toRadians(45));
            //if (gamepad1.dpadDownWasReleased()) robot.setDesiredTurretPosition(Math.toRadians(180));

            telemetryM.addData("lastGoalTx", vision.getLastGoalTx());
            telemetryM.addData("turret error (deg)", Math.toDegrees(robot.getDesiredTurretPosition() - robot.getTurretPosition()));
            telemetryM.addData("desired turret position", robot.getDesiredTurretPosition());
            telemetryM.addData("turret position", robot.getTurretPosition());
            telemetryM.addData("turret velocity", robot.getTurretVelocity());
            telemetryM.addData("turret1 pos", robot.turret1.getPosition());
            telemetryM.addData("turret2 pos", robot.turret2.getPosition());

            telemetryM.update(telemetry);
            idle();
        }
    }
}
