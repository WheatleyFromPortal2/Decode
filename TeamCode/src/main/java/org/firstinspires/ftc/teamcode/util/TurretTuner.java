/** this OpMode is used to test/tune the turret **/

package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

@TeleOp(name="TurretTuner", group="Util")
public class TurretTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance(hardwareMap);

        // reset turretEncoder
        robot.turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        waitForStart();
        while (opModeIsActive()) {
            robot.calcPIDF();

            if (gamepad1.dpadLeftWasReleased()) robot.setDesiredTurretPosition(Math.toRadians(-90));
            if (gamepad1.dpadUpWasReleased()) robot.setDesiredTurretPosition(Math.toRadians(0));
            if (gamepad1.dpadRightWasReleased()) robot.setDesiredTurretPosition(Math.toRadians(90));
            if (gamepad1.dpadDownWasReleased()) robot.setDesiredTurretPosition(Math.toRadians(180));

            telemetryM.addData("turret ticks", robot.turretEncoder.getCurrentPosition());
            telemetryM.addData("turret position", robot.getTurretPosition());
            telemetryM.addData("turret position (degrees)", Math.toDegrees(robot.getTurretPosition()));
            telemetryM.addData("turret1", robot.turret1.getPower());
            telemetryM.addData("turret2", robot.turret2.getPower());

            telemetryM.update(telemetry);
            idle();
        }
    }
}
