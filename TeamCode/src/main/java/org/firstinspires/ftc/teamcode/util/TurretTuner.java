/** this OpMode is used to test/tune the turret **/

package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

@TeleOp(name="LaunchDelay", group="Util")
public class TurretTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance(hardwareMap);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        waitForStart();
        while (opModeIsActive()) {
            robot.calcPIDF();

            telemetryM.addData("turret position", robot.getTurretPosition());
            telemetryM.addData("turret position (degrees)", Math.toDegrees(robot.getTurretPosition()));

            telemetryM.update();
            idle();
        }
    }
}
