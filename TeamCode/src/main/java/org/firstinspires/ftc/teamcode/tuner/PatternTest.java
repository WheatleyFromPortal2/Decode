package org.firstinspires.ftc.teamcode.tuner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.Indexer;
import org.firstinspires.ftc.teamcode.subsys.Pattern;
import org.firstinspires.ftc.teamcode.subsys.Pattern.Ball;

import java.util.Queue;

@TeleOp(name="Pattern Test", group = "Tuner")
public class PatternTest extends LinearOpMode {

    Pattern pattern;
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        pattern = new Pattern(robot);
        pattern.inputString(Tunables.patternT);
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            pattern.update();
            telemetry.addData("State: ", pattern.getState());
            telemetry.update();
        }
    }
}
