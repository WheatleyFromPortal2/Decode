package org.firstinspires.ftc.teamcode.tuner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsys.Indexer;
import org.firstinspires.ftc.teamcode.subsys.Pattern;
import org.firstinspires.ftc.teamcode.subsys.Pattern.Ball;

import java.util.Queue;

@TeleOp
public class PatternTest extends LinearOpMode {

    Pattern pattern;
    Indexer indexer;
    Queue<Ball> queue;

    @Override
    public void runOpMode() throws InterruptedException {
        queue.add(Ball.PURPLE);
        queue.add(Ball.PURPLE);
        queue.add(Ball.PURPLE);
        indexer = new Indexer(hardwareMap);
        pattern = new Pattern(queue, indexer, hardwareMap);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            pattern.update();
        }
    }
}
