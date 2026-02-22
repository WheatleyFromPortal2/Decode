package org.firstinspires.ftc.teamcode.tuner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsys.Indexer;
import org.firstinspires.ftc.teamcode.subsys.launch.Intake;
import org.firstinspires.ftc.teamcode.subsys.launch.Transfer;

@TeleOp(name = "Indexer Test", group = "tuner")
public class IndexerTest extends LinearOpMode {

    Indexer indexer;
    Intake intake;
    Transfer transfer;

    @Override
    public void runOpMode() throws InterruptedException {
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);

        waitForStart();

        while (opModeIsActive() & !isStopRequested()) {
            if (gamepad1.a) {
                intake.forward();
            } else {
                intake.hold();
            }

            if (gamepad1.left_bumper) {
                indexer.blockerClosed();
            } else {
                indexer.blockerOpen();
            }

            if (gamepad1.left_trigger>0.1 && !gamepad1.right_bumper) {
                indexer.kickerChute();
            }

            if (gamepad1.right_bumper) {
                indexer.kickerUp();
            } else {
                indexer.kickerDown();
            }

            if (gamepad1.b) {
                transfer.forward();
            } else {
                transfer.off();
            }
            if (gamepad1.x && !gamepad1.b) {
                transfer.reverse();
            }
        }
    }
}
