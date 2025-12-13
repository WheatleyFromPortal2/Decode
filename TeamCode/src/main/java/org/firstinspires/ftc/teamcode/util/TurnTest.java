/** OpMode to test automated turning behaviours **/
package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TurnTest", group="Util")
public class TurnTest extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    private boolean automatedDrive = false;

    private enum TurnMode {
        A,
        B,
        C,
        D
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap); // create our Pedro Pathing follower
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // create our telemetry class
    }

    @Override
    public void loop() {
        follower.update();

        // stop automated following if the follower is done OR user presses start
        if (automatedDrive && (gamepad1.startWasReleased() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // robot centric
            );
        }

        // automated turning tests
        if (gamepad1.aWasReleased()) automatedTurn(TurnMode.A);
        if (gamepad1.bWasReleased()) automatedTurn(TurnMode.B);
        if (gamepad1.yWasReleased()) automatedTurn(TurnMode.C);
        if (gamepad1.xWasReleased()) automatedTurn(TurnMode.D);

        // telemetry
        telemetryM.addData("automatedDrive", automatedDrive);
        telemetryM.addData("isTeleOpDrive", follower.isTeleopDrive());
        telemetryM.addData("isBusy", follower.isBusy());

        telemetryM.update(telemetry); // update telemetry
    }

    private void automatedTurn(TurnMode turnMode) {
        switch (turnMode) {
            // don't want to use follower.turnToDegrees because I want to keep this OpMode as close to TeleOp as possible for testing
            case A:
                follower.turnTo(Math.toRadians(0));
                break;
            case B:
                follower.turnTo(Math.toRadians(90));
                break;
            case C:
                follower.turnTo(Math.toRadians(180));
                break;
            case D:
                follower.turnTo(Math.toRadians(270));
                break;
        }
        automatedDrive = true; // stop teleop control
    }
}
