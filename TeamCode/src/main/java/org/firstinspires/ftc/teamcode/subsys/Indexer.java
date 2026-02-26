package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunables;

public class Indexer {
    private Servo kicker, blocker;

    private double lastKickerPos = -1; // force hardware write for first run
    private double lastBlockerPos = -1; // force hardware write for first run

    public Indexer(HardwareMap hw) {
        kicker = hw.get(Servo.class, "kickerServo");
        blocker = hw.get(Servo.class, "sorterServo");

        disable(); // set disabled to start
    }

    public void disable() { // set to normal mode (allow passthrough of balls)
        kickerDown();
        blockerClosed();
    }

    public void kickerDown() {
        setKickerPosFast(Tunables.indexerKickerIntake);
    }

    public void kickerUp() {
        setKickerPosFast(Tunables.indexerKickerUp);
    }

    public void kickerChute() {
        setKickerPosFast(Tunables.indexerKickerChute);
    }

    public void blockerClosed() {
        setBlockerPosFast(Tunables.indexerBlockerClosed);
    }

    public void blockerOpen() {
        setBlockerPosFast(Tunables.indexerBlockerOpen);
    }

    public void blockerUp() {
        setBlockerPosFast(Tunables.indexerBlockerUp);
    }

    public void setKickerPosFast(double newPos) {
        newPos = Range.clip(newPos, 0.0, 1.0);
        if (newPos != lastKickerPos) { // only write to hardware if new position
            kicker.setPosition(newPos);
            lastKickerPos = newPos;
        }
    }

    public void setBlockerPosFast(double newPos) {
        newPos = Range.clip(newPos, 0.0, 1.0);
        if (newPos != lastBlockerPos) { // only write to hardware if new position
            blocker.setPosition(newPos);
            lastBlockerPos = newPos;
        }
    }
}
