package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Queue;

public class Pattern {
    public enum ball {
        PURPLE,
        GREEN
    }

    Queue<ball> queue;
    Indexer indexer;

    private DigitalChannel upIndex;
    private DigitalChannel downIndex;
    private DigitalChannel purplePin;
    private DigitalChannel greenPin;

    public Pattern(Queue<ball> queue, Indexer indexer, HardwareMap hw) {
        this.queue = queue;
        this.indexer = indexer;
        upIndex = hw.get(DigitalChannel.class, "upIndex");
        downIndex = hw.get(DigitalChannel.class, "downIndex");
        purplePin = hw.get(DigitalChannel.class, "purplePin");
        greenPin = hw.get(DigitalChannel.class, "greenPin");
    }

    public void update() {

    }

    public boolean upIndex() {
        return upIndex.getState();
    }

    public boolean downIndex() {
        return downIndex.getState();
    }

    public boolean purple() {
        return purplePin.getState();
    }

    public boolean green() {
        return greenPin.getState();
    }
}
