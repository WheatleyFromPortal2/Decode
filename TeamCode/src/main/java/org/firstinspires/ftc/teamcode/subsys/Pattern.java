package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tunables;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Queue;

public class Pattern {
    public enum ball {
        PURPLE,
        GREEN
    }
    private enum State {
        IDLE,
        READ,
        SORT,
        WAIT
    }

    private State status;
    private boolean cache = false;

    Queue<ball> queue;
    Deque<ball> cached;
    Indexer indexer;

    private DigitalChannel upIndex;
    private DigitalChannel downIndex;
    private DigitalChannel middleIndex;
    private RevColorSensorV3 color;
    private ElapsedTime time;

    public Pattern(Queue<ball> queue, Indexer indexer, HardwareMap hw) {
        this.queue = queue;
        this.indexer = indexer;
        upIndex = hw.get(DigitalChannel.class, "upIndex");
        downIndex = hw.get(DigitalChannel.class, "downIndex");
        middleIndex = hw.get(DigitalChannel.class, "middleIndex");
        color = hw.get(RevColorSensorV3.class, "color");

        status = State.IDLE;
        cached = new ArrayDeque<>();
        time = new ElapsedTime();
    }

    public void update() {
        ball target = queue.poll();
        switch (status) {
            case IDLE:
                cache=false;
                ball cd = cached.poll();
                if (cd!=null && cd==target) {
                    status = State.SORT;
                    break;
                }
                indexer.blockerClosed();
                indexer.kickerUp();
                if (middleIndex()) {
                    status = State.READ;
                    Tunables.ballCount++;
                }
                break;
            case READ:
                ball current = color();
                if (target==null || target==current) {
                    cache = false;
                } else {
                    cache = true;
                    cached.addFirst(current);
                }
                status = State.SORT;
                break;
            case SORT:
                if (cache) {
                    indexer.blockerOpen();
                } else {
                    indexer.kickerDown();
                }
                status=State.WAIT;
                time.reset();
                break;
            case WAIT:
                if (time.milliseconds()>Tunables.indexWait) {
                    status= State.IDLE;
                }
                break;
        }
    }

    public boolean upIndex() {
        return upIndex.getState();
    }

    public boolean downIndex() {
        return downIndex.getState();
    }

    public boolean middleIndex() {
        return middleIndex.getState();
    }

    public ball color() {
        int p = color.red();
        if (p > Tunables.purpleBot && p < Tunables.purpleTop) {
            return ball.PURPLE;
        } else {
            return ball.GREEN;
        }
    }
}
