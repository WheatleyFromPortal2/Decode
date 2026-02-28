package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Queue;

public class Pattern {
    public enum Ball {
        PURPLE,
        GREEN
    }

    private enum State {
        IDLE,
        IDLE2,
        PREP,
        C0,
        C112,
        C112_1,
        C112_2,
        C113,
        C113_1,
        C123,
        C123_1,
        C123_2,
        C2,
        C2_1
    }

    private boolean C0 = false;
    private boolean C112 = false;
    private boolean C113 = false;
    private boolean C123 = false;
    private boolean C2 = false;
    private boolean intake = false;
    private StateMachine cacher;
//    private boolean cache = false;
//
//    Queue<Ball> queue;
//    Deque<Ball> cached;
//    Indexer indexer;
//
//    private DigitalChannel upIndex;
//    private DigitalChannel downIndex;
//    private DigitalChannel middleIndex;
//    private RevColorSensorV3 color;
    private ElapsedTime time;
    private Robot robot;

    public Pattern(Robot robot) {
        this.robot = robot;
//        this.queue = queue;
//        this.indexer = indexer;
//        upIndex = hw.get(DigitalChannel.class, "upIndex");
//        downIndex = hw.get(DigitalChannel.class, "downIndex");
//        middleIndex = hw.get(DigitalChannel.class, "middleIndex");
//        color = hw.get(RevColorSensorV3.class, "color");
//        cached = new ArrayDeque<>();
        time = new ElapsedTime();

        cacher = new StateMachineBuilder()
                .state(State.IDLE)
                .onExit(() -> {
                    robot.indexer.blockerClosed();
                })
                .transition(() -> intake)
                .state(State.IDLE2)
                .transitionTimed(0.5)
                .state(State.PREP)
                .onEnter(() -> robot.indexer.blockerClosed())
                .transition(() -> C0, State.C0)
                .transition(() -> C112, State.C112)
                .transition(() -> C113, State.C113)
                .transition(() -> C123, State.C123)
                .transition(() -> C2, State.C2)
                .state(State.C0)
                .onEnter(() -> robot.intake.forward())
                .onExit(() -> {
                    robot.intake.hold();
                    C0 = false;
                    intake = false;
                })
                .transitionTimed(Tunables.sortTime*3, State.IDLE)
                .state(State.C112)
                .onEnter(() -> {
                    robot.intake.forward();
                    robot.indexer.kickerUp();
                    robot.indexer.blockerOpen();
                    robot.transfer.reverse();
                })
                .transitionTimed(Tunables.sortTime*1)
                .state(State.C112_1)
                .onEnter(() -> {
                    robot.indexer.kickerDown();
                    robot.indexer.blockerClosed();
                    robot.transfer.forward();
                })
                .onExit(()-> {
//                    robot.indexer.kickerChute();
                    robot.indexer.blockerUp();
                    robot.intake.off();
                    robot.transfer.forward();
                })
                .transitionTimed(Tunables.sortTime*1)
                .state(State.C112_2)
                .onEnter(() -> robot.intake.forward())
                .onExit(() -> {
                    robot.intake.off();
                    C112 = false;
                    intake = false;
                })
                .transitionTimed(Tunables.sortTime*1, State.IDLE)
                .state(State.C113)
                .onEnter(() -> {
                    robot.intake.forward();
                    robot.indexer.kickerUp();
                    robot.indexer.blockerOpen();
                    robot.transfer.reverse();
                })
                .transitionTimed(Tunables.sortTime*1)
                .state(State.C113_1)
                .onEnter(() -> {
                    robot.indexer.kickerDown();
                    robot.indexer.blockerClosed();
                    robot.transfer.forward();
                })
                .onExit(() -> {
                    robot.indexer.blockerUp();
                    robot.intake.off();
                    C113 = false;
                    intake = false;
                })
                .transitionTimed(Tunables.sortTime*2, State.IDLE)
                .state(State.C123)
                .onEnter(() -> {
                    robot.intake.forward();
                    robot.indexer.kickerDown();
                    robot.indexer.blockerClosed();
                    robot.transfer.forward();
                })
                .transitionTimed(Tunables.sortTime*1)
                .state(State.C123_1)
                .onEnter(() -> {
                    robot.indexer.kickerUp();
                    robot.indexer.blockerOpen();
                    robot.transfer.reverse();
                })
                .transitionTimed(Tunables.sortTime*1)
                .state(State.C123_2)
                .onEnter(() -> {
                    robot.indexer.kickerDown();
                    robot.indexer.blockerClosed();
                    robot.transfer.off();
                })
                .onExit(() -> {
                    robot.indexer.blockerUp();
                    robot.transfer.forward();
                    robot.intake.off();
                    C123 = false;
                    intake = false;
                })
                .transitionTimed(Tunables.sortTime*1, State.IDLE)
                .state(State.C2)
                .onEnter(() -> {
                    robot.indexer.kickerUp();
                    robot.indexer.blockerOpen();
                    robot.transfer.reverse();
                    robot.intake.forward();
                })
                .transitionTimed(Tunables.sortTime*2)
                .state(State.C2_1)
                .onEnter(() -> {
                    robot.indexer.kickerDown();
                    robot.indexer.blockerClosed();
                    robot.transfer.off();
                })
                .onExit(() -> {
                    robot.indexer.blockerUp();
                    robot.intake.off();
                    robot.transfer.forward();
                    C2 = false;
                    intake = false;
                })
                .transitionTimed(Tunables.sortTime*1, State.IDLE)
                .build();

        cacher.start();
    }

    public void update() {
        cacher.update();
    }

    public String getState() {
        return cacher.getState().toString();
    }

    public void inputString(String code) {
        switch(code) {
            case "C0":
                C0=true;
                intake=true;
                break;
            case "C112":
                C112=true;
                intake=true;
                break;
            case "C113":
                C113=true;
                intake=true;
                break;
            case "C123":
                C123=true;
                intake=true;
                break;
            case "C2":
                C2=true;
                intake=true;
                break;
            default:
                break;
        }
    }

    public void inputTriplets(Vision.Triplet inputTriplet, Vision.Triplet outputTriplet) {
        if (outputTriplet == Vision.Triplet.UNKNOWN) {
            inputString("C0"); // maximize speed
        }
        switch (inputTriplet) {
            case GPP:
                switch (outputTriplet) {
                    case GPP:
                        inputString("C0");
                        break;
                    case PGP:
                        inputString("C112");
                        break;
                    case PPG:
                        inputString("C113");
                        break;
                }
                break;
            case PGP:
                switch (outputTriplet) {
                    case GPP:
                        inputString("C112");
                        break;
                    case PGP:
                        inputString("C0");
                        break;
                    case PPG:
                        inputString("C123");
                        break;
                }
                break;
            case PPG:
                switch (outputTriplet) {
                    case GPP:
                        inputString("C2");
                        break;
                    case PGP:
                        inputString("C113");
                        break;
                    case PPG:
                        inputString("C0");
                        break;
                }
                break;
        }
    }

//    public void update() {
//        Ball target = queue.poll();
//        switch (status) {
//            case IDLE:
//                cache=false;
//                Ball cd = cached.poll();
//                if (cd!=null && cd==target) {
//                    status = State.SORT;
//                    break;
//                }
//                indexer.blockerClosed();
//                indexer.kickerUp();
//                if (middleIndex()) {
//                    status = State.READ;
//                    Tunables.ballCount++;
//                }
//                break;
//            case READ:
//                Ball current = color();
//                if (target==null || target==current) {
//                    cache = false;
//                    queue.remove();
//                } else {
//                    cache = true;
//                    cached.addFirst(current);
//                }
//                status = State.SORT;
//                break;
//            case SORT:
//                if (cache) {
//                    indexer.blockerOpen();
//                } else {
//                    indexer.kickerDown();
//                }
//                status=State.WAIT;
//                time.reset();
//                break;
//            case WAIT:
//                if (time.milliseconds()>Tunables.indexWait) {
//                    status= State.IDLE;
//                }
//                break;
//        }
//    }
//
//    public void setPattern(Queue<Ball> queue) {
//        this.queue=queue;
//    }
//
//    public boolean upIndex() {
//        return upIndex.getState();
//    }
//
//    public boolean downIndex() {
//        return downIndex.getState();
//    }
//
//    public boolean middleIndex() {
//        return middleIndex.getState();
//    }
//
//    public Ball color() {
//        int p = color.red();
//        if (p > Tunables.purpleBot && p < Tunables.purpleTop) {
//            return Ball.PURPLE;
//        } else {
//            return Ball.GREEN;
//        }
//    }
}
