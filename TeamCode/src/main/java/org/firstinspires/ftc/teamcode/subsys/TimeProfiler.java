/** simple and flexible time profiler that outputs a string **/

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.util.Timer;

public class TimeProfiler {
    Timer segmentTimer;
    String currentSegment;
    String output;
    boolean isTiming;

    public TimeProfiler() {
        segmentTimer = new Timer();
        currentSegment = "";
        output = "";
        isTiming = false;
    }

    public void start(String segment) { // start timing, and if we are already timing, stop that
        if (isTiming) {
            stop();
        }
        currentSegment = segment;
        segmentTimer.resetTimer();
        isTiming = true;
    }

    public void stop() { // stop current time segment and save it
        output = output + currentSegment + ": " + segmentTimer.getElapsedTime() + "ms\n";
        isTiming = false;
    }

    public String getOutputAndReset() { // get output of time segments and reset
        if (isTiming) stop();
        String result = output;
        output = "";
        return result;
    }
}
