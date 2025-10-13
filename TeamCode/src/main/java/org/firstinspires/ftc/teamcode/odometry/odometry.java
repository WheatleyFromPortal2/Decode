package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class odometry implements Runnable {
    DcMotor verticalLeft, verticalRight, horizontal;
    double countsPerInch;
    boolean isRunning = true;

    double x, y;

    public odometry(DcMotor verticalLeft, DcMotor verticalRight, DcMotor horizontal, double countsPerInch, int threadSleepDelay) {
        this.verticalLeft = verticalLeft;
        this.verticalRight = verticalRight;
        this.horizontal = horizontal;
        this.countsPerInch = countsPerInch;
    }

    @Override
    public void run() {
        while (isRunning) {
            x = (verticalLeft.getCurrentPosition() + verticalRight.getCurrentPosition()) / 2.0;
            y = horizontal.getCurrentPosition();
            try { Thread.sleep(75); } catch (InterruptedException e) { e.printStackTrace(); }
        }
    }

    public void stop() { isRunning = false; }

    public double returnXCoordinate() { return x; }

    public double returnYCoordinate() { return y; }

    public void reverseLeftEncoder() {}

    public void reverseRightEncoder() {}
}
