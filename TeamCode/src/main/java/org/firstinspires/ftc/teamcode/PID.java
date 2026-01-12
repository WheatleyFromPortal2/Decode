/** general PIDF controller to use for pointing robot towards goal april tag **/
// taken from: https://roboftc.github.io/code/pidf.html

package org.firstinspires.ftc.teamcode;

public class PID {
    // PIDF coefficients
    public double kP, kI, kD;

    // State variables
    private double sumError = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void set(double kP, double kI, double kD) { // update coefficients
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset() {
        sumError = 0;
        lastError = 0;
        lastTime = 0;
    }

    public double calc(double error) {
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = (lastTime == 0) ? 0 : (currentTime - lastTime);

        // Proportional
        double P = kP * error;

        // Integral
        if (deltaTime > 0) {
            sumError += error * deltaTime;
        }
        double I = kI * sumError;

        // Derivative
        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        double D = kD * derivative;

        // Store for next loop
        lastError = error;
        lastTime = currentTime;

        // Total output
        return P + I + D;
    }
}
