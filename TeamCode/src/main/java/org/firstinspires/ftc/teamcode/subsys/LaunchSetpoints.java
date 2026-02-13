package org.firstinspires.ftc.teamcode.subsys;

public class LaunchSetpoints {
    public static final int MOTOR_TICKS_PER_MOTOR_REV = 28; // encoder ticks per motor revolution
    public static final double LAUNCH_RATIO = 1; // motor->flywheel ratio (output rotations / motor rotations)

    private double TPS;
    private double hoodPos;
    private double turretPos;

    public LaunchSetpoints(double startingTPS, double startingHoodPos, double startingTurretPos ) {
        TPS = startingTPS;
        hoodPos = startingHoodPos;
        turretPos = startingTurretPos;
    }

    public LaunchSetpoints copy() {
        return new LaunchSetpoints(TPS, hoodPos, turretPos);
    }

    /** getter **/

    public double getHoodPos() {
        return hoodPos;
    }

    public double getTurretPos() {
        return turretPos;
    }

    // flywheel velocity

    public double getTPS() {
        return TPS;
    }

    public double getRPM() {
        return TPSToRPM(TPS);
    }

    public double getRPS() { // get radian/s
        return TPSToRPS(TPS);
    }

    /** setter **/

    public void setHoodPos(double newHoodPos) {
        hoodPos = newHoodPos;
    }

    public void setTurretPos(double newTurretPos) {
        turretPos = newTurretPos;
    }

    // flywheel velocity

    public void setTPS(double newTPS) {
        TPS = newTPS;
    }

    public void setRPM(double newRPM) {
        TPS = RPMToTPS(newRPM);
    }

    public void incrementRPM(double addRPM) {
        TPS += RPMToTPS(addRPM);
    }

    public void decrementRPM(double subtractRPM) {
        TPS -= RPMToTPS(subtractRPM);
    }

    public void setRPS(double newRPS) {
        TPS = RPSToTPS(newRPS);
    }

    /** conversion **/

    private static double getMotorTicksPerOutputRev() {
        return MOTOR_TICKS_PER_MOTOR_REV / LAUNCH_RATIO;
    }

    private double TPSToRPM(double TPS) {
        double ticksPerOutputRev = getMotorTicksPerOutputRev();
        return (TPS / ticksPerOutputRev) * 60; // output RPM
    }

    private double RPMToTPS(double outputRPM) {
        double ticksPerOutputRev = getMotorTicksPerOutputRev();
        return (outputRPM / 60) * ticksPerOutputRev; // motor TPS
   }

    private double TPSToRPS(double TPS) {
        double ticksPerOutputRev = getMotorTicksPerOutputRev();
        return (TPS / ticksPerOutputRev) * 2 * Math.PI; // output RPS
    }

    private double RPSToTPS(double RPS) {
        double ticksPerOutputRev = getMotorTicksPerOutputRev();
        return (RPS / (2 * Math.PI)) * ticksPerOutputRev; // motor TPS
    }
}
