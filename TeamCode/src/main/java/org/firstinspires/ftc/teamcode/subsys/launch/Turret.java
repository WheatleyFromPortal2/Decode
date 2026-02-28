package org.firstinspires.ftc.teamcode.subsys.launch;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.PIDF;

public class Turret {
    private Servo turret1, turret2;
    private DcMotorEx encoder;

    private static final int TURRET_TICKS_PER_REV = 1024; // tested 1-13-26
    //public static final double TURRET_ENCODER_RATIO = 5.5; // ratio from turretEncoder->turret
    public static final double TURRET_ENCODER_RATIO = (double) 110 / 35;

    private PIDF pidf;
    /** vars that change **/
    private double lastDesiredPos = 0;
    private double desiredPos = 0;
    private double posOffset = 0;

    /** end vars that change **/

    public Turret(HardwareMap hw, DcMotorEx encoder) {
        turret1 = hw.get(Servo.class, "turret1");
        turret2 = hw.get(Servo.class, "turret2");

        this.encoder = encoder;

        pidf = new PIDF(Tunables.turretP, Tunables.turretI, Tunables.turretD, Tunables.turretF);

        resetEncoder();
    }

    public void update() {
        pidf.updateTerms(Tunables.turretP, Tunables.turretI, Tunables.turretD, Tunables.turretF);
        double curPos = getPos();
        double extraPower = 0;

        if (Math.abs(curPos - desiredPos) < Tunables.turretPIDFMargin) {
            extraPower = pidf.calc(desiredPos, getPos());
        }

        double newTurretServoPos = Range.scale(desiredPos, Tunables.turretMaxRight, Tunables.turretMaxLeft, Tunables.turretLimitRight, Tunables.turretLimitLeft);
        newTurretServoPos -= extraPower;
        newTurretServoPos = Range.clip(newTurretServoPos, Tunables.turretLimitLeft, Tunables.turretLimitRight);

        turret1.setPosition(newTurretServoPos);
        turret2.setPosition(newTurretServoPos);
        lastDesiredPos = desiredPos;
    }

    /** internal methods **/

    private double ticksToRadians(double ticks) {
        double encoderRevs = ticks / TURRET_TICKS_PER_REV; // don't negate ticks because encoder is mounted on the top now
        double turretRevs = encoderRevs / TURRET_ENCODER_RATIO;
        return turretRevs * 2 * Math.PI; // convert to radians
    }

    private void resetEncoder() {
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // reset our encoder (this only seems to work when run after the OpMode is started)
        encoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // we won't be using the motor at all
    }

    /** getter methods **/

    public double getVelocity() { // get turret speed in radians/s
        return ticksToRadians(encoder.getVelocity());
    }

    public double getDesiredPos() {
        return desiredPos;
    }

    public double getPos() { // return our current turret angle in radians +/- from facing forwards
        return ticksToRadians(encoder.getCurrentPosition() + posOffset);
        //return -(turret1.getPosition() - 0.5) * 2 * TURRET_SERVO_MAX_RANGE;
    }

    public void updateOffset() {
        posOffset = -HandoffState.turretPos;
    }

    public double getError() { // get difference between servo set position and measured encoder position
        return getPos() - desiredPos;
    }

    /** setter methods **/

    public void setDesiredPos(double radians) { // set desired turret angle
        //if (!isPowered) on();
        desiredPos = normalizeRadians(radians);
    }

    /*
    public void off() {
        turret1.setPwmDisable();
        turret2.setPwmDisable();
        isPowered = false;
    }

    public void on() {
        turret1.setPwmEnable();
        turret2.setPwmEnable();
        isPowered = true;
    }
    */

    public void lock() {
        desiredPos = 0;
        //on();
    }

    public void zero() { // fully reset everything
        lastDesiredPos = 0;
        resetEncoder();
        //off();
    }

    public void setRawServoPositions(double pos) { // !only use this for calibration!
        turret1.setPosition(pos);
        turret2.setPosition(pos);
    }
}
