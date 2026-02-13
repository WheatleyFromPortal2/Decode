package org.firstinspires.ftc.teamcode.subsys.launch;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunables;

public class Turret {
    private ServoImplEx turret1, turret2;
    private DcMotorEx encoder;

    private static final int TURRET_TICKS_PER_REV = 1024; // tested 1-13-26
    public static final double TURRET_ENCODER_RATIO = 5.5; // ratio from turretEncoder->turret
    //public static final double TURRET_SERVO_RATIO = 5.5 / 3; // ratio from turret1/2->turret

    /** vars that change **/
    private boolean isPowered = false;
    private double lastDesiredPos = 0;
    private double desiredPos = 0;

    /** end vars that change **/

    public Turret(HardwareMap hw) {
        turret1 = hw.get(ServoImplEx.class, "turret1");
        turret2 = hw.get(ServoImplEx.class, "turret2");
        //turret1.setPwmEnable();
        //turret2.setPwmEnable();

        encoder = hw.get(DcMotorEx.class, "turretEncoder");

        resetEncoder();
    }

    public void update() {
        if (isPowered) {
            //double newTurretServoPos = ((-desiredPos + Tunables.turretOffset) / (Tunables.turretMax * 2)) + 0.5;

            double turretAngle = Tunables.turretCenterOffset - desiredPos;
            double preClamped = Range.clip(turretAngle, -Tunables.turretMaxRight, Tunables.turretMaxLeft);
            double maxAdjusted = (preClamped + Tunables.turretMaxRight) / (Tunables.turretMaxLeft + Tunables.turretMaxRight);

            if (Double.isNaN(maxAdjusted)) {
                throw new IllegalStateException("turret position is NaN!");
            }

            double newTurretServoPos = Range.clip(maxAdjusted, 0.0, 1.0); // make sure we don't cause an exception

            if (newTurretServoPos == lastDesiredPos) {
                // do nothing, save loop time
            } else {
                turret1.setPosition(newTurretServoPos);
                turret2.setPosition(newTurretServoPos);
                lastDesiredPos = desiredPos;
            }
        }
    }

    /** internal methods **/

    private double ticksToRadians(double ticks) {
        double encoderRevs = -ticks / TURRET_TICKS_PER_REV; // negate ticks because positive is ccw for radians 🤦
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
        return ticksToRadians(encoder.getCurrentPosition());
        //return -(turret1.getPosition() - 0.5) * 2 * TURRET_SERVO_MAX_RANGE;
    }

    public double getError() { // get difference between servo set position and measured encoder position
        return getPos() - desiredPos;
    }

    /** setter methods **/

    public void setDesiredPos(double radians) { // set desired turret angle
        if (!isPowered) on();
        desiredPos = normalizeRadians(radians);
    }

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

    public void lock() {
        desiredPos = 0;
        on();
    }

    public void zero() { // fully reset everything
        lastDesiredPos = 0;
        isPowered = true;
        resetEncoder();
        off();
    }

    public void setRawServoPositions(double pos) { // !only use this for calibration!
        turret1.setPosition(pos);
        turret2.setPosition(pos);
    }
}
