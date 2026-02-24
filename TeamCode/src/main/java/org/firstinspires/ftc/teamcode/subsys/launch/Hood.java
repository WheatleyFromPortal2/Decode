package org.firstinspires.ftc.teamcode.subsys.launch;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunables;

public class Hood {
    private Servo servo;

    // last position we set the servo to (not measured from minimum)
    private double lastPos;

    public Hood(HardwareMap hw) {
        servo = hw.get(Servo.class, "hoodServo");

        servo.setPosition(Tunables.hoodMinimumPos);
        lastPos = Tunables.hoodMinimumPos; // force initial write
    }

    /** internal methods **/
    private void setPositionFast(double newPos) {
        // clamp within set hardware limits
        newPos = Range.clip(newPos, Tunables.hoodMinimumPos, Tunables.hoodMaximumPos);

        if (Math.abs(newPos - lastPos) > Tunables.hoodWriteMargin) { // don't write excessively
            servo.setPosition(newPos);
            lastPos = newPos;
        }
    }

    /** getter methods **/

    public double getPos() { // return position as measured from minimum
        return lastPos - Tunables.hoodMinimumPos;
    }

    public double getRadians() {
        return posToRadians(lastPos);
    }

    public double getAbsolutePos() {
        return lastPos;
    }

    /** setter methods **/

    public void setPos(double newPos) {
        setPositionFast(newPos + Tunables.hoodMinimumPos);
        // set position relative to minimum
    }

    public void setRawPos(double pos) {
        // !only use this for testing!
        double clampedPos = Range.clip(pos, 0.0, 1.0);
        servo.setPosition(clampedPos);
        lastPos = clampedPos;
    }

    public void setRadians(double radians) {
        setPositionFast(radiansToPos(radians));
    }

    private double radiansToPos(double radians) {
        return Range.scale(radians, Tunables.hoodMinimumRadians, Tunables.hoodMaximumRadians, Tunables.hoodMaximumPos, Tunables.hoodMinimumPos); // reverse to correct
    }

    private double posToRadians(double pos) {
        return Range.scale(pos, Tunables.hoodMinimumPos, Tunables.hoodMaximumPos, Tunables.hoodMaximumRadians, Tunables.hoodMinimumRadians); // reverse to correct
    }
}
