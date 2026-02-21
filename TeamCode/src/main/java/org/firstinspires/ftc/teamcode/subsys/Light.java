/** control GoBilda RGB indicator light using servo interface **/

package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Light {
    // from: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    double OFF = 0.0;
    double RED = 0.279;
    double GREEN = 0.5;

    private double lastColor = -1; // force hardware write first try

    Servo light;

    public Light(HardwareMap hw) {
        light = hw.get(Servo.class, "light");
        off(); // start with light off
    }

    public void off() {
        setColorFast(OFF);
    }

    public void red() {
        setColorFast(RED);
    }

    public void green() {
        setColorFast(GREEN);
    }

    public void setColorFast(double color) {
        color = Range.clip(color, 0.0, 1.0);
        if (color != lastColor) { // only write to hardware if necessary
            light.setPosition(color);
            lastColor = color;
        }
    }
}
