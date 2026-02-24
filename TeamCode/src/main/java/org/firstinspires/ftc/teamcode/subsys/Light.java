/** control GoBilda RGB indicator light using servo interface **/

package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Light {
    // from: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    double OFF = 0.0;
    double RED = 0.279;
    double ORANGE = 0.333;
    double YELLOW = 0.388;
    double GREEN = 0.5;
    double BLUE = 0.611;
    double VIOLET = 0.721;
    double WHITE = 1.0;

    private double lastColor = -1; // force hardware write first try

    private Servo light;

    public Light(HardwareMap hw) {
        light = hw.get(Servo.class, "light");

        off(); // start with light off
    }

    public void off() { setColorFast(OFF); }

    /** color methods **/
    public void red() { setColorFast(RED); }
    public void orange() { setColorFast(ORANGE); }
    public void yellow() { setColorFast(YELLOW); }
    public void green() { setColorFast(GREEN); }
    public void blue() { setColorFast(BLUE); }
    public void violet() { setColorFast(VIOLET); }
    public void white() { setColorFast(WHITE); }

    public void setColorFast(double color) {
        color = Range.clip(color, 0.0, 1.0);
        if (color != lastColor) { // only write to hardware if necessary
            light.setPosition(color);
            lastColor = color;
        }
    }
}
