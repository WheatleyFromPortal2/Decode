/** control GoBilda RGB indicator light using servo interface **/

package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Light {
    // from: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    private final double OFF = 0.0;
    private final double RED = 0.279;
    private final double ORANGE = 0.333;
    private final double YELLOW = 0.388;
    private final double GREEN = 0.5;
    private final double BLUE = 0.611;
    private final double VIOLET = 0.721;
    private final double WHITE = 1.0;

    private Servo light;

    private double lastColor = -1; // force hardware write first try
    private double color = OFF;

    public Light(HardwareMap hw) {
        light = hw.get(Servo.class, "light");

        setColorFast(OFF); // start off
    }

    public void update() {
        color = Range.clip(color, 0.0, 1.0); // clip color to servo limits
        if (color != lastColor) { // only write to hardware if necessary
            light.setPosition(color);
            lastColor = color;
        }
    }

    public void off() { color = OFF; }

    /** color methods **/
    public void red() { color = RED; }
    public void orange() { color = ORANGE; }
    public void yellow() { color = YELLOW; }
    public void green() { color = GREEN; }
    public void blue() { color = BLUE; }
    public void violet() { color = VIOLET; }
    public void white() { color = WHITE; }

    public void setColorFast(double color) {
    }
}
