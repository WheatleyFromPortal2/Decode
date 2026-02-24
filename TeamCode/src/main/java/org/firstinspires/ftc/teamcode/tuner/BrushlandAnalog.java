/** OpMode to configure Brushland Labs color rangefinder (docs: https://docs.brushlandlabs.com/sensors/color-rangefinder) **/
// taken from: https://docs.brushlandlabs.com/sensors/color-rangefinder/configuration

package org.firstinspires.ftc.teamcode.tuner;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;


@TeleOp(name="BrushlandAnalog", group="Tuner")
public class BrushlandAnalog extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color"));
        waitForStart();
        /* Using this example configuration, you can detect both artifact colors based on which pin is reading true:
            pin0 --> purple
            pin1 --> green */
        crf.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 20); // placeholder
    }
}