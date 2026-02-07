/** starting by the blue goal **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueICQCGoalAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueGoalAuto extends BlueAuto {
    @Override
    public Pose getStartPose() {
        return new Pose(32, 132.4, Math.toRadians(90)); // Start Pose of our robot.
    }
}
