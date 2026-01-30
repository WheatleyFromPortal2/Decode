/** starting by the red goal **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "RedGoalAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedGoalAuto extends RedAuto {
    public Pose getStartPose() {
        return new Pose(112, 132.4, Math.toRadians(90)); // Start Pose of our robot.
    }
}
