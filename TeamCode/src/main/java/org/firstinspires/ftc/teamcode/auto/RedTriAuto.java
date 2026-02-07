/** starting by the triangle on the red side **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "!DONT USE!RedTriAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedTriAuto extends RedAuto{
    @Override
    public Pose getStartPose() {
        return new Pose(87.96652719665272, 8.28451882845189, Math.toRadians(90)); // Start Pose of our robot.
    }
}