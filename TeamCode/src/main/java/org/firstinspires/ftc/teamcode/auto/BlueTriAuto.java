/** starting by the triangle on the blue side **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "!DONT USE!BlueTriAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueTriAuto extends BlueAuto{
    @Override
    public Pose getStartPose() {
        return new Pose(56.03347280334728, 8.43514644351465, Math.toRadians(90)); // Start Pose of our robot.
    }
}
