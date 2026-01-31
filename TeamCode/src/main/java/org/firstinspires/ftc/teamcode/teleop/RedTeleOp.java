/** this is our teleop OpMode for red team **/

package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RedTeleOp", group="TeleOp")
public class RedTeleOp extends BozoTeleOp {
    @Override // how our x-stick input should be modified to reflect the driver's position
    public boolean isBlueTeam() { return false; } // flip the control

    @Override
    public Pose getGoalPose() { // return our goal pose
        // because the buildConfig() function is protected, we cannot call it here - possibly replace separate variables
        return new Pose (144, 144); // taken from RedAuto.java
    }

}
