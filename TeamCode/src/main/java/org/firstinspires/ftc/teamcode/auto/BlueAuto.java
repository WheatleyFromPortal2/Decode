/** base blue auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

public abstract class BlueAuto extends BozoAuto { // these positions override the base auto class
    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                new Pose (12, 138), // goal position (we don't need a heading)
                new Pose(50.60314364696112, 103.4801573265256, Math.toRadians(90)), // scorePose
                new Pose (58, 80, Math.toRadians(90)), // scoreIntermediatePose
                new Pose(58, 94, Math.toRadians(180)), // pickup1StartPose
                new Pose(26, 94, Math.toRadians(180)), // pickup1EndPose
                new Pose(58, 69, Math.toRadians(180)), // pickup2StartPose
                new Pose(20, 69, Math.toRadians(180)), // pickup2EndPose
                new Pose(58, 47, Math.toRadians(180)), // pickup3StartPose
                new Pose(20, 47, Math.toRadians(180)), // pickup3EndPose
                new Pose(24, 73, Math.toRadians(90)), // releasePose
                new Pose(40, 65, Math.toRadians(90)), // endPose (idk if we can end in red square)
                0.928514559173916 // scoreTurretPos
        );
    }
}
