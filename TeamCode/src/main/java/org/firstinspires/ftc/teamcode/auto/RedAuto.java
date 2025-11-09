// base red auto
package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

public abstract class RedAuto extends BozoAuto { // these positions override the base auto class
    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                new Pose(84, 84, Math.toRadians(44)), // scorePose
                new Pose(102, 84, Math.toRadians(0)), // pickup1StartPose
                new Pose(128, 84, Math.toRadians(0)), // pickup1EndPose
                new Pose(102, 60, Math.toRadians(0)), // pickup2StartPose
                new Pose(128, 60, Math.toRadians(0)), // pickup2EndPose
                new Pose(102, 36, Math.toRadians(0)), // pickup3StartPose
                new Pose(128, 36, Math.toRadians(0)), // pickup3EndPose
                new Pose(52, 36, Math.toRadians(270)) // endPose
        );
    }
}
