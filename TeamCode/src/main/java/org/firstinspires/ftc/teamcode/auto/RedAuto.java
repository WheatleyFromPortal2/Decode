/** base red auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

public abstract class RedAuto extends BozoAuto { // these positions override the base auto class
    @Override
    protected AutoConfig buildConfig() {
        // 11-20-25: this is just BlueAuto.java but reflected over x = 72
        return new AutoConfig(
                new Pose (132, 138), // goal position (we don't need a heading)
                new Pose(94, 103.4801573265256, Math.toRadians(44)), // scorePose
                new Pose(90, 80, Math.toRadians(44)), // scoreIntermediatePose
                new Pose(86, 94, Math.toRadians(0)), // pickup1StartPose
                new Pose(126, 94, Math.toRadians(0)), // pickup1EndPose
                new Pose(86, 69, Math.toRadians(0)), // pickup2StartPose
                new Pose(126, 69, Math.toRadians(0)), // pickup2EndPose
                new Pose(86, 47, Math.toRadians(0)), // pickup3StartPose
                new Pose(126, 47, Math.toRadians(0)), // pickup3EndPose
                new Pose(118, 80, Math.toRadians(90)), // releasePose
                new Pose(104, 65, Math.toRadians(270)) // endPose
        );
    }
}
