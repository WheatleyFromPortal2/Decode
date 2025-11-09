// base blue auto
package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

public abstract class BlueAuto extends BozoAuto { // these positions override the base auto class
    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                new Pose (12, 138), // goal position (we don't need a heading)
                new Pose(60, 84, Math.toRadians(128)), // scorePose
                new Pose(42, 84, Math.toRadians(180)), // pickup1StartPose
                new Pose(16, 84, Math.toRadians(180)), // pickup1EndPose
                new Pose(42, 60, Math.toRadians(180)), // pickup2StartPose
                new Pose(16, 60, Math.toRadians(180)), // pickup2EndPose
                new Pose(42, 36, Math.toRadians(180)), // pickup3StartPose
                new Pose(16, 36, Math.toRadians(180)), // pickup3EndPose
                new Pose(52, 36, Math.toRadians(90)) // endPose
        );
    }
}
