/** base blue auto **/
package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

public abstract class BlueAuto extends BozoAuto { // these positions override the base auto class
    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                new Pose (12, 138), // goal position (we don't need a heading)
                new Pose(50.60314364696112, 103.4801573265256, Math.toRadians(134.56903472936997)), // scorePose (try a closer one now) heading: 2.331219434738159
                new Pose (50, 80, Math.toRadians(134.56903472936997)), // scoreIntermediatePose
                new Pose(58, 94, Math.toRadians(180)), // pickup1StartPose
                new Pose(26, 94, Math.toRadians(180)), // pickup1EndPose
                new Pose(58, 70, Math.toRadians(180)), // pickup2StartPose
                new Pose(16, 80, Math.toRadians(180)), // pickup2EndPose
                new Pose(58, 47, Math.toRadians(180)), // pickup3StartPose
                new Pose(20, 47, Math.toRadians(180)), // pickup3EndPose
                new Pose(12, 40, Math.toRadians(270)), // pickup4StartPose
                new Pose(12, 25, Math.toRadians(270)), // pickup4EndPose
                new Pose(16, 68, Math.toRadians(80)), // releasePose
                new Pose(40, 65, Math.toRadians(90)) // endPose (idk if we can end in red square)
        );
    }
}
