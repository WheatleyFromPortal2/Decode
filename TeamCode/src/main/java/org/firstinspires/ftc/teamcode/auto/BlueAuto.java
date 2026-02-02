/** base blue auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto { // these positions override the base auto class
    Pose goalPose = new Pose (0, 144);
    Pose scorePose = new Pose(50.60314364696112, 103.4801573265256, Math.toRadians(90));
    Pose scoreIntermediatePose = new Pose (58, 80, Math.toRadians(90));
    Pose pickup1StartPose = new Pose(58, 94, Math.toRadians(180));
    Pose pickup1EndPose = new Pose(26, 94, Math.toRadians(180));
    Pose pickup2StartPose = new Pose(58, 69, Math.toRadians(180));
    Pose pickup2EndPose = new Pose(20, 69, Math.toRadians(180));
    Pose pickup3StartPose = new Pose(58, 47, Math.toRadians(180));
    Pose pickup3EndPose = new Pose(20, 47, Math.toRadians(180));
    Pose releasePose = new Pose(20, 73, Math.toRadians(90));
    Pose endPose = new Pose(40, 65, Math.toRadians(90));
    double scoreTurretPos = 0.928514559173916;

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                goalPose,
                scorePose,
                scoreIntermediatePose,
                pickup1StartPose,
                pickup1EndPose,
                pickup2StartPose,
                pickup2EndPose,
                pickup3StartPose,
                pickup3EndPose,
                releasePose,
                endPose,
                scoreTurretPos
        );
    }
}
