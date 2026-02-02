/** base red auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto { // these positions override the base auto class
    // 1-9-26: this is just BlueAuto.java but reflected over x = 72
    public static Pose goalPose = new Pose (144, 144);
    public static Pose scorePose = new Pose(94, 103.4801573265256, Math.toRadians(90));
    public static Pose scoreIntermediatePose = new Pose(86, 80, Math.toRadians(90));
    public static Pose pickup1StartPose = new Pose(86, 94, Math.toRadians(0));
    public static Pose pickup1EndPose = new Pose(118, 94, Math.toRadians(0));
    public static Pose pickup2StartPose = new Pose(86, 69, Math.toRadians(0));
    public static Pose pickup2EndPose = new Pose(124, 69, Math.toRadians(0));
    public static Pose pickup3StartPose = new Pose(86, 47, Math.toRadians(0));
    public static Pose pickup3EndPose = new Pose(124, 47, Math.toRadians(0));
    public static Pose releasePose = new Pose(120, 73, Math.toRadians(30));
    public static Pose endPose = new Pose(104, 65, Math.toRadians(270));
    public static double scoreTurretPos = -0.928514559173916;

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
