/** base blue auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto { // these positions override the base auto class
    public static Pose goalPose = new Pose (0, 144);
    public static Pose scorePose = new Pose(50.60314364696112, 103.4801573265256, Math.toRadians(90));
    public static Pose scoreIntermediatePose = new Pose (58, 80, Math.toRadians(90));
    public static Pose pickup1StartPose = new Pose(58, 94, Math.toRadians(180));
    public static Pose pickup1EndPose = new Pose(26, 94, Math.toRadians(180));
    public static Pose pickup2StartPose = new Pose(58, 69, Math.toRadians(180));
    public static Pose pickup2EndPose = new Pose(20, 69, Math.toRadians(180));
    public static Pose pickup3StartPose = new Pose(58, 47, Math.toRadians(180));
    public static Pose pickup3EndPose = new Pose(20, 47, Math.toRadians(180));
    public static Pose releasePose = new Pose(16, 78, Math.toRadians(90));
    public static Pose endPose = new Pose(40, 65, Math.toRadians(90));
    public static double scoreTurretPos = 0.988514559173916;

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
