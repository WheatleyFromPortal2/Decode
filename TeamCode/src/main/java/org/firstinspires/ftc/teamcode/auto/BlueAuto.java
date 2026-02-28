/** base blue auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto { // these positions override the base auto class
    public static Pose goalPose = new Pose (0, 144);
    public static Pose scorePose = new Pose(50.60314364696112, 103.4801573265256, Math.toRadians(180) + 0.948514559173916);
    public static Pose scoreIntermediatePose = new Pose (45, 80, Math.toRadians(90));
    public static Pose pickup1StartPose = new Pose(40, 82, Math.toRadians(180));
    public static Pose pickup1EndPose = new Pose(18, 82, Math.toRadians(180));
    public static Pose pickup2StartPose = new Pose(40, 60, Math.toRadians(180));
    public static Pose pickup2EndPose = new Pose(6, 60, Math.toRadians(180));
    public static Pose pickup3StartPose = new Pose(40, 35, Math.toRadians(180));
    public static Pose pickup3EndPose = new Pose(6, 35, Math.toRadians(180));
    public static Pose releasePose = new Pose(15, 74, Math.toRadians(90));
    public static Pose endPose = new Pose(40, 65, Math.toRadians(90));
    public static double scoreTurretPos = -1.5;

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
