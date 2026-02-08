/** base red auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto { // these positions override the base auto class
    public static Pose goalPose = new Pose(144, 144);
    public static Pose scorePose = new Pose(93.39685635303888, 103.4801573265256, Math.toRadians(90));
    public static Pose scoreIntermediatePose = new Pose(99, 80, Math.toRadians(90));
    public static Pose pickup1StartPose = new Pose(86, 94, Math.toRadians(0));
    public static Pose pickup1EndPose = new Pose(117, 94, Math.toRadians(0));
    public static Pose pickup2StartPose = new Pose(82, 69, Math.toRadians(0));
    public static Pose pickup2EndPose = new Pose(123, 68, Math.toRadians(0));
    public static Pose pickup3StartPose = new Pose(82, 47, Math.toRadians(0));
    public static Pose pickup3EndPose = new Pose(123, 47, Math.toRadians(0));
    public static Pose releasePose = new Pose(129, 74, Math.toRadians(90));
    public static Pose endPose = new Pose(104, 65, Math.toRadians(90));
    public static double scoreTurretPos = -0.91; // because our servos are misaligned, this must be diff from BlueAuto.java.
    // need to fix turret offset

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
