// this class is just used to provide auto parameters
package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

public class AutoConfig {
    public Pose goalPose;
    public Pose scorePose;
    public Pose scoreIntermediatePose;
    public Pose pickup1StartPose;
    public Pose pickup1EndPose;
    public Pose pickup2StartPose;
    public Pose pickup2EndPose;
    public Pose pickup3StartPose;
    public Pose pickup3EndPose;
    public Pose pickup4StartPose;
    public Pose pickup4EndPose;
    public Pose endPose;

    public AutoConfig(Pose goalPose,
                      Pose scorePose,
                      Pose scoreIntermediatePose,
                      Pose pickup1StartPose,
                      Pose pickup1EndPose,
                      Pose pickup2StartPose,
                      Pose pickup2EndPose,
                      Pose pickup3StartPose,
                      Pose pickup3EndPose,
                      Pose pickup4StartPose,
                      Pose pickup4EndPose,
                      Pose endPose) {
        this.goalPose = goalPose;
        this.scorePose = scorePose;
        this.scoreIntermediatePose = scoreIntermediatePose;
        this.pickup1StartPose = pickup1StartPose;
        this.pickup1EndPose = pickup1EndPose;
        this.pickup2StartPose = pickup2StartPose;
        this.pickup2EndPose = pickup2EndPose;
        this.pickup3StartPose = pickup3StartPose;
        this.pickup3EndPose = pickup3EndPose;
        this.pickup4StartPose = pickup4StartPose;
        this.pickup4EndPose = pickup4EndPose;
        this.endPose = endPose;
    }
}
