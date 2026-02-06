package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    public static PIDFCoefficients translationalPIDF = new PIDFCoefficients(0.1, 0, 0.008, 0.11);
    public static FilteredPIDFCoefficients drivePIDF = new FilteredPIDFCoefficients(0.02, 0, 0.004, 0.6, 0.06);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.29) // robot mass in kg
            // calibrated 2-5-26
            .forwardZeroPowerAcceleration(-34.582)
            .lateralZeroPowerAcceleration(-65)
            .translationalPIDFCoefficients(translationalPIDF)
            .drivePIDFCoefficients(drivePIDF);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("frontLeft")
            .rightFrontMotorName("frontRight")
            .leftRearMotorName("backLeft")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true) // use brake mode
            // calibrated 2-5-26
            .xVelocity(77.597) // use forward velocity tuner
            .yVelocity(60.498); // lateral velocity tuner (it goes to the left!)

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(158) // this is in mm
            .strafePodX(135) // this is in mm
            .distanceUnit(DistanceUnit.MM) // let's keep this in metric 👍
            .hardwareMapName("odo") // from README.md
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED); // flipped for turret bot

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
