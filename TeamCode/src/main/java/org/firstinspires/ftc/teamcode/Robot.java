/*
this is basically our mega-class that holds all robot data that is shared between auto and teleop
 */
package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class Robot { // create our global class for our robot
    public DcMotorEx intake, launch; // drive motors are handled by Pedro Pathing
    public Servo lowerTransfer, upperTransfer;



    public static final int TICKS_PER_REV = 28; // REV Robotics 5203/4 series motors have 28ticks/revolution
    public static final double launchRatio = (double) 0.8; // this is correct because 5202-0002-0001's gearbox ratio is 1:1, and we go from a 16tooth -> 20tooth pulley

    // PIDF coefficients
    public static final double launchP = 300; // orig 2.5
    public static final double launchI = 0.01; // orig 0.1
    public static final double launchD = 0.01; // orig 0.2
    public static final double launchF = (double) 1 / 2340; // 6000 rpm motor
    public static final double lowerTransferLowerLimit = 0.28;
    public static final double lowerTransferUpperLimit = 0.49;

    public static final double upperTransferClosed = 0.36; // servo position where upper transfer prevents balls from passing into launch
    public static final double upperTransferOpen = 0.66; // servo position where upper transfer allows balls to pass into launch

    public static final int launchDelay = 250; // time to wait for servos to move during launch (in ms)
    public final double scoreVelocityMargin = 100; // margin of 100tps TODO: tune this

    public Robot(HardwareMap hw) { // create all of our hardware
        // DC motors (all are DcMotorEx for current monitoring)
        intake = hw.get(DcMotorEx.class, "intake");
        launch = hw.get(DcMotorEx.class, "launch");

        // servos
        lowerTransfer = hw.get(Servo.class, "lowerTransfer");
        upperTransfer = hw.get(Servo.class, "upperTransfer");

        // sensors

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launch.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder
        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        PIDFCoefficients pidfOrig = launch.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(launchP, launchI, launchD, launchF);
        launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        //telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
        //        pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
    }
    /*
    public double[] getXYFromGoal(double goalX, double goalY) { // TODO: fix this to use pedro pathing odo
        double botX = odo.getPosX(DistanceUnit.MM);
        double botY = odo.getPosY(DistanceUnit.MM);
        double xDst = goalX - botX;
        double yDst = goalY - botY;
        double[] dists = {xDst, yDst};
        return dists;
    }

    public double getDstFromGoal(double goalX, double goalY) {
        double xDst = getXYFromGoal(goalX, goalY)[0];
        double yDst = getXYFromGoal(goalX, goalY)[1];
        return Math.pow(Math.pow(xDst, 2) + Math.pow(yDst, 2), 0.5); // use pythag to find dst from goal
    }

    public double getGoalHeading(double goalX, double goalY) { // return bot heading to point towards goal in radians
        double xDst = getXYFromGoal(goalX, goalY)[0];
        double yDst = getXYFromGoal(goalX, goalY)[1];
        return Math.atan(xDst / yDst);
    }

    public double getHeading() { // return current bot heading in radians
        // TODO: fix this
        //return odo.getHeading(AngleUnit.RADIANS);
        return 1;
    }
    public double getNeededVelocity(double goalX, double goalY) { // returns dst from goal in MM
        double d = getDstFromGoal(goalX, goalY);
        double numerator = 19.62 * Math.pow(d, 2);
        double denominator = (Math.pow(3, 0.5) * d) - 0.8;
        return Math.pow(numerator / denominator, 0.5); // thank u rahul
    }
    public void moveMM(double x, double y) { // move x and y

    }
    public double getLaunchHeading(double goalX, double goalY) { // TODO: fix this
        double botX = odo.getPosX(DistanceUnit.MM);
        double botY = odo.getPosY(DistanceUnit.MM);
        double xDst = goalX - botX;
        double yDst = goalY - botY;
        return Math.pow(Math.pow(xDst, 2) + Math.pow(yDst, 2), 0.5); // use pythag to find dst from goal
    }

    public double angleDifference(double target) { // calculate the shortest difference between 2 angles in radians
        double currentHeading = getHeading();
        double diff = target - currentHeading;
        // makes sure we always turn the shortest amount to a heading (don't go all the way around)
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    public double setLaunchTangentialSpeed(double tangentialSpeed) { // input tangentialSpeed (in m/s) and set launch velocity to have ball shoot at that speed
        double TPS = 0;
        // this will be the hardest function to code
        // it basically needs to be a relation between the rotational speed of launch and the actual output speed of the ball
        // at the end, we will output the desired TPS of our motor to monitor once it reaches it
        return TPS;
    }
    */
    public double getLaunchRPM() { // return launch velocity in RPM
        return (launch.getVelocity() / Robot.TICKS_PER_REV ) * 60 * launchRatio;
    }
    public double getLaunchRadians() { // return launch velocity in radians/second
        return ((launch.getVelocity() / Robot.TICKS_PER_REV) * 2 * Math.PI);
    }
    public double getLaunchCurrent() { // return launch current in amps
        return launch.getCurrent(CurrentUnit.AMPS);
    }
    public boolean isLaunchWithinMargin(double desiredScoreVelocity) {
        return Math.abs(desiredScoreVelocity - launch.getVelocity()) < scoreVelocityMargin; // measure if our velocity is within our margin of error
    }
    public double getIntakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
    public void initServos() { // set servos to starting state
        upperTransfer.setPosition(Robot.upperTransferClosed); // make sure balls cannot launch
        lowerTransfer.setPosition(Robot.lowerTransferLowerLimit); // make sure lower transfer is not getting in the way
    }
    public void launchBall() throws InterruptedException { // launch a ball
        // TODO: make this asynchronous (eliminate all the waits)
        upperTransfer.setPosition(upperTransferOpen);
        sleep(launchDelay); // allow time for upper transfer to move
        lowerTransfer.setPosition(lowerTransferUpperLimit);
        sleep(launchDelay); // allow time for lower transfer to move
        // hopefully the ball has launched by now
        upperTransfer.setPosition(upperTransferClosed); // close upper transfer
        lowerTransfer.setPosition(lowerTransferLowerLimit); // set lower transfer to its lowest
    }
}