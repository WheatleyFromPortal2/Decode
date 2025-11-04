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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
public class Robot { // create our global class for our robot
    public DcMotorEx frontLeft, frontRight, backLeft, backRight, intake, launch;
    public Servo lowerTransfer, upperTransfer;
    public com.qualcomm.hardware.gobilda.GoBildaPinpointDriver odo;



    public static final int TICKS_PER_REV = 28; // REV Robotics 5203/4 series motors have 28ticks/revolution
    public static final double launchRatio = (double) 16 / 24; // this is correct because 5202-0002-0001's gearbox ratio is 1:1, and we go from a 16tooth -> 24tooth pulley

    // PIDF coefficients
    public static final double launchP = 20; // orig 2.5
    public static final double launchI = 0.01; // orig 0.1
    public static final double launchD = 0.01; // orig 0.2
    public static final double launchF = (double) 1 / 2000; // TODO: fix this complete guess
    public static final double lowerTransferLowerLimit = 0.28;
    public static final double lowerTransferUpperLimit = 0.49;

    public static final double upperTransferClosed = 0.36; // servo position where upper transfer prevents balls from passing into launch
    public static final double upperTransferOpen = 0.66; // servo position where upper transfer allows balls to pass into launch

    public static final double turnMaxPower = 1; // our max turn power is max (this is prob a bad idea)
    public static final double turnWait = 20;
    public static final int launchDelay = 500; // time to wait for servos to move during launch (in ms)

    public Robot(HardwareMap hw) { // create all of our hardware
        // DC motors (all are DcMotorEx for current monitoring)
        frontLeft = hw.get(DcMotorEx.class, "frontLeft");
        frontRight = hw.get(DcMotorEx.class, "frontRight");
        backLeft = hw.get(DcMotorEx.class, "backLeft");
        backRight = hw.get(DcMotorEx.class, "backRight");
        intake = hw.get(DcMotorEx.class, "intake");
        launch = hw.get(DcMotorEx.class, "launch");

        // servos
        lowerTransfer = hw.get(Servo.class, "lowerTransfer");
        upperTransfer = hw.get(Servo.class, "upperTransfer");

        // sensors
        odo = hw.get(GoBildaPinpointDriver.class, "odo");

        // set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD); // used to be reversed
        backRight.setDirection(DcMotorSimple.Direction.FORWARD); // used to be reversed
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launch.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // brake when we turn off the motor
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // brake when we turn off the motor
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // brake when we turn off the motor
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // brake when we turn off the motor

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
        // set up odo
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU(); // set pos to 0, 0, 0 and recalibrate IMU
    }

    public void stopAllMotors() { // stops all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public double[] getXYFromGoal(double goalX, double goalY) {
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
        return odo.getHeading(AngleUnit.RADIANS);
    }
    public double getLaunchRPM() { // return launch velocity in RPM
        return (launch.getVelocity() / Robot.TICKS_PER_REV ) * 60;
    }
    public double getLaunchRadians() { // return launch velocity in radians/second
        return ((launch.getVelocity() / Robot.TICKS_PER_REV) * 2 * Math.PI);
    }
    public double getLaunchCurrent() { // return launch current in amps
        return launch.getCurrent(CurrentUnit.AMPS);
    }
    public double getNeededVelocity(double goalX, double goalY) { // returns dst from goal in MM
        double d = getDstFromGoal(goalX, goalY);
        double numerator = 19.62 * Math.pow(d, 2);
        double denominator = (Math.pow(3, 0.5) * d) - 0.8;
        return Math.pow(numerator / denominator, 0.5); // thank u rahul
    }
    public void moveMM(double x, double y) { // move x and y

    }
    public double getLaunchHeading(double goalX, double goalY) {
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

    public void rotateToHeading(double target) throws InterruptedException { // rotate the bot to a heading in radians
        double error = angleDifference(target);

        while (Math.abs(error) > 0.01) { // ~0.5deg tolerance
            error = angleDifference(target);

            double turnPower = turnMaxPower * Math.signum(error); // reduce our turning speed by max turn power

            // set our motors to the turn power
            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower); // right motors are opposite
            backRight.setPower(-turnPower); // right motors are opposite

            sleep((long) turnWait);
        }

        stopAllMotors(); // stop all motors
    }
    public double setLaunchTangentialSpeed(double tangentialSpeed) { // input tangentialSpeed (in m/s) and set launch velocity to have ball shoot at that speed
        double TPS = 0;
        // this will be the hardest function to code
        // it basically needs to be a relation between the rotational speed of launch and the actual output speed of the ball
        // at the end, we will output the desired TPS of our motor to monitor once it reaches it
        return TPS;
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