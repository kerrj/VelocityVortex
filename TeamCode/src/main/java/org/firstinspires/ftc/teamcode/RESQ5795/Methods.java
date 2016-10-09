package org.firstinspires.ftc.teamcode.RESQ5795;

/**
 * Created by hunai on 10/3/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by hunai on 6/14/2016.
 */
public class Methods extends LinearOpMode {
    GyroSensor gyro;//set all these names for your robot specifically
    DcMotor L;//left motor
    DcMotor R;//right motor
    int ENCODER_CPR = 1120;    //encoder counts per revolution
    double GEAR_RATIO = 1.5;     //gear ratio
    double WHEEL_DIAMETER = 3.3;    //diameter of wheel in inches
    double DISTANCE;
    double CIRCUMFERENCE = 12.07;
    public double Counts(double distance) {
        //DISTANCE in inches
        this.DISTANCE = distance;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
        return COUNTS;
    }

    public double Distance(double COUNTS) {
        double ROTATIONS = COUNTS / GEAR_RATIO / ENCODER_CPR;
        double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double distance = ROTATIONS * CIRCUMFERENCE;
        return distance;
    }

    public void turnLeft(int degrees, double speed) throws InterruptedException {
        int startHeading = gyro.getHeading();
        int targetHeading = startHeading - degrees;
        if (targetHeading < 0) {
            targetHeading = targetHeading + 360;
        }
        while (gyro.getHeading() < targetHeading - 3 || gyro.getHeading() > targetHeading + 3) {
            L.setPower(-speed);
            R.setPower(speed);
            telemetry.addData("currentheading", gyro.getHeading());
            telemetry.addData("targetHeading", targetHeading);
            waitOneFullHardwareCycle();
        }
        L.setPower(0);
        R.setPower(0);
    }

    public void turnRight(int degrees, double speed) throws InterruptedException {
        int startHeading = gyro.getHeading();
        int targetHeading = startHeading + degrees;
        if (targetHeading > 360) {
            targetHeading = targetHeading - 360;
        }
        while (gyro.getHeading() < targetHeading - 3 || gyro.getHeading() > targetHeading + 3) {
            L.setPower(speed);
            R.setPower(-speed);

            telemetry.addData("currentheading", gyro.getHeading());
            telemetry.addData("targetHeacx ding", targetHeading);
            waitOneFullHardwareCycle();
        }
        L.setPower(0);
        R.setPower(0);
    }

    public void goForward(double finaldistance, double speed) throws InterruptedException {
        double initialcount=L.getCurrentPosition();
        while (L.getCurrentPosition() < initialcount + Counts(finaldistance)) {
            L.setPower(speed);
            R.setPower(speed);
            waitOneFullHardwareCycle();
        }
        L.setPower(0);
        R.setPower(0);
    }

    public void goBackward(double finaldistance, double speed) throws InterruptedException {
        double initialcount=L.getCurrentPosition();
        while (L.getCurrentPosition() > initialcount - Counts(finaldistance)) {
            L.setPower(-speed);
            R.setPower(-speed);
            waitOneFullHardwareCycle();
        }
        L.setPower(0);
        R.setPower(0);
    }

    public void runOpMode() throws InterruptedException {

    }

}
