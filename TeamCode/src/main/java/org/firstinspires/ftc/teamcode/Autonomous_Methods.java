package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by hunai on 9/23/2016.
 */
public class Autonomous_Methods extends OpMode{
    public void init(){}
    public void loop(){}
    GyroSensor gyro;
    DcMotor L, R;
    int ENCODER_CPR = 1680;    //encoder counts per revolution
    double GEAR_RATIO = 1.5;     //gear ratio
    double WHEEL_DIAMETER = 4;    //diameter of wheel in inches
    double DISTANCE;


    public double Counts(double distance) {
        //DISTANCE in inches
        this.DISTANCE = distance;
        double CIRCUMFERENCE = Math.PI*WHEEL_DIAMETER;
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
            telemetry.addData("targetHeading", targetHeading);
        }
        L.setPower(0);
        R.setPower(0);
    }

}
