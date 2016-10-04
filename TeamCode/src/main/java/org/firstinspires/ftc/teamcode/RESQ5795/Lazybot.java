package org.firstinspires.ftc.teamcode.RESQ5795;

/**
 * Created by hunai on 10/3/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hunai on 7/21/2016.
 */
public class Lazybot extends LinearOpMode{
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    public void runOpMode()throws InterruptedException{
        frontLeft=hardwareMap.dcMotor.get("frontLeft");
        frontRight=hardwareMap.dcMotor.get("frontRight");
        backLeft=hardwareMap.dcMotor.get("backLeft");
        backRight=hardwareMap.dcMotor.get("backRight");
        waitForStart();
        double X2=0, Y1=0, X1=0, threshold=.1;
        while(opModeIsActive()){
            X1 = gamepad1.left_stick_x;
            Y1 = gamepad1.left_stick_y;
            X2 = gamepad1.right_stick_x;
            if (Math.abs(X1) < threshold) X1 = 0;
            if (Math.abs(X2) < threshold) X2 = 0;
            if (Math.abs(Y1) < threshold) Y1 = 0;

            frontLeft.setPower(Math.max(-1, Math.min(1, Y1 + X1 + X2)));
            frontRight.setPower(Math.max(-1, Math.min(1, -Y1 + X1 + X2)));
            backLeft.setPower(Math.max(-1, Math.min(1, Y1 - X1 + X2)));
            backRight.setPower(Math.max(-1, Math.min(1, -Y1 - X1 + X2)));
        }

    }
}


