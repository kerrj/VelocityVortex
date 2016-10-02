package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by hunai on 9/16/2016.
 */

@Autonomous
@Disabled
public class ColorDrive extends LinearOpMode{
    public void runOpMode() throws InterruptedException{
//        ColorSensor floorSensor;
//        ColorSensor beaconSensor;
//        DcMotor left;
//        DcMotor right;
//        hardwareMap.logDevices();
//        floorSensor=hardwareMap.colorSensor.get("floorSensor");
//        beaconSensor=hardwareMap.colorSensor.get("beaconSensor");
//        left=hardwareMap.dcMotor.get("left");
//        right=hardwareMap.dcMotor.get("right");
//        right.setDirection(DcMotor.Direction.REVERSE);
        I2cAddr f=I2cAddr.create8bit(0x42);
////        I2cAddr b=I2cAddr.create7bit(0x6c);
//        floorSensor.setI2cAddress(f);
////        beaconSensor.setI2cAddress(b);
        waitForStart();
//        while((floorSensor.red()<200)&&(floorSensor.blue()<200)&&(floorSensor.green()<200)&&opModeIsActive()){
//            left.setPower(.5);
//            right.setPower(.5);
//            telemetry.addData("R",floorSensor.red());
//            telemetry.addData("G",floorSensor.green());
//            telemetry.addData("B",floorSensor.blue());
//        }
//        left.setPower(0);
//        right.setPower(0);
        while(opModeIsActive()){
            Log.d("active", "active");
            telemetry.addData("Test","test");
//            telemetry.addData("Sensor",floorSensor.toString());
//            telemetry.addData("address",floorSensor.getI2cAddress());
//            telemetry.addData("R",floorSensor.red());
//            telemetry.addData("G",floorSensor.green());
//            telemetry.addData("B",floorSensor.blue());
            idle();
        }
//        if(beaconSensor.red()>200){
//
//        }
    }
}
