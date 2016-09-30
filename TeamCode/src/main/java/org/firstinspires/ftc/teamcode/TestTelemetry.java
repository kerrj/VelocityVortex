package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by hunai on 9/23/2016.
 */
@TeleOp
public class TestTelemetry extends OpMode {
    ColorSensor floorSensor;
    ColorSensor beaconSensor;
    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor rangeSensor;
    DcMotor right;
    DcMotor left;
    State state_s;
    enum State{
        Drivestraight, Turn, Driveparralelcorner, Checkdisfromwall, Stop, Drivetoline
    }

    @Override
    public void init() {
        right=hardwareMap.dcMotor.get("right");
        left=hardwareMap.dcMotor.get("left");
        floorSensor=hardwareMap.colorSensor.get("floorSensor");
        beaconSensor=hardwareMap.colorSensor.get("beaconSensor");
        gyro=hardwareMap.gyroSensor.get("gyro");
        rangeSensor=(ModernRoboticsI2cRangeSensor)hardwareMap.i2cDeviceSynch.get("rangeSensor");
        I2cAddr f=I2cAddr.create8bit(0x42);
        I2cAddr b=I2cAddr.create8bit(0x6c);
        beaconSensor.setI2cAddress(b);
        floorSensor.setI2cAddress(f);
        left.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("blue", floorSensor.blue());
        telemetry.addData("green", floorSensor.green());
    }

    @Override
    public void loop() {
        switch(state_s) {
            case Drivestraight:
                if(right.getCurrentPosition()<5){
                    right.setPower(.3);
                    left.setPower(.3);
                }
                state_s=State.Drivestraight;
                break;
            case Turn:
                state_s=State.Turn;
                break;
            case Drivetoline:
            if (floorSensor.red() < 8 && floorSensor.green() < 8 && floorSensor.blue() < 8) {
                right.setPower(.1);
                left.setPower(.1);
                telemetry.addData("red", floorSensor.red());
                telemetry.addData("blue", floorSensor.blue());
                telemetry.addData("green", floorSensor.green());
            }
                break;
            case Stop:
            if(floorSensor.red() > 8 && floorSensor.green() > 8 && floorSensor.blue() > 8) {
                right.setPower(0);
                left.setPower(0);
                telemetry.addData("red", floorSensor.red());
                telemetry.addData("blue", floorSensor.blue());
                telemetry.addData("green", floorSensor.green());
            }
            break;
        }
//        telemetry.addData("test","test");
//        telemetry.addData("Sensor",floorSensor.toString());
//        telemetry.addData("address",floorSensor.getI2cAddress().get8Bit());
//        telemetry.addData("RF",floorSensor.red());
//        telemetry.addData("GF",floorSensor.green());
//        telemetry.addData("BF",floorSensor.blue());
//        telemetry.addData("RB",beaconSensor.red());
//        telemetry.addData("GB", beaconSensor.green());
//        telemetry.addData("BB", beaconSensor.blue());
    }
}
