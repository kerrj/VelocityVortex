package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Justin on 10/7/2016.
 */
@TeleOp
public class SensorDisplay extends OpMode {
    private ModernRoboticsI2cRangeSensor range;
    private ColorSensor color;
    private GyroSensor gyro;
    private TouchSensor touch;
    private IrSeekerSensor ir;

    @Override
    public void init() {
        I2cDeviceSynch deviceSynch=hardwareMap.i2cDeviceSynch.get("range");
        range=new ModernRoboticsI2cRangeSensor(deviceSynch);
        color=hardwareMap.colorSensor.get("color");
        I2cAddr coloraddress=I2cAddr.create8bit(0x3c);
        color.setI2cAddress(coloraddress);
        color.enableLed(true);
        gyro=hardwareMap.gyroSensor.get("gyro");
        touch=hardwareMap.touchSensor.get("touch");
        ir=hardwareMap.irSeekerSensor.get("ir");
        gyro.calibrate();
    }

    @Override
    public void loop() {
        telemetry.addData("Distance",range.getDistance(DistanceUnit.INCH));
        telemetry.addData("red",color.red());
        telemetry.addData("green",color.green());
        telemetry.addData("blue",color.blue());
        telemetry.addData("Heading",gyro.getHeading());
        telemetry.addData("Touch",touch.isPressed());
        telemetry.addData("IRAngle",ir.getAngle());
    }
}
