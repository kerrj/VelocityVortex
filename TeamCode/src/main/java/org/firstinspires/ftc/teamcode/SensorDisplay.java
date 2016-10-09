package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

/**
 * Created by Justin on 10/7/2016.
 */
@TeleOp
public class SensorDisplay extends OpMode {
    private UltrasonicSensor ultra;
    private ColorSensor color;
    private DcMotor motor;
    private GyroSensor gyro;
    private TouchSensor touch;
    private IrSeekerSensor ir;

    @Override
    public void init() {
        ultra=hardwareMap.ultrasonicSensor.get("ultra");
        color=hardwareMap.colorSensor.get("color");
        motor=hardwareMap.dcMotor.get("motor");
        gyro=hardwareMap.gyroSensor.get("gyro");
        touch=hardwareMap.touchSensor.get("touch");
        ir=hardwareMap.irSeekerSensor.get("ir");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance",ultra.getUltrasonicLevel());
        telemetry.addData("red",color.red());
        telemetry.addData("green",color.green());
        telemetry.addData("blue",color.blue());
        telemetry.addData("Heading",gyro.getHeading());
        telemetry.addData("Touch",touch.isPressed());
        telemetry.addData("Counts",motor.getCurrentPosition());
        telemetry.addData("IRAngle",ir.getAngle());
    }
}
