package org.firstinspires.ftc.teamcode.VelocityVortex;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Justin on 11/11/2016.
 */
@TeleOp
@Disabled
public class AlanMotorTest extends OpMode{
    DcMotor left,right;
    Servo servo;
    double power=0;
    long lastSet,lastPush;

    @Override
    public void init() {
        left=hardwareMap.dcMotor.get("left");
        right=hardwareMap.dcMotor.get("right");
        servo=hardwareMap.servo.get("servo");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastSet=System.currentTimeMillis();
        lastPush=System.currentTimeMillis();
    }
    @Override
    public void loop() {
        if(gamepad1.y){
            servo.setPosition(.5);
            lastPush=System.currentTimeMillis();
        }
        if(System.currentTimeMillis()-lastPush>500){
            servo.setPosition(0);
        }
        if(System.currentTimeMillis()-lastSet>300){
            if(gamepad1.a){
                power-=.05;
                if(power<0){
                    power=0;
                }
                lastSet=System.currentTimeMillis();
            }
            if(gamepad1.b) {
                power += .05;
                if (power > 1) {
                    power = 1;
                }
                lastSet=System.currentTimeMillis();
            }
        }
        left.setPower(power);
        right.setPower(power);
        telemetry.addData("Power",power);
    }
}
