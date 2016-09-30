package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hunai on 9/14/2016.
 */
public class Mecanum_comb extends OpMode{
    DcMotor left_front;
    DcMotor left_back;
    DcMotor right_front;
    DcMotor right_back;
    public double clip(double d){
        double dl=d;
        if(d>1){
            dl=1;
        }else if(d<-1){
            dl=-1;
        }
        return dl;
    }

    double x, y, Rotation_Velocity, Speed, Turn_Angle;
    double lf_pwr, lb_pwr, rf_pwr, rb_pwr;
    double pi=Math.PI;
    public void init(){
        left_front = hardwareMap.dcMotor.get("lf");
        left_back = hardwareMap.dcMotor.get("lb");
        right_front = hardwareMap.dcMotor.get("rf");
        right_back = hardwareMap.dcMotor.get("rb");
        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop(){
        x=gamepad1.left_stick_x;
        y=gamepad1.left_stick_y;
        Speed=Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        Turn_Angle=Math.atan2(y, x);
        Rotation_Velocity=gamepad1.right_stick_x;
        lf_pwr=Speed*Math.sin(Turn_Angle-(pi/4))-Rotation_Velocity;
        rf_pwr=Speed*Math.cos(Turn_Angle-(pi/4))+Rotation_Velocity;
        lb_pwr=Speed*Math.cos(Turn_Angle-(pi/4))-Rotation_Velocity;
        rb_pwr=Speed*Math.sin(Turn_Angle-(pi/4))+Rotation_Velocity;
        left_front.setPower((lf_pwr)/2);
        left_back.setPower((lb_pwr)/2);
        right_front.setPower((rf_pwr)/2);
        right_back.setPower((rb_pwr)/2);
    }
    public void stop(){


    }
}
