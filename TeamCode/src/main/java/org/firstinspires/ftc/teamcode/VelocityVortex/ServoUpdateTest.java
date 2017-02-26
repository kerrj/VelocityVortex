package org.firstinspires.ftc.teamcode.VelocityVortex;

import android.graphics.Path;
import android.util.Log;

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
public class ServoUpdateTest extends Robot{

    boolean b=true;
    double max=0,readmax1=0,readmax2=0;
    ReadThread thread1,thread2;
    @Override
    public void init() {
        super.init();
        thread1=new ReadThread();
        thread1.setServo(capLeft);
        thread1.setNumber(1);
//        thread2=new ReadThread();
//        thread2.setServo(capRight);
//        thread2.setNumber(2);
        thread1.start();
//        thread2.start();
    }
    @Override
    public void loop() {
        super.loop();
//        if(gamepad1.a) {
//            double start = System.nanoTime() / 1.0E6;
//            if (b) {
//                capLeft.setPosition(.5);
////                lfa.getVoltage();
//                b=false;
//            } else {
//                capLeft.setPosition(.4);
////                lfa.getVoltage();
//                b=true;
//            }
//            double end=System.nanoTime()/1.0E6;
//            max=Math.max(max,end-start);
//            telemetry.addData("WriteElapsed", end - start);
//            telemetry.addData("WriteMax",max);
//        }else{
//            max=0;
//            readmax=0;
//            double start = System.nanoTime() / 1.0E6;
//            capLeft.setPosition(.5);
//            telemetry.addData("WriteElapsed", System.nanoTime() / 1.0E6 - start);
//        }
        telemetry.addData("WriteMax1",readmax1);
        telemetry.addData("WriteMax2",readmax2);
    }
    public void stop(){
        super.stop();
        thread1.running=false;
        thread2.running=false;
    }

//    public class ReadThread extends Thread{
//        public boolean running=true;
//        boolean a=true;
//        public void run(){
//            while(running){
//                double start = System.nanoTime() / 1.0E6;
//                if (a) {
////                    capLeft.setPosition(.5);
//                    lfa.getVoltage();
//                    a=false;
//                } else {
////                    capLeft.setPosition(.4);
//                    lfa.getVoltage();
//                    a=true;
//                }
//                double end=System.nanoTime()/1.0E6;
//                readmax=Math.max(readmax,end-start);
//            }
//        }
//    }
public class ReadThread extends Thread{
    public boolean running=true;
    boolean a=true;
    Servo servo;
    int n=0;
    public void setServo(Servo servo){
        this.servo=servo;
    }
    public void setNumber(int n){
        this.n=n;
    }
    public void run(){
        while(running){
            if(gamepad1.a) {
                double start = System.nanoTime() / 1.0E6;
                if (a) {
                    capLeft.setPosition(.5);
                    capRight.setPosition(.5);
                    //                lfa.getVoltage();
                    a = false;
                } else {
                    capLeft.setPosition(.4);
                    capRight.setPosition(.4);
                    //                lfa.getVoltage();
                    a = true;
                }
                double end = System.nanoTime() / 1.0E6;
                if(n==1){
                    readmax1 = Math.max(readmax1, end - start);
                }else{
                    readmax2 = Math.max(readmax2, end - start);
                }
            }
        }
    }
}
}
