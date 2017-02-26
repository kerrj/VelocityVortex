package org.firstinspires.ftc.teamcode.Swerve.Core;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Justin on 2/25/2017.
 */
public class SwerveThread {
    private MotorThread motorThread1,motorThread2;
    private ServoThread servoThread;


    public SwerveThread(SwerveModuleThreadVariant[] modules){
        motorThread1=new MotorThread(modules[0]);
        motorThread2=new MotorThread(modules[1]);
        motorThread1.start();
        motorThread2.start();
        servoThread=new ServoThread(modules);
        servoThread.start();
    }



    public void kill(){
        motorThread1.kill();
        motorThread2.kill();
        servoThread.kill();
    }


    private class ServoThread extends Thread{
        private boolean running=true;
        private SwerveModuleThreadVariant[] modules;
        public ServoThread(SwerveModuleThreadVariant[] modules){
            this.modules=modules;
        }
        public void kill(){
            running=false;
        }
        public void run(){
            while(running){
                double start=System.nanoTime()/1.0E6;
                for(SwerveModuleThreadVariant m:modules){
                    m.updateServo();
                }
                if(System.nanoTime()/1.0E6-start<2){
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }




    private class MotorThread extends Thread{
        private boolean running=true;
        private SwerveModuleThreadVariant module;
        public MotorThread(SwerveModuleThreadVariant module){
            this.module=module;
        }
        public void kill(){
            running=false;
        }
        public void run(){
            while(running){
                double start=System.nanoTime()/1.0E6;
                module.updateMotor();
                if(System.nanoTime()/1.0E6-start<1){
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }
}
