package org.firstinspires.ftc.teamcode.Swerve.Core;

/**
 * Created by Justin on 2/25/2017.
 */
public class SwerveThread extends Thread {
    SwerveModuleThreadVariant[] modules;
    boolean running=true;

    public SwerveThread(SwerveModuleThreadVariant[] modules){
        this.modules=modules;
    }
    public void kill(){
        running=false;
    }
    public void run(){
        while(running){
            double start=System.nanoTime()/1.0E6;
            for(SwerveModuleThreadVariant m:modules){
                m.update();
            }
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
