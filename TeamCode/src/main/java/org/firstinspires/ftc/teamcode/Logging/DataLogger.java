package org.firstinspires.ftc.teamcode.Logging;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.TimeZone;

/**
 * Created by Justin on 11/10/2016.
 */
public class DataLogger {
    private long refreshTime;
    private LoggingThread thread;
    private boolean running=false;
    private Robot robot;

    private ArrayList<DataPoint> dataPoints;

    public static DataLogger create(long refreshTime){
        DataLogger logger=new DataLogger();
        logger.refreshTime=refreshTime;
        logger.dataPoints=new ArrayList<>();
        return logger;
    }

    public void mapHardware(Robot robot){
        this.robot=robot;
    }

    public void start(){
        thread=new LoggingThread();
        running=true;
        thread.start();
    }

    public void stop(){
        running=false;
    }

    private class LoggingThread extends Thread{
        public void run(){
            while(running){
                if(System.currentTimeMillis()%refreshTime==0){
                    double[] joystickValues={0,0,0,0,0,0,0,0};
                    if(robot.gamepad1!=null){
                        joystickValues[0]=robot.gamepad1.left_stick_x;
                        joystickValues[1]=robot.gamepad1.left_stick_y;
                        joystickValues[2]=robot.gamepad1.right_stick_x;
                        joystickValues[3]=robot.gamepad1.right_stick_y;
                    }
                    if(robot.gamepad2!= null){
                        joystickValues[4]=robot.gamepad2.left_stick_x;
                        joystickValues[5]=robot.gamepad2.left_stick_y;
                        joystickValues[6]=robot.gamepad2.right_stick_x;
                        joystickValues[7]=robot.gamepad2.right_stick_y;
                    }
                    double[] motorPowers={robot.lfm.getPower(),robot.rfm.getPower(),robot.rbm.getPower(),robot.lbm.getPower()};
                    double[] servoPositions={robot.lfe.getAngle(),robot.rfe.getAngle(),robot.rbe.getAngle(),robot.lbe.getAngle()};
                    HashMap<String,double[]> vuforiaData=null;
                    if(robot.vuforia!=null){
                        vuforiaData=robot.vuforia.getVuforiaData();
                    }
                    int[] encoderPositions={robot.lfm.getCurrentPosition(),robot.rfm.getCurrentPosition(),robot.rbm.getCurrentPosition(),robot.lbm.getCurrentPosition()};
                    double neckPosition=robot.neck.getPosition();
                    dataPoints.add(new DataPoint(joystickValues,servoPositions,motorPowers,vuforiaData,neckPosition,encoderPositions));
                }
            }
        }
        public void writeToFile(){
            Date date=new Date();
            String filename=Integer.toString(date.getHours())+"-"+Integer.toString(date.getMinutes())+"-"+Integer.toString(date.getSeconds())+"_"+Integer.toString(date.getMonth()+1)+"-"+Integer.toString(date.getDay());
            File file=new File("storage/emulated/0/5795DataLogs"+filename+".txt");
            file.mkdir();
            try {
                file.createNewFile();
                FileOutputStream fos=new FileOutputStream(file);

            } catch (IOException e) {
                e.printStackTrace();
                Log.e("DataLogger","Unable to print to file");
            }
        }

        public String parse(){
            String contents="";
            for(DataPoint dataPoint:dataPoints){
                contents+="\n";
                contents+=dataPoint.toString();
            }
            return "";
        }
    }
}
