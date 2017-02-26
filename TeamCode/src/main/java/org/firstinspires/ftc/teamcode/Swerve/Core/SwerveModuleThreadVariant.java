package org.firstinspires.ftc.teamcode.Swerve.Core;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;
import android.widget.Toast;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.Swerve.Core.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Swerve.Core.PID;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Created by hunai on 9/14/2016.
 * @author Duncan
 */
public class SwerveModuleThreadVariant {
    private DcMotor driveMotor; //SpeedController used so this can be talon, victor, jaguar, CAN talon...
    public  Servo steerServo;
    public AbsoluteEncoder steerEncoder;
    public ServoController controller;
    public int portNumber;
    public double positionX, positionY; //position of this wheel relative to the center of the robot
    //from the robot's perspective, +y is forward and +x is to the right
    private double targetAngle = 0;//initialize these as 0
    private double targetServoPower=.5;
    private DcMotorSimple.Direction targetServoDirection= DcMotorSimple.Direction.FORWARD;
    private double motorPower = 0,lastMotorPower=0;
    public  PID pid;
    public enum ModuleDirection{counterclockwise,clockwise}

    private File directory;
    private File pidFile;
    private boolean running=true;

    public double currentAngle;
    public double lastServoPower;
    //    public double currentMotorPower;
    public DcMotorSimple.Direction currentServoDirection= DcMotorSimple.Direction.FORWARD;


    /**
     * @param driveMotor motor controller for drive motor
     * @param steerServo servo controller for steer motor
     * @param steerEncoder absolute encoder on steering motor
     * @param positionX x coordinate of wheel relative to center of robot (inches)
     * @param positionY y coordinate of wheel relative to center of robot (inches)
     */
    public SwerveModuleThreadVariant(DcMotor driveMotor, Servo steerServo, AbsoluteEncoder steerEncoder, double positionX, double positionY) {
        this.steerServo = steerServo;
        this.controller=steerServo.getController();
        this.portNumber=steerServo.getPortNumber();
        this.driveMotor = driveMotor;
        this.steerEncoder = steerEncoder;
        this.positionX = positionX;
        this.positionY = positionY;
        directory= FtcRobotControllerActivity.getActivity().getBaseContext().getExternalFilesDir(null);
        pidFile=new File(directory,"pid.txt");
        if(!pidFile.exists()){
            try {
                pidFile.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        String contents="";
        try {
            FileInputStream fis=new FileInputStream(pidFile);
            byte[] buf=new byte[fis.available()];
            fis.read(buf,0,buf.length);
            contents=new String(buf,"UTF-8");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        String p = contents.substring(0, contents.indexOf(";"));
        String ida = contents.substring(contents.indexOf(";")+1);
        String i=ida.substring(0,ida.indexOf(";"));
        String da=ida.substring(ida.indexOf(";")+1);
        String d=da.substring(0,da.indexOf(";"));
        String a=da.substring(da.indexOf(";")+1);
        Log.d("P", p);
        Log.d("I", i);
        Log.d("D", d);
        Log.d("A",a);
        try {
            double KP = Double.valueOf(p);
            double KI = Double.valueOf(i);
            double KD = Double.valueOf(d);
            double KA=Double.valueOf(a);
            pid=new PID(KP, KI,KD,driveMotor,KA);
        }catch(NumberFormatException e){
            e.printStackTrace();
            pid=new PID(2.0,1.0,0,driveMotor,0);
        }
        //        pid=new PID(2.0/3.0,2.5,.01,driveMotor,1);
        currentAngle=steerEncoder.getAngle();
        //        currentMotorPower=driveMotor.getPower();
        lastServoPower=steerServo.getPosition();
    }
    /**
     * @param angle in radians
     * @param speed motor speed [-1 to 1]
     */
    public void set(double angle, double speed) {
        angle = wrapAngle(angle);
        Vector currentVector=new Vector(Math.cos(currentAngle),Math.sin(currentAngle));
        Vector targetVector =new Vector(Math.cos(angle),Math.sin(angle));
        double dist=Math.acos(Vector.dotProduct(currentVector,targetVector)/(currentVector.getMagnitude()*targetVector.getMagnitude()));
        //if the setpoint is more than 90 degrees from the current position, flip everything
        if (Math.abs(dist) > Math.PI / 2) {
            angle = wrapAngle(angle + Math.PI);
            speed *= -1;
        }
        targetAngle = angle;
        motorPower = speed;
    }

    public void setMotorPower(double power){
        motorPower=power;
    }

    public static double wrapAngle(double angle) {
        angle %= 2*Math.PI;
        if (angle<0) angle += 2*Math.PI;
        return angle;
    }
    public void setServoPower( double speed, double initPosition) {
        double newSpeed = speed;
        if (speed == 0) {
            newSpeed = initPosition;
        } else if (speed < 0) {
            newSpeed = speed / 2 + initPosition;
        } else if (speed > 0) {
            newSpeed = speed / 2 + initPosition;
        }

        if(newSpeed>1){
            newSpeed=1;
        }
        if(newSpeed<0){
            newSpeed=0;
        }
        targetServoPower = newSpeed;
    }




    public double getDelta(){
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(currentAngle), Math.sin(currentAngle));

        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
        return angleBetween;
    }

    public double getDelta(double theoreticalAngle){
        Vector currentVector=new Vector(Math.cos(currentAngle),Math.sin(currentAngle));
        Vector targetVector =new Vector(Math.cos(theoreticalAngle),Math.sin(theoreticalAngle));
        double dist=Math.acos(Vector.dotProduct(currentVector,targetVector)/(currentVector.getMagnitude()*targetVector.getMagnitude()));
        //if the setpoint is more than 90 degrees from the current position, flip everything

        if (Math.abs(dist) > Math.PI / 2) {
            theoreticalAngle=wrapAngle(theoreticalAngle+Math.PI);
        }
        targetVector = new Vector(Math.cos(theoreticalAngle), Math.sin(theoreticalAngle));
        currentVector = new Vector(Math.cos(currentAngle), Math.sin(currentAngle));

        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
        return angleBetween;
    }

    public ModuleDirection getDirection(double angle){
        if (getDelta(angle) > 0) {
            return ModuleDirection.counterclockwise;
        } else {
            return ModuleDirection.clockwise;
        }
    }
    public void update(){
        currentAngle=steerEncoder.getAngle();
        if(Math.abs(lastMotorPower-motorPower)>.025||motorPower==0) {
            driveMotor.setPower(motorPower);
            lastMotorPower=motorPower;
        }
        setServoPower( pid.setPIDpower(-getDelta(),motorPower), .5);//negative
        if(Math.abs(targetServoPower-lastServoPower)>.02||targetServoPower==.5){
            if(targetServoPower>.75){
                targetServoPower=1;
            }else if(targetServoPower<.25){
                targetServoPower=0;
            }
            controller.setServoPosition(portNumber, targetServoPower);
            lastServoPower=targetServoPower;
        }
    }

//    public void run(){
//        while(running){
//            double startTime=System.nanoTime()/1.0E6;
//            currentAngle=steerEncoder.getAngle();
//            if(Math.abs(lastMotorPower-motorPower)>.025||motorPower==0) {
//                driveMotor.setPower(motorPower);
//                lastMotorPower=motorPower;
//            }
//            setServoPower( pid.setPIDpower(-getDelta(),motorPower), .5);//negative
//            if(Math.abs(targetServoPower-lastServoPower)>.02||targetServoPower==.5){
//                if(targetServoPower>.75){
//                    targetServoPower=1;
//                }else if(targetServoPower<.25){
//                    targetServoPower=0;
//                }
//                controller.setServoPosition(portNumber, targetServoPower);
//                lastServoPower=targetServoPower;
//            }
////            if(Math.abs(targetServoPower-lastServoPower)>.02||targetServoPower==.5){
////                controller.setServoPosition(portNumber, targetServoPower);
////                lastServoPower=targetServoPower;
////            }
//            if(System.nanoTime()/1.0E6-startTime<1){
//                try {
//                    Thread.sleep(1);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }
//        }
//
//        kill();
//    }

    public void kill(){
        running=false;
        steerServo.setPosition(.5);
        driveMotor.setPower(0);
    }

}