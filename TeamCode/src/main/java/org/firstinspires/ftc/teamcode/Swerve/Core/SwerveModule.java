package org.firstinspires.ftc.teamcode.Swerve.Core;


import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.Swerve.Core.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Swerve.Core.PID;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

/**
 * Created by hunai on 9/14/2016.
 * @author Duncan
 */
public class SwerveModule {
    private DcMotor driveMotor; //SpeedController used so this can be talon, victor, jaguar, CAN talon...
    public  Servo steerServo;
    public AbsoluteEncoder steerEncoder;
    public double positionX, positionY; //position of this wheel relative to the center of the robot
    //from the robot's perspective, +y is forward and +x is to the right
    private double targetAngle = 0;//initialize these as 0
    private double motorPower = 0;
    public  PID pid;
    public enum ModuleDirection{counterclockwise,clockwise}

    private File directory;
    private File pidFile;

    /**
     * @param driveMotor motor controller for drive motor
     * @param steerServo servo controller for steer motor
     * @param steerEncoder absolute encoder on steering motor
     * @param positionX x coordinate of wheel relative to center of robot (inches)
     * @param positionY y coordinate of wheel relative to center of robot (inches)
     */
    public SwerveModule(DcMotor driveMotor, Servo steerServo, AbsoluteEncoder steerEncoder, double positionX, double positionY) {
        this.steerServo = steerServo;
        this.driveMotor = driveMotor;
        this.steerEncoder = steerEncoder;
        this.positionX = positionX;
        this.positionY = positionY;
        pid=new PID(2/3.0,2.5,0);
    }
    /**
     * @param angle in radians
     * @param speed motor speed [-1 to 1]
     */
    public void set(double angle, double speed) {
        angle = wrapAngle(angle);
        Vector currentVector=new Vector(Math.cos(steerEncoder.getAngle()),Math.sin(steerEncoder.getAngle()));
        Vector targetVector =new Vector(Math.cos(angle),Math.sin(angle));
        double dist=Math.acos(Vector.dotProduct(currentVector,targetVector)/(currentVector.getMagnitude()*targetVector.getMagnitude()));
        //if the setpoint is more than 90 degrees from the current position, flip everything
        if (Math.abs(dist) > Math.PI / 2) {
            angle = wrapAngle(angle + Math.PI);
            speed *= -1;
        }

        targetAngle=angle;
        motorPower=speed;
    }

    public void setPower(double power){
        motorPower=power;
    }


    private double wrapAngle(double angle) {
        angle %= 2*Math.PI;
        if (angle<0) angle += 2*Math.PI;
        return angle;
    }
    public void setServoPower( double speed, double initPosition) {
        double newSpeed = 0;
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
        steerServo.setPosition(newSpeed);
    }

    public double getDelta(){
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));

        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
        return angleBetween;
    }

    public ModuleDirection getDirection(){
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));

        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
        if (angleBetween > 0) {
            return ModuleDirection.counterclockwise;
        } else {
            return ModuleDirection.clockwise;
        }
    }

    public void update() {
        driveMotor.setPower(motorPower);//set the motor power
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));
        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
        setServoPower( pid.setPIDpower(-angleBetween), .5);//negative
        //power curve adjustment
        if(steerServo.getPosition()>.6){
            steerServo.setPosition(1);
        }else if(steerServo.getPosition()<.4){
            steerServo.setPosition(0);
        }
        else if(steerServo.getPosition()>.5&&steerServo.getPosition()<.6){
            double scale=steerServo.getPosition()-.5;
            steerServo.setPosition(.5+scale*2);
        }else if(steerServo.getPosition()>.4&&steerServo.getPosition()<.5){
            double scale=.5-steerServo.getPosition();
            steerServo.setPosition(.5-scale*2);
        }

    }

    public void stop(){
        motorPower=0;
    }
}