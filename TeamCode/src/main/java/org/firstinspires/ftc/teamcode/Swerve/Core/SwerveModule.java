package org.firstinspires.ftc.teamcode.Swerve.Core;


import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Swerve.Core.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Swerve.Core.PID;

/**
 * Created by hunai on 9/14/2016.
 * @author Duncan
 */
public class SwerveModule {
    private DcMotor driveMotor; //SpeedController used so this can be talon, victor, jaguar, CAN talon...
    private Servo steerServo;
    private AbsoluteEncoder steerEncoder;
    public double positionX, positionY; //position of this wheel relative to the center of the robot
    //from the robot's perspective, +y is forward and +x is to the right
    private boolean enabled = true;
    private double targetAngle = 0;//initialize these as 0
    private double motorPower = 0;
    private PID pid;


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
        pid=new PID(.2, 0,0);
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
    public void enable() {
        enabled = true;
    }

    public void disable() {
        driveMotor.setPower(0);
        steerServo.setPosition(.5);
        enabled = false;
    }

    public void rest() {
        driveMotor.setPower(0);
    }

    private double wrapAngle(double angle) {
        angle %= 2*Math.PI;
        if (angle<0) angle += 2*Math.PI;
        return angle;
    }
    public double setServoPower(Servo continuousServo, double speed, double initPosition) {
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
        if(newSpeed<-1){
            newSpeed=-1;
        }
        continuousServo.setPosition(newSpeed);
        return newSpeed;
    }
    public void update(boolean waitForServos) {
        if (!waitForServos) {//motors not waiting for servos
            driveMotor.setPower(motorPower);//set the motor power
//            Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
//            Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));
//
//            //angleBetween is the angle from currentPosition to target position in radians
//            //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
//            double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
//            setServoPower(steerServo, pid.setPIDpower(angleBetween), .5);
            Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
            Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));
            //angleBetween is the angle from currentPosition to target position in radians
            //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
            double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
            //give the servo a piecewise scaling function: full speed until about 5 degrees away, then linearly slower
            if (angleBetween > Math.toRadians(50)){
                steerServo.setPosition(0);
            } else if (angleBetween < -Math.toRadians(50)) {
                steerServo.setPosition(1);
            }else{
                double scaleFactor = Math.abs(angleBetween) / Math.toRadians(50);
                if(angleBetween>0){
                    steerServo.setPosition(.5 - .5 * Math.abs(scaleFactor));
                }else if(angleBetween<0){
                    steerServo.setPosition(.5 + .5 * Math.abs(scaleFactor));

                }
            }
        } else {//motors waiting for servos
            driveMotor.setPower(0);
//            Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
//            Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));
//
//            //angleBetween is the angle from currentPosition to target position in radians
//            //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
//            double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
//            setServoPower(steerServo, pid.setPIDpower(angleBetween), .5);
//
//            //set power if close enough
//            if (Math.abs(angleBetween) < .25) {
//                driveMotor.setPower(motorPower);
//            }
            Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
            Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));
            //angleBetween is the angle from currentPosition to target position in radians
            //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
            double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
            //give the servo a piecewise scaling function: full speed until about 5 degrees away, then linearly slower
            if (angleBetween > Math.toRadians(40)){
                steerServo.setPosition(0);
            } else if (angleBetween < -Math.toRadians(40)) {
                steerServo.setPosition(1);
            }else{
                double scaleFactor = Math.abs(angleBetween) / Math.toRadians(40);
                if(angleBetween>0){
                    steerServo.setPosition(.5 - .5 * Math.abs(scaleFactor));
                }else if(angleBetween<0){
                    steerServo.setPosition(.5 + .5 * Math.abs(scaleFactor));

                }
            }
            if(Math.abs(angleBetween)<Math.toRadians(40)){
                driveMotor.setPower(motorPower);
            }
        }
    }

    public void stop(){
        motorPower=0;
    }
}