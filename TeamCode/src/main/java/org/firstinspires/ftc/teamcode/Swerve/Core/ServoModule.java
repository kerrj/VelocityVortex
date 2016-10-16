package org.firstinspires.ftc.teamcode.Swerve.Core;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hunai on 9/14/2016.
 * @author Duncan
 */
public class ServoModule {
    private Servo steerServo;
    private AbsoluteEncoder steerEncoder;
    //from the robot's perspective, +y is forward and +x is to the right
    private double targetAngle = 0;//initialize these as 0


    /**
     * @param steerServo   servo controller for steer motor
     * @param steerEncoder absolute encoder on steering motor
     */
    public ServoModule(Servo steerServo, AbsoluteEncoder steerEncoder) {
        this.steerServo = steerServo;
        this.steerEncoder = steerEncoder;
    }

    public double getDelta(){
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));
        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
        return Math.abs(angleBetween);
    }

    /**
     * @param angle in radians
     */
    public void set(double angle) {
        targetAngle = wrapAngle(angle);
    }
    private double wrapAngle(double angle) {
        angle %= 2 * Math.PI;
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Method called every loop iteration
     */
    public void update() {
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));
        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);


        //give the servo a piecewise scaling function: full speed until about 5 degrees away, then linearly slower
        if (angleBetween > .1) {//counterclockwise outside 1 degree of correct angle, rotate servo counterclockwise
            steerServo.setPosition(0);
        } else if (angleBetween < -.1) {//clockwise outside 1 degree of correct angle, rotate servo clockwise
            steerServo.setPosition(1);
        }else{
            double scaleFactor = Math.abs(angleBetween) / .1;
            if(angleBetween>0){
                steerServo.setPosition(.5 - .5 * Math.abs(scaleFactor));
            }else if(angleBetween<0){
                steerServo.setPosition(.5 + .5 * Math.abs(scaleFactor));

            }
        }
        //set power if close enough
    }
}
