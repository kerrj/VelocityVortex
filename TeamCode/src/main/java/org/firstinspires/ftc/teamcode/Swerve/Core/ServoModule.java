package org.firstinspires.ftc.teamcode.Swerve.Core;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by hunai on 9/14/2016.
 * @author Duncan
 */
public class ServoModule {
    private Servo steerServo;
    private AbsoluteEncoder steerEncoder;
    //from the robot's perspective, +y is forward and +x is to the right
    private double targetAngle = 0;//initialize these as 0


    private double PROPORTION=1;
    private double INTEGRAL_TIME=.01;
    private double DERIVATIVE_TIME=.1;

    private double previousError=0;
    private double integral=0;
    private long lastTime;


    private double setPIDPower(){
        double dt=(System.currentTimeMillis()-lastTime)/1000;
        lastTime=System.currentTimeMillis();
        double error=getDelta();

        integral+=error*dt;
        double derivative=(error-previousError)/dt;
        previousError=error;

        double output=.5+   PROPORTION*(error+(integral/INTEGRAL_TIME)+(DERIVATIVE_TIME*derivative));
        if(output>1){
            output=1;
        }else if(output<0){
            output=0;
        }
        return output;
    }

    /**
     * @param steerServo   servo controller for steer motor
     * @param steerEncoder absolute encoder on steering motor
     */
    public ServoModule(Servo steerServo, AbsoluteEncoder steerEncoder) {
        this.steerServo = steerServo;
        this.steerEncoder = steerEncoder;
        lastTime=System.currentTimeMillis();
    }

    public double getDelta(){
        Vector targetVector = new Vector(Math.cos(targetAngle), Math.sin(targetAngle));
        Vector currentVector = new Vector(Math.cos(steerEncoder.getAngle()), Math.sin(steerEncoder.getAngle()));
        //angleBetween is the angle from currentPosition to target position in radians
        //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
        double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x,
                currentVector.x * targetVector.x + currentVector.y * targetVector.y);
        return angleBetween;
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
        //set power if close enough
    }
}
