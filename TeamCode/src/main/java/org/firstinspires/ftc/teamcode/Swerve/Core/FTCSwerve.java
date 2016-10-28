package org.firstinspires.ftc.teamcode.Swerve.Core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Justin on 10/7/2016.
 * This class is an FTC wrapper of an FRC swerve drive which provides all the interfacing functionality
 */
public class FTCSwerve {
    private final int WHEEL_DIAMETER=3;//inches
    private final double GEAR_RATIO=2;
    private final int COUNTS_PER_REV=1120;
    private double startPosition=0;
    DcMotor motor;

    SwerveDrive swerveDrive;
    public FTCSwerve(AnalogInput frontLeft,AnalogInput frontRight, AnalogInput backLeft, AnalogInput backRight,
                     DcMotor lf,DcMotor rf,DcMotor lb,DcMotor rb,
                     Servo frontLeftServo,Servo frontRightServo,Servo backLeftServo,Servo backRightServo,double width,double length){
        this.motor=lf;
        swerveDrive=new SwerveDrive(frontLeft,frontRight,backLeft,backRight,//encoders
                                    lf,rf,lb,rb,//motors
                                    frontLeftServo,frontRightServo,backLeftServo,backRightServo,width,length);
    }

    /**
     *
     * @param direction the direction the robot moves, origin is the middle of the robot. x is right y is up
     * @param powerScale a scalar multiple which changes robot speed, must be <=1
     */
//    public void translate(Vector direction,double powerScale){
//        swerveDrive.translate(direction,powerScale);
//    }

    /**
     *
     *@param power negative is counterclockwise, positive is clockwise
     */
//    public void rotate(double power){
//        swerveDrive.rotate(power);
//    }


    /**
     *
     * @return  number of inches of displacement since last resetPosition, only use with linear motion
     */
    public double getLinearInchesTravelled(){
        double deltaCounts=Math.abs(motor.getCurrentPosition()-startPosition);
        double circumference=Math.PI*WHEEL_DIAMETER;
        double revolutions=deltaCounts/COUNTS_PER_REV;
        double distance=revolutions*circumference*GEAR_RATIO;
        return distance;
    }

    public void stop(){
        swerveDrive.stop();
    }

    public void lockWheels(){
        swerveDrive.lockWheels();
    }
    /**
     * Resets the displacement of the robot to 0. getLinearInchesTravelled will again return 0
     */
    public void resetPosition(){
        startPosition=motor.getCurrentPosition();
    }

    /**
     * Call this method every loop iteration
     * @param waitForServos when true the robot doesn't move the motors until the servos are in the correct position,
     *                      should only be used during autonomous
     */
    public void update(boolean waitForServos,double threshold){
        swerveDrive.update(waitForServos,threshold);
    }


    public void drive(double translationX,double translationY,double rotation,double powerscale){
        swerveDrive.driveWithOrient(-translationX,translationY,-rotation,0,powerscale);
    }

}
