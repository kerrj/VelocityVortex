package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Justin on 10/7/2016.
 * This class is an FTC wrapper of an FRC swerve drive which provides all the interfacing functionality
 */
public class FTCSwerve {
    private final int WHEEL_DIAMETER=3;//inches
    private final double GEAR_RATIO=1.5;
    private final int COUNTS_PER_REV=1680;
    private double startPosition=0;
    private DcMotor front,back;
//    Servo frontLeftServo,frontRightServo,backLeftServo,backRightServo;
//    AnalogInput frontLeftEncoder,frontRightEncoder,backLeftEncoder,backRightEncoder;

    SwerveDrive swerveDrive;

    public FTCSwerve(AnalogInput frontLeft,AnalogInput frontRight, AnalogInput backLeft, AnalogInput backRight,
                     DcMotor front,DcMotor back,
                     Servo frontLeftServo,Servo frontRightServo,Servo backLeftServo,Servo backRightServo,
                     double robotWidth,double robotLength){
        this.front=front;
        this.back=back;
        swerveDrive=new SwerveDrive(frontLeft,frontRight,backLeft,backRight,//encoders
                                    front,back,//motors
                                    frontLeftServo,frontRightServo,backLeftServo,backRightServo,//servos
                                    robotWidth,robotLength);//width and length
        swerveDrive.setPivot(0,0);//the robot will only rotate around the origin
    }

    /**
     *
     * @param direction the direction the robot moves, origin is the middle of the robot. x is right y is up
     * @param powerScale a scalar multiple which changes robot speed, must be <=1
     */
    public void driveTowards(Vector direction,double powerScale){
        swerveDrive.driveNormal(direction.x,direction.y,0,powerScale);
    }

    /**
     *
     * @param direction counterclockwise is negative,clockwise is positive
     *@param powerScale a scalar multiple which changes robot speed, must be <=1
     */
    public void rotate(double direction,double powerScale){
        swerveDrive.driveNormal(0,0,direction,powerScale);
    }


    /**
     *
     * @return  number of inches of displacement since last resetPosition, only use with linear motion
     */
    public double getInchesTravelled(){
        double deltaCounts=front.getCurrentPosition()-startPosition;
        double circumference=Math.PI*WHEEL_DIAMETER;
        double revolutions=deltaCounts/COUNTS_PER_REV;
        double distance=revolutions*circumference;
        return distance;
    }

    /**
     * Resets the displacement of the robot to 0. getInchesTravelled will again return 0
     */
    public void resetPosition(){
        startPosition=front.getCurrentPosition();
    }

    /**
     * Call this method every loop iteration
     * @param waitForServos when true the robot doesn't move the motors until the servos are in the correct position,
     *                      should only be used during autonomous
     */
    public void update(boolean waitForServos){
        swerveDrive.update(waitForServos);
    }

}
