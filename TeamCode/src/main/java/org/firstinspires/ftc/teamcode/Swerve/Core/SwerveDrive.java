package org.firstinspires.ftc.teamcode.Swerve.Core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hunai on 9/14/2016.
 * @author duncan
 */

public class SwerveDrive {
    DcMotor left, right;
    //Add values when swerve is done and also look at what absolute encoder is for
    ServoModule[] modules;
    private double leftTargetPower=0;
    private double rightTargetPower=0;

    /**
     * Custom constructor for current robot.
     */
    public SwerveDrive(AnalogInput frontLeft,AnalogInput frontRight, AnalogInput backLeft, AnalogInput backRight,
                        DcMotor left,DcMotor right,
                        Servo frontleftServo,Servo frontRightServo,Servo backLeftServo,Servo backRightServo){
        //initialize array of modules
        //array can be any size, as long as the position of each module is specified in its constructor
        modules = new ServoModule[] {
                //front left
                new ServoModule(frontleftServo, new AbsoluteEncoder(Constants.FL_OFFSET, frontLeft)),
                //front right
                new ServoModule(frontRightServo, new AbsoluteEncoder(Constants.FR_OFFSET, frontRight)),
                //back left
                new ServoModule(backLeftServo, new AbsoluteEncoder(Constants.BL_OFFSET, backLeft)),
                //back right
                new ServoModule(backRightServo, new AbsoluteEncoder(Constants.BR_OFFSET, backRight))
        };
    }


    public void translate(Vector direction,double power){
        leftTargetPower=power;
        rightTargetPower=power;

        //left side
        modules[0].set(direction.getAngle());
        modules[2].set(direction.getAngle());
        if(modules[0].getDelta()>Math.PI/2){
            leftTargetPower*=-1;
            modules[0].set(direction.getAngle()+Math.PI);
            modules[2].set(direction.getAngle()+Math.PI);
        }

        //right side
        modules[1].set(direction.getAngle());
        modules[3].set(direction.getAngle());
        if(modules[1].getDelta()>Math.PI/2){
            rightTargetPower*=-1;
            modules[1].set(direction.getAngle()+Math.PI);
            modules[3].set(direction.getAngle()+Math.PI);
        }
    }

    public void rotate(double power){
        leftTargetPower=power;
        rightTargetPower=-power;//intentionally negative

        //left side
        modules[0].set(Math.PI/4);
        modules[2].set(3*Math.PI/4);
        if(modules[0].getDelta()>Math.PI/2){
            leftTargetPower*=-1;
            modules[0].set(Math.PI/4+Math.PI);
            modules[2].set(3*Math.PI/4+Math.PI);
        }

        //right side
        modules[1].set(7*Math.PI/4);
        modules[3].set(5*Math.PI/4);
        if(modules[1].getDelta()>Math.PI/2){
            rightTargetPower*=-1;
            modules[1].set(7*Math.PI/4+Math.PI);
            modules[3].set(5*Math.PI/4+Math.PI);
        }
    }

    /**
     * Method called every loop iteration
     */
    public void update(boolean waitForServos){
        for (ServoModule module : modules) {
            module.update();
        }
        if(!waitForServos) {
            left.setPower(leftTargetPower);
            right.setPower(rightTargetPower);
        }else{
            if(modules[0].getDelta()<.25&&modules[1].getDelta()<.25&&modules[2].getDelta()<.25&& modules[3].getDelta()<.25){
                left.setPower(leftTargetPower);
                right.setPower(rightTargetPower);
            }else{
                left.setPower(0);
                right.setPower(0);
            }
        }
    }

    public void stop(){
        leftTargetPower=0;
        rightTargetPower=0;
    }

}


