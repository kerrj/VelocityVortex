package org.firstinspires.ftc.teamcode.CameraStuff;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Justin on 11/3/2016.
 */
public class EyeOfSauron {
    private Servo base;
    private Servo neck;

    public EyeOfSauron(Servo base, Servo neck){
        this.base=base;
        this.neck=neck;
    }

    /**
     *
     * @param elevation between -90 and 90, where 0 is horizontal with the ground and 90 is straight up
     * @param rotation between -90 and 90, where -90 is all the way left and 90 is all the way right
     */
    public void lookAt(double elevation,double rotation){
        neck.setPosition(scale(elevation));
        base.setPosition(scale(rotation));
    }

    private double scale(double angle){
        double s=angle/180;
        s=+1;
        if(s>1){
            s=1;
        }else if(s<0){
            s=0;
        }
        return s;
    }

    private double reverseScale(double position){
        double s=position-.5;
        s*=90;
        if(s>90){
            s=90;
        }if(s<-90){
            s=-90;
        }
        return s;
    }

    public double getElevation(){
        return reverseScale(neck.getPosition());
    }
    public double getRotation(){
        return reverseScale(base.getPosition());
    }
    public void incrementElevation(double dAngle){

    }
}
