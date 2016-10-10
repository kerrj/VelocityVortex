package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Swerve.Vector;

import java.util.HashMap;

/**
 * Created by Justin on 10/9/2016.
 */
public class FTCTarget  {

    //---------------------------------------------------------------------------------------
    //EVERYTHING IN THIS CLASS ASSUMES A HORIZONTAL PHONE, WITH Z AXIS PARALLEL TO THE GROUND
    //---------------------------------------------------------------------------------------

    private double x,y,z,xRotation,yRotation,zRotation;
    private String name;

    /**
     *
     * @param data hashmap containing data for the given name.
     * @param name name of the image target, MUST be a key in the hashmap
     */
    public FTCTarget(HashMap<String,double[]> data,String name){
        double[] values=data.get(name);
        x=values[3];
        y=values[4];
        z=values[5];
        xRotation=values[0];
        yRotation=values[1];
        zRotation=values[2];
        this.name=name;
    }

    /**
     * Null constructor
     */
    public FTCTarget(){

    }

    /**
     * @return true if the target is populated with values
     */
    public boolean isFound(){
        if(name!=null){
            return true;
        }else{
            return false;
        }
    }

    /**
     * Assumes the phone is oriented horizontally
     * @return distance in mm from the image target, ONLY Z DIRECTION
     */
    public double getDistance(){
        return z;
    }

    /**
     *
     * @return y rotation in radians, where 0 is parallel with the robot
     */
    public double getYRotation(){
        return yRotation;
    }

    /**
     *
     * @return angle in radians [-pi,pi] to the image target, only on plane parallel to playing field
     */
    public double getAngle(){
        return Math.atan2(x,z);
    }
}
