package org.firstinspires.ftc.teamcode.Logging;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Created by Justin on 11/10/2016.
 */
public class DataPoint {
    public double[] getJoystickValues() {
        return joystickValues;
    }

    public double[] getServoPositions() {
        return servoPositions;
    }

    public double[] getMotorPowers() {
        return motorPowers;
    }

    public HashMap<String,double[]> getVuforiaData() {
        return vuforiaData;
    }

    public double getNeckposition() {
        return neckposition;
    }

    public int[] getEncoderLocations() {
        return encoderLocations;
    }

    private double[] joystickValues=new double[8];
    private double[] servoPositions=new double[4];
    private double[] motorPowers= new double[4];
    private HashMap<String,double[]> vuforiaData;
    private double neckposition;
    private int[] encoderLocations=new int[4];


    public DataPoint(double[] joystickValues, double[] servoPositions, double[] motorPowers, HashMap<String,double[]> vuforiaData, double neckposition
    ,int[] encoderLocations){
        this.joystickValues=joystickValues;
        this.servoPositions=servoPositions;
        this.motorPowers=motorPowers;
        this.vuforiaData=vuforiaData;
        this.neckposition=neckposition;
        this.encoderLocations=encoderLocations;
    }


    public String toString(){
        String results="";
        for(double d:joystickValues){
            results+=Double.toString(d)+";";
        }
        for(double d:servoPositions){
            results+=Double.toString(d)+";";
        }
        for(double d:motorPowers){
            results+=Double.toString(d)+";";
        }
        for(String s:getTargets(vuforiaData)){
        }
        return "";
    }

    public ArrayList<String> getTargets(HashMap<String,double[]> data){
        ArrayList<String> s=new ArrayList<>();
        if(data.containsKey("Tools")) s.add("Tools");
        if(data.containsKey("Wheels")) s.add("Wheels");
        if(data.containsKey("Legos")) s.add("Legos");
        if(data.containsKey("Gears")) s.add("Gears");
        return s;
    }
}
