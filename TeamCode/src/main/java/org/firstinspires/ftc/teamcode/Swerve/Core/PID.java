package org.firstinspires.ftc.teamcode.Swerve.Core;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hunaid on 9/30/2016.
 */
public class PID {
    double P_Gain=0;
    double I_Gain=0;
    double D_Gain=0;
    double A_Gain=0;


    double time;
    double integral=0;

    double lastpow=0;
    double deriv=0;
    double previousError=0;
    double dt;
    int[] positionHistory=new int[3];
    long[] positionTimes=new long[3];

    private DcMotor driveMotor;

    public PID(double P, double I, double D,DcMotor motor,double a){
        positionHistory=new int[]{motor.getCurrentPosition(),motor.getCurrentPosition(),motor.getCurrentPosition()};
        positionTimes=new long[]{System.currentTimeMillis(),System.currentTimeMillis()+1,System.currentTimeMillis()+2};
        driveMotor=motor;
        P_Gain = P;
        I_Gain = I;
        A_Gain=a;
        D_Gain = D;
        time=System.currentTimeMillis();
    }
    public double setPIDpower(double error){
        //update history of motor position and time taken
        for(int i=0;i<2;i++){
            positionHistory[i]=positionHistory[i+1];
            positionTimes[i]=positionTimes[i+1];
        }
        positionHistory[2]=driveMotor.getCurrentPosition();
        positionTimes[2]=System.currentTimeMillis();

        //calculate dposition/dt
        double[] v=new double[]{(positionHistory[1]-positionHistory[0])/(positionTimes[1]-positionTimes[0]),
                (positionHistory[2]-positionHistory[1])/(positionTimes[2]-positionTimes[1])};
        //calculate "intermediate" time for upcoming acceleration calculation
        double[] intermediateTimes=new double[]{(positionTimes[0]+positionTimes[1])/2,(positionTimes[1]+positionTimes[2])/2};
        //calculate dv/dt
        double a=(v[1]-v[0])/(intermediateTimes[1]-intermediateTimes[0]);//acceleration!

        Log.d("Acceleration",Double.toString(a));
        dt=(System.currentTimeMillis())-time;//change in time
        time=System.currentTimeMillis();//reset "last" time

        if(Math.abs(error)<Math.toRadians(10)){
            integral+=(error*dt/1000.0);
            if(integral>1){
                integral=1;
            }else if(integral<-1){
                integral=-1;
            }
        }else{
            integral=0;
        }

        deriv=((error-previousError)*1000.0)/dt;
        previousError=error;

        Log.d("Derivative",Double.toString(deriv));
        lastpow=(error*P_Gain)+(deriv*D_Gain)+(integral*I_Gain)-(a*A_Gain);
        Log.d("Power",Double.toString(lastpow));

        return lastpow;
    }
}