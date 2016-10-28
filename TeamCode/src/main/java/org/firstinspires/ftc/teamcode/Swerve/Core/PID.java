package org.firstinspires.ftc.teamcode.Swerve.Core;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hunai on 9/30/2016.
 */
public class PID {
    double P_Gain=0;
    double I_Gain=0;
    double D_Gain=0;

    double time;
    double integral=0;

    double lastpow=0;
    double deriv=0;
    double previousError=0;
    double dt;

    public PID(double P, double I, double D){
        P_Gain = P;
        I_Gain = I;
        D_Gain = D;
        time=System.currentTimeMillis();
    }
    public double setPIDpower(double error){
        dt=(System.currentTimeMillis())-time;//change in time
        time=System.currentTimeMillis();//reset "last" time

        if(Math.abs(error)<Math.toRadians(5)){
            integral+=(error*dt/1000.0);
            if(integral>1){
                integral=1;
            }else if(integral<-1){
                integral=-1;
            }
        }


        deriv=(error-previousError)/dt/1000;
        previousError=error;

        lastpow=(error*P_Gain)+(deriv*D_Gain)+(integral*I_Gain);

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return lastpow;
    }
}