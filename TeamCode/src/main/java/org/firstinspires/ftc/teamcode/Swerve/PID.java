package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hunai on 9/30/2016.
 */
public class PID {
    DcMotor right, left;
    double P_Gain=.1;
    double I_Gain=0;
    double D_Gain=0;
    final double RAMP=.5;
    double time;
    double integral=0;
    double target=96;
    double last;
    double lastpow=0;
    int init;
    final double diameter=4.0;
    double deriv;
    double pos;
    double error;
    double dt;

    public PID(double P, double I, double D){
        P_Gain = P;
        I_Gain = I;
        D_Gain = D;
    }
    public double getDistance(int count) {
        double circum=Math.PI*diameter;
        double rotation=count/1120.0;
        return circum*rotation;
    }
    public double Proportion(double target){
        pos=getDistance(right.getCurrentPosition()-init);
        error=target-pos;
        return error;
    }

    public double Derivative(double target){
        dt=(System.currentTimeMillis()-time);
        time=System.currentTimeMillis();
        pos=getDistance(right.getCurrentPosition()-init);
        error=target-pos;
        if(Math.abs(error)<.5){
            deriv=0;
        }
        else{
            deriv=(error)/dt;
        }
        return deriv;
    }
    public double Integral(double target){
        dt=(System.currentTimeMillis()-time);
        time=System.currentTimeMillis();
        pos=getDistance(right.getCurrentPosition()-init);
        error=target-pos;
        if(Math.abs(error)<10){
            integral=integral+(error*dt);
        }
        return integral;
    }
    public double setPIDpower(double target){
        lastpow=(Proportion(target)*P_Gain)+(Derivative(target)*D_Gain)+(Integral(target)*I_Gain);
        return lastpow;
    }

}
