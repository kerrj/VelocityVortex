package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Justin on 11/13/2016.
 */
@TeleOp
public class DigitalTest extends Robot {
    long last;
    double pos=.5;

    public void init(){
        super.init();
        last=System.currentTimeMillis();
    }
    public void loop(){
        if(System.currentTimeMillis()-last>200){
            if(gamepad1.a){
                last=System.currentTimeMillis();
                pos+=.01;
                if(pos>1){
                    pos=1;
                }
            }
            if(gamepad1.b){
                last=System.currentTimeMillis();
                pos-=.01;
                if(pos<0){
                    pos=0;
                }
            }

        }
        rf.setPosition(pos);
        rb.setPosition(pos);
        telemetry.addData("power",rf.getPosition());
    }
    public void stop(){
        super.stop();
    }
}
