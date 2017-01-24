package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hunai on 1/23/2017.
 */
@TeleOp
public class RandoServoTest extends Robot{
    double capLeftChange, capRightChange;
    double lastSet;
    public void init(){
        super.init();
        //capleftout=1
        //caprigtout=.4
        //capleftin=.7
        //caprightin=.7
        //capleftclose=.9
        //caprightclose=.6

        capLeftChange=.5;
        capRightChange=.5;
    }
    public void loop(){
        if(System.currentTimeMillis()-lastSet>500) {
            if (gamepad1.a) {
                capLeftChange += .1;
                lastSet = System.currentTimeMillis();
            }
            if (gamepad1.b) {
                capLeftChange -= .1;
                lastSet = System.currentTimeMillis();
            }
            if (gamepad1.x) {
                capRightChange += .1;
                lastSet = System.currentTimeMillis();
            }
            if (gamepad1.y) {
                capRightChange -= .1;
                lastSet = System.currentTimeMillis();
            }
            if (capRightChange < 0) {
                capRightChange = 0;
                lastSet = System.currentTimeMillis();
            }
            if (capLeftChange < 0) {
                capLeftChange = 0;
                lastSet = System.currentTimeMillis();
            }
            if (capRightChange > 1) {
                capRightChange = 1;
                lastSet = System.currentTimeMillis();
            }
            if (capLeftChange > 1) {
                capLeftChange = 1;
                lastSet = System.currentTimeMillis();
            }
            telemetry.addData("capRigtPosition: ", capRight.getPosition());
            telemetry.addData("capLeftPosition: ", capLeft.getPosition());
            capRight.setPosition(capRightChange);
            capLeft.setPosition(capLeftChange);
        }
    }
    public void stop() {

    }
}
