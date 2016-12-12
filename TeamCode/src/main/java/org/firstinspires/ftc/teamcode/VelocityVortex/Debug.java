package org.firstinspires.ftc.teamcode.VelocityVortex;

import android.mtp.MtpConstants;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;

/**
 * Created by Justin on 11/13/2016.
 */
@TeleOp
public class Debug extends Robot {
    Long lastSet;
    boolean reset=true;
    double motorPower=.05;
    public void init(){
        super.init();
        lastSet=System.currentTimeMillis();
    }
    public void loop(){
        if(reset){
            swerveDrive.resetPosition();
            reset=false;
        }
        if(swerveDrive.getLinearInchesTravelled()<12){
            swerveDrive.drive(0,1,0,.2);
        }else{
            swerveDrive.stop();
        }
        swerveDrive.update(true,15,true);
//        Vector direction=new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
//        if(direction.getMagnitude()>.01||Math.abs(gamepad1.right_stick_y)>.01){
//            swerveDrive.drive(direction.x,direction.y,0,.5);
//        }else{
//            swerveDrive.stop();
//        }
//        if(System.currentTimeMillis()-lastSet>300) {
//            if (gamepad1.a) {
//                motorPower += .01;
//                lastSet = System.currentTimeMillis();
//            }
//            if(gamepad1.b){
//                motorPower-=.01;
//                lastSet=System.currentTimeMillis();
//            }
//            if(motorPower>1){
//                motorPower=1;
//            }
//            if(motorPower<0){
//                motorPower=0;
//            }
//        }
//        swerveDrive.swerveDrive.setTurnPower(motorPower);
//        swerveDrive.update(true,10);
//        telemetry.addData("turnPower",motorPower);
    }
}
