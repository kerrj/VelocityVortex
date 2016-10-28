package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;
import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;

import java.util.Random;

/**
 * Created by Justin on 10/14/2016.
 */
@TeleOp
public class SwerveTeleop extends Robot {


    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        Vector direction=new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
//        if(direction.getMagnitude()>.1){
////            swerveDrive.translate(direction,direction.getMagnitude());
//        }else{
//            swerveDrive.stop();
//        }
//
//
//
//        if(Math.abs(gamepad1.right_stick_x)>.1){
////            swerveDrive.rotate(gamepad1.right_stick_x);
//
//        }
        if(direction.getMagnitude()>.1||Math.abs(gamepad1.right_stick_x)>.1) {
            swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, 1);
        }else{
            swerveDrive.stop();
        }


        if(gamepad1.a){
            swerveDrive.lockWheels();
        }
        if(gamepad1.b){
            swerveDrive.drive(0,1,0,0);
        }

        swerveDrive.update(true,25);
    }
}
