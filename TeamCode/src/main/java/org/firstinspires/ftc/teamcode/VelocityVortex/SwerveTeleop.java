package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;
import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;

import java.util.Random;

/**
 * Created by Justin on 10/14/2016.
 */
@TeleOp
public class SwerveTeleop extends Robot {

    Vector direction=new Vector(0,1);

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        Vector d=new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
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
        if(d.getMagnitude()>.1||Math.abs(gamepad1.right_stick_x)>.1) {
            direction=d;
            swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, 1);
        }else{
            swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, 0);
        }


        if(gamepad1.a){
            swerveDrive.lockWheels();
        }
        if(gamepad1.b){
            swerveDrive.drive(0,1,0,0);
        }

        swerveDrive.update(true,45);
        telemetry.addData("digitalpower",rb.getPosition());
    }
    public void stop(){
        super.stop();
    }
}
