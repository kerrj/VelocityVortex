package org.firstinspires.ftc.teamcode.VelocityVortex;

import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;

/**
 * Created by Justin on 11/18/2016.
 */
public class TeleOp extends Robot {
    Vector direction=new Vector(0,1);

    public void init(){
        super.init();

    }


    public void loop(){
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
// Raise/lower Linear Slide
        if (Math.abs(gamepad2.right_stick_y)>.1) {
            slideMotor.setPower(gamepad2.right_stick_y);
        }
// ball collection
        if (gamepad2.dpad_right) {
            capLeft.setPosition(CAP_LEFT_OUT);
            capRight.setPosition(CAP_RIGHT_OUT);
        } //set cap ball servo to out position

        else if(gamepad2.dpad_down){
            capLeft.setPosition(CAP_LEFT_HOLD);
            capRight.setPosition(CAP_RIGHT_HOLD);
        } // close cap ball servo to hold ball

        else if (gamepad2.dpad_left) {
            capLeft.setPosition(CAP_LEFT_IN);
            capRight.setPosition(CAP_RIGHT_IN);
        } // initial position for servo

        if(gamepad2.a){


        }
    }


    public void stop(){
        super.stop();
    }
}
