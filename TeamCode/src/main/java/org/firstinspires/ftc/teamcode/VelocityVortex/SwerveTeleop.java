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

    long lastSet,lastPush;

    Vector direction=new Vector(0,1);
    double power=0;
    boolean wheelIn=true;


    @Override
    public void init() {
        super.init();
        lastSet=System.currentTimeMillis();
        lastPush=System.currentTimeMillis();
        buttonWheel.setPosition(WHEEL_OUT);
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("left",capLeft.getPosition());
        telemetry.addData("right",capRight.getPosition());
        //drive ========================================================================================================
        Vector d=new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
        if(gamepad1.left_bumper){
            if (d.getMagnitude() > .1 || Math.abs(gamepad1.right_stick_x) > .1) {
                direction = d;
                swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, .3);
            } else {
                swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, 0);
                //            swerveDrive.stop();
            }
        }else {
            if (d.getMagnitude() > .1 || Math.abs(gamepad1.right_stick_x) > .1) {
                direction = d;
                swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, .75);
            } else {
                swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, 0);
                //            swerveDrive.stop();
            }
        }

        swerveDrive.update(true,30,true);

        //========================================================================================================



        //shooter servo===========================================================================================
        if(gamepad1.right_bumper){
            shootServo.setPosition(SHOOTER_UP);
            lastPush=System.currentTimeMillis();
        }else{
            shootServo.setPosition(SHOOTER_DOWN);
        }
//        if(System.currentTimeMillis()-lastPush>400){
//            shootServo.setPosition(SHOOTER_UP);
//        }
        //========================================================================================================

        //slide========================================================================================================
        slideMotor.setPower(-gamepad2.right_stick_y);
        //        if (-gamepad2.right_stick_y<-.1){
//            if(slideMotor.getCurrentPosition()-slideStartPosition>SLIDE_DOWN){
//                slideMotor.setPower(-gamepad2.right_stick_y);
//            }else{
//                slideMotor.setPower(0);
//            }
//        }else if(-gamepad2.right_stick_y>.1){
//            if(slideMotor.getCurrentPosition()-slideStartPosition<SLIDE_UP){
//                slideMotor.setPower(-gamepad2.right_stick_y);
//            }else{
//                slideMotor.setPower(0);
//            }
//        }else{
//            slideMotor.setPower(0);
//        }
        //========================================================================================================


        //fly wheel speed ========================================================================================================
        if(System.currentTimeMillis()-lastSet>250){
//            if(gamepad2.x){ //decrement power by pressing X
//                power-=.05;
//                if(power<.5){
//                    power=.5;
//                }
//                lastSet=System.currentTimeMillis();
//            }
//            if(gamepad2.y) {//increment power by pressing Y
//                power += .05;
//                if (power > .85) {
//                    power = .85;
//                }
//                lastSet=System.currentTimeMillis();
//            }
//            if (gamepad2.b) {
//                power = 0;
//            }
//            if (gamepad2.a) {
//                power = .75;
//            }
            if(gamepad1.a){
                power=.75;
            }else if(gamepad1.b){
                power=0;
            }
            //button pusher==========================================================================================================
            if(gamepad2.left_bumper&&wheelIn){
                buttonWheel.setPosition(WHEEL_OUT);
                wheelIn=false;
            }
            if(gamepad2.left_bumper&&!wheelIn){
                buttonWheel.setPosition(WHEEL_IN);
                wheelIn=true;
            }
        }
        shootLeft.setPower(power);
        shootRight.setPower(power);
        //========================================================================================================

        // cap ball collection========================================================================================================
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
        //========================================================================================================

        //intake==========================================================================================================
        sweeper.setPower(gamepad2.left_stick_y);
        //==========================================================================================================
        telemetry.addData("ShooterPower",power);
    }
    public void stop(){
        super.stop();
    }
}
