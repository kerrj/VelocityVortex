package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;
import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;

import java.util.Currency;
import java.util.Random;

/**
 * Created by Justin on 10/14/2016.
 */
@TeleOp
public class SwerveTeleop extends Robot {

    long lastSet,lastPush;

    Vector direction=new Vector(0,1);
    double power=0;
    boolean wheelIn=true,wheelsRotated=false;



    @Override
    public void init() {
        super.init();
        lfm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rbm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lbm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rfm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lastSet=System.currentTimeMillis();
        lastPush=System.currentTimeMillis();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("left",capLeft.getPosition());
        telemetry.addData("right",capRight.getPosition());
        //drive ========================================================================================================
        Vector d=new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
        if(gamepad1.dpad_up&&gamepad1.dpad_right){
            swerveDrive.setPivotPoint(7,7);
        }else if(gamepad1.dpad_right&&gamepad1.dpad_down){
            swerveDrive.setPivotPoint(7,-7);
        }else if(gamepad1.dpad_down&&gamepad1.dpad_left){
            swerveDrive.setPivotPoint(-7,-7);
        }else if(gamepad1.dpad_left&&gamepad1.dpad_up){
            swerveDrive.setPivotPoint(-7,7);
        }else if(gamepad1.dpad_up){
            swerveDrive.setPivotPoint(0,20);
        }else if(gamepad1.dpad_right){
            swerveDrive.setPivotPoint(20,0);
        }else if(gamepad1.dpad_down){
            swerveDrive.setPivotPoint(0,-20);
        }else if(gamepad1.dpad_left){
            swerveDrive.setPivotPoint(-20,0);
        }else{
            swerveDrive.setPivotPoint(0,0);
        }

        if(gamepad1.left_bumper){
            if (d.getMagnitude() > .02 || Math.abs(gamepad1.right_stick_x) > .02) {
                direction = d;
                swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x/1.5, .3);
                if(Math.abs(gamepad1.right_stick_x)>.02&&d.getMagnitude()<.02){
                    wheelsRotated=true;
                }else{
                    wheelsRotated=false;
                }
            } else if(wheelsRotated) {
                swerveDrive.drive(0,0,1,0);
            } else {
                swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x/1.5, 0);
            }
        }else {
            if (d.getMagnitude() > .02 || Math.abs(gamepad1.right_stick_x) > .02) {
                direction = d;
                swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x/1.5, .6);
                if(Math.abs(gamepad1.right_stick_x)>.02&&d.getMagnitude()<.02){
                    wheelsRotated=true;
                }else{
                    wheelsRotated=false;
                }
            } else if(wheelsRotated){
                swerveDrive.drive(0,0,1,0);
            }else {
                swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x/1.5, 0);
            }
        }

        swerveDrive.update(true,30,false);

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
            if(gamepad2.a){
                power=.75;
            }else if(gamepad2.b){
                power=0;
            }
            //button pusher==========================================================================================================
            if(gamepad2.left_bumper&&wheelIn){
                buttonWheel.setPosition(WHEEL_OUT);
                wheelIn=false;
                lastSet=System.currentTimeMillis();
            }
            if(gamepad2.left_bumper&&!wheelIn){
                buttonWheel.setPosition(WHEEL_IN);
                wheelIn=true;
                lastSet=System.currentTimeMillis();
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
