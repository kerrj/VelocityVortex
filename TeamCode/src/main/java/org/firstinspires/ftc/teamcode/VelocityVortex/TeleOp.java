package org.firstinspires.ftc.teamcode.VelocityVortex;

import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;

/**
 * Created by Justin on 11/18/2016.
 */
public class TeleOp extends Robot {
    Vector direction=new Vector(0,1);
    long lastSet,lastPush;
    double power = 0;
    boolean wheelIn;

    public void init(){
        super.init();
        lastSet=System.currentTimeMillis();
        lastPush=System.currentTimeMillis();
        wheelIn=true;
    }


    public void loop(){
        super.loop();
        Vector d=new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);

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
        // Raise/lower Linear Slide
        if (gamepad2.right_stick_y>.1) {
            if(slideMotor.getCurrentPosition()<SLIDE_UP) {
                slideMotor.setPower(gamepad2.right_stick_y);
            }else{
                slideMotor.setPower(0);
            }
        }else if(gamepad2.right_stick_y<-.1){
            if(slideMotor.getCurrentPosition()>SLIDE_DOWN){
                slideMotor.setPower(gamepad2.right_stick_y);
            }else{
                slideMotor.setPower(0);
            }
        }else{
            slideMotor.setPower(0);
        }
        // cap ball collection
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


        //shooter servo
        if(gamepad2.right_bumper){
            shootServo.setPosition(SHOOTER_DOWN); //turn servo down for half a second before it reverts to original position
            lastPush=System.currentTimeMillis();
        }
        if(System.currentTimeMillis()-lastPush>300){
            shootServo.setPosition(SHOOTER_UP);
        }

        if(System.currentTimeMillis()-lastSet>200){
            if(gamepad2.x){ //decrement power by pressing X
                power-=.05;
                if(power<.7){
                    power=.7;
                }
                lastSet=System.currentTimeMillis();
            }
            if(gamepad1.y && power <= .85) {//increment power by pressing Y
                power += .05;
                if (power > .85) {
                    power = .85;
                }
                lastSet=System.currentTimeMillis();
            }
            if (gamepad2.b) {
                power = 0;
            }
            if (gamepad2.a) {
                power = .75;
            }
        }
        shootLeft.setPower(power);
        shootRight.setPower(power);

        if(gamepad2.left_bumper&&wheelIn){
            buttonWheel.setPosition(WHEEL_OUT);
            wheelIn=false;
        }
        if(gamepad2.left_bumper&&!wheelIn){
            buttonWheel.setPosition(WHEEL_IN);
            wheelIn=true;
        }
        telemetry.addData("Power",power);


    }


    public void stop(){
        super.stop();
    }
}