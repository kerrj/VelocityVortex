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
    double power=.75;


    @Override
    public void init() {
        super.init();
        lastSet=System.currentTimeMillis();
        lastPush=System.currentTimeMillis();

    }

    @Override
    public void loop() {
        super.loop();
        Vector d=new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
        if(d.getMagnitude()>.1||Math.abs(gamepad1.right_stick_x)>.1) {
            direction=d;
            swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, 1);
        }else{
            swerveDrive.drive(direction.x, direction.y, gamepad1.right_stick_x, 0);
        }

        swerveDrive.update(true,25);


        if (gamepad1.dpad_up) {
            if(slideMotor.getCurrentPosition()-slideStartPosition<SLIDE_UP) {
                slideMotor.setPower(1);
            }else{
                slideMotor.setPower(0);
            }
        }else if(gamepad1.dpad_down){
            if(slideMotor.getCurrentPosition()-slideStartPosition>SLIDE_DOWN){
                slideMotor.setPower(-1);
            }else{
                slideMotor.setPower(0);
            }
        }else{
            slideMotor.setPower(0);
        }
        if(System.currentTimeMillis()-lastSet>200){
            if(gamepad1.x){ //decrement power by pressing X
                power-=.05;
                if(power<.7){
                    power=.7;
                }
                lastSet=System.currentTimeMillis();
            }
            if(gamepad1.y) {//increment power by pressing Y
                power += .05;
                if (power > .85) {
                    power = .85;
                }
                lastSet=System.currentTimeMillis();
            }
            if (gamepad1.b) {
                power = 0;
            }
            if (gamepad1.a) {
                power = .75;
            }
        }
        shootLeft.setPower(power);
        shootRight.setPower(power);
//        telemetry.addData("shooterPower",power);
//        telemetry.addData("slidePower",slideMotor.getPower());
//        telemetry.addData("slidePosition",slideMotor.getCurrentPosition());
    }
    public void stop(){
        super.stop();
    }
}
