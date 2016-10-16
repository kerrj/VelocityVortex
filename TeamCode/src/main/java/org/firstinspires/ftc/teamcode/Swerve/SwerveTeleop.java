package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

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
        Vector direction=new Vector(gamepad1.left_stick_y,gamepad1.left_stick_x);
        if(direction.getMagnitude()>.1){
            swerveDrive.translate(direction,direction.getMagnitude());
        }else{
            swerveDrive.stop();
        }

        if(Math.abs(gamepad1.right_stick_x)>.1){
            swerveDrive.rotate(gamepad1.right_stick_x);
        }
        swerveDrive.update(false);
    }
}
