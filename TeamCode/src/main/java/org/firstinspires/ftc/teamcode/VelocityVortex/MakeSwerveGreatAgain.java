package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.*;

/**
 * Created by Justin on 11/24/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Disabled
public class MakeSwerveGreatAgain extends Robot {

    public void init(){
        super.init();
    }
    public void init_loop(){
        swerveDrive.drive(0,1,0,0);
        swerveDrive.update(true,15,true);
    }
    public void loop(){
        super.loop();
        if(gamepad1.a){
            //floor it
            swerveDrive.drive(0,1,0,1);
            swerveDrive.update(true,15,false);
        }else{
            swerveDrive.drive(0,1,0,0);
            swerveDrive.update(true,15,false);
        }
    }
    public void stop(){
        super.stop();
    }

}
